// Benchmark inter-camera timestamp spread between two sensor_msgs/Image streams.
// Uses message_filters::ApproximateTime. Prints rolling p50/p95/p99 of the
// pairwise spread every print_period_s seconds and writes every matched pair
// to a CSV for offline analysis.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using Image = sensor_msgs::msg::Image;
using ApproxPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;

class SyncBench : public rclcpp::Node {
public:
  SyncBench() : Node("sync_bench") {
    // Parameters
    topic_a_ = declare_parameter<std::string>("topic_a", "/cam_uvc/image_raw");
    topic_b_ = declare_parameter<std::string>("topic_b", "/camera/camera/color/image_raw");
    slop_s_ = declare_parameter<double>("slop_s", 0.034);   // 34 ms budget
    queue_size_ = declare_parameter<int>("queue_size", 30);
    print_period_s_ = declare_parameter<double>("print_period_s", 2.0);
    csv_path_ = declare_parameter<std::string>("csv_path", "/tmp/sync_bench.csv");

    RCLCPP_INFO(get_logger(), "topic_a=%s topic_b=%s slop=%.3fs queue=%d csv=%s",
                topic_a_.c_str(), topic_b_.c_str(), slop_s_, queue_size_, csv_path_.c_str());

    // Sensor data QoS: BEST_EFFORT + KEEP_LAST. Must match what cameras publish.
    auto qos = rclcpp::SensorDataQoS();

    sub_a_.subscribe(this, topic_a_, qos.get_rmw_qos_profile());
    sub_b_.subscribe(this, topic_b_, qos.get_rmw_qos_profile());

    // Count every message the synchronizer sees — not a second subscription.
    sub_a_.registerCallback([this](const Image::ConstSharedPtr&) { ++count_a_; });
    sub_b_.registerCallback([this](const Image::ConstSharedPtr&) { ++count_b_; });

    sync_ = std::make_shared<message_filters::Synchronizer<ApproxPolicy>>(
        ApproxPolicy(queue_size_), sub_a_, sub_b_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(slop_s_));
    sync_->registerCallback(std::bind(&SyncBench::cb, this,
                                      std::placeholders::_1, std::placeholders::_2));

    timer_ = create_wall_timer(
        std::chrono::duration<double>(print_period_s_),
        [this]() { report(); });

    start_ = now();
  }

  ~SyncBench() override {
    std::ofstream csv(csv_path_);
    csv << "t_wall_ns,t_a_ns,t_b_ns,delta_ms,spread_ms\n";
    for (auto& r : rows_)
      csv << r.wall_ns << ',' << r.ta_ns << ',' << r.tb_ns
          << ',' << r.delta_ms << ',' << std::abs(r.delta_ms) << '\n';
    RCLCPP_INFO(get_logger(), "final: matched=%zu cam_a=%lu cam_b=%lu csv=%s",
                rows_.size(), count_a_.load(), count_b_.load(), csv_path_.c_str());
  }

private:
  void cb(const Image::ConstSharedPtr& a, const Image::ConstSharedPtr& b) {
    const int64_t ta = static_cast<int64_t>(a->header.stamp.sec) * 1000000000LL
                     + static_cast<int64_t>(a->header.stamp.nanosec);
    const int64_t tb = static_cast<int64_t>(b->header.stamp.sec) * 1000000000LL
                     + static_cast<int64_t>(b->header.stamp.nanosec);
    // Signed: positive means A is ahead of B (UVC arrives later than RealSense)
    const double delta_ms  = (ta - tb) / 1.0e6;
    const double spread_ms = std::abs(delta_ms);

    spreads_ms_.push_back(spread_ms);
    deltas_ms_.push_back(delta_ms);
    rows_.push_back({now().nanoseconds(), ta, tb, delta_ms});
  }

  static double percentile(std::vector<double> v, double p) {
    if (v.empty()) return std::nan("");
    std::sort(v.begin(), v.end());
    const size_t idx = std::min(v.size() - 1,
                                static_cast<size_t>(std::ceil(p / 100.0 * v.size())) - 1);
    return v[idx];
  }

  void report() {
    const double elapsed = (now() - start_).seconds();
    if (spreads_ms_.empty()) {
      RCLCPP_WARN(get_logger(),
          "[%.1fs] NO MATCHES. cam_a=%lu cam_b=%lu. "
          "Check QoS, topic names, and slop (currently %.3fs).",
          elapsed, count_a_.load(), count_b_.load(), slop_s_);
      return;
    }
    // Report on the last window only, so numbers reflect current state
    std::vector<double> window(
        spreads_ms_.begin() + window_start_, spreads_ms_.end());
    std::vector<double> dwin(
        deltas_ms_.begin() + window_start_, deltas_ms_.end());

    const double p50 = percentile(window, 50);
    const double p95 = percentile(window, 95);
    const double p99 = percentile(window, 99);
    const double mx  = *std::max_element(window.begin(), window.end());

    double sum = 0, sumsq = 0;
    for (double x : dwin) { sum += x; sumsq += x * x; }
    const double mean   = sum / dwin.size();
    const double stddev = std::sqrt(sumsq / dwin.size() - mean * mean);

    RCLCPP_INFO(get_logger(),
        "[%.1fs] matched=%zu (+%zu) a=%lu b=%lu | spread ms "
        "p50=%.2f p95=%.2f p99=%.2f max=%.2f | bias=%.2f stddev=%.2f",
        elapsed, spreads_ms_.size(), window.size(),
        count_a_.load(), count_b_.load(), p50, p95, p99, mx, mean, stddev);

    window_start_ = spreads_ms_.size();
  }

  std::string topic_a_, topic_b_, csv_path_;
  double slop_s_;
  int queue_size_;
  double print_period_s_;

  message_filters::Subscriber<Image> sub_a_, sub_b_;
  std::shared_ptr<message_filters::Synchronizer<ApproxPolicy>> sync_;

  std::atomic<uint64_t> count_a_{0}, count_b_{0};

  struct Row { int64_t wall_ns, ta_ns, tb_ns; double delta_ms; };

  std::vector<double> spreads_ms_;
  std::vector<double> deltas_ms_;   // signed: ta - tb
  std::vector<Row>    rows_;
  size_t window_start_ = 0;
  rclcpp::Time start_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SyncBench>());
  rclcpp::shutdown();
  return 0;
}
