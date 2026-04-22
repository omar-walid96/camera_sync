// Benchmark inter-camera timestamp spread for N configurable topics.
// Each topic gets an independent subscription + rolling frame buffer.
// Matching: greedy nearest-neighbour from the newest anchor frame (topic[0])
// within slop_s. Reports per-pair p50/p95/p99 every print_period_s seconds
// and writes every matched set to CSV.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

using Image = sensor_msgs::msg::Image;

class SyncBench : public rclcpp::Node {
  struct FrameBuf {
    std::deque<std::pair<int64_t, Image::ConstSharedPtr>> q;
    size_t total = 0;
  };

public:
  SyncBench() : Node("sync_bench") {
    topics_ = declare_parameter<std::vector<std::string>>("topics",
        {"/cam_uvc/image_raw", "/camera/camera/color/image_raw"});
    auto lp  = declare_parameter<std::vector<std::string>>("labels", std::vector<std::string>{});
    slop_ns_ = static_cast<int64_t>(
        declare_parameter<double>("slop_s", 0.034) * 1e9);
    queue_size_     = declare_parameter<int>   ("queue_size",     30);
    print_period_s_ = declare_parameter<double>("print_period_s", 2.0);
    csv_path_       = declare_parameter<std::string>("csv_path", "/tmp/sync_bench.csv");

    N_ = topics_.size();
    if (N_ < 2) throw std::runtime_error("Need at least 2 topics.");

    labels_.resize(N_);
    for (size_t i = 0; i < N_; ++i)
      labels_[i] = (i < lp.size()) ? lp[i] : topics_[i];

    bufs_.resize(N_);
    auto qos = rclcpp::SensorDataQoS();
    for (size_t i = 0; i < N_; ++i) {
      subs_.push_back(create_subscription<Image>(topics_[i], qos,
          [this, i](const Image::ConstSharedPtr& msg) { on_frame(i, msg); }));
      RCLCPP_INFO(get_logger(), "  [%zu] %s (%s)", i, topics_[i].c_str(), labels_[i].c_str());
    }

    // CSV header
    csv_.open(csv_path_);
    csv_ << "t_wall_ns";
    for (size_t i = 0; i < N_; ++i) csv_ << ",t_" << i << "_ns";
    csv_ << ",max_spread_ms\n";

    start_ = now();
    timer_ = create_wall_timer(
        std::chrono::duration<double>(print_period_s_),
        [this]() { report(); });

    RCLCPP_INFO(get_logger(), "SyncBench ready: %zu topics, slop=%.3fms, csv=%s",
                N_, slop_ns_ / 1e6, csv_path_.c_str());
  }

  ~SyncBench() override {
    if (csv_.is_open()) csv_.close();
    std::vector<size_t> counts(N_);
    for (size_t i = 0; i < N_; ++i) counts[i] = bufs_[i].total;
    RCLCPP_INFO(get_logger(), "final: matched=%zu", matched_);
  }

private:
  static int64_t stamp_ns(const Image::ConstSharedPtr& m) {
    return static_cast<int64_t>(m->header.stamp.sec) * 1000000000LL
         + static_cast<int64_t>(m->header.stamp.nanosec);
  }

  void on_frame(size_t i, const Image::ConstSharedPtr& msg) {
    auto& b = bufs_[i];
    b.q.push_back({stamp_ns(msg), msg});
    if (b.q.size() > static_cast<size_t>(queue_size_)) b.q.pop_front();
    ++b.total;
    try_match();
  }

  void try_match() {
    // Iterate anchor topic (0) newest→oldest; stop below last emitted ts.
    for (auto it = bufs_[0].q.rbegin(); it != bufs_[0].q.rend(); ++it) {
      const int64_t anchor = it->first;
      if (anchor <= last_match_ts_) break;

      std::vector<int64_t> chosen_ts(N_);
      chosen_ts[0] = anchor;
      bool ok = true;

      for (size_t j = 1; j < N_; ++j) {
        int64_t best_diff = slop_ns_ + 1;
        int64_t best_ts   = 0;
        for (auto& f : bufs_[j].q) {
          int64_t diff = std::abs(f.first - anchor);
          if (diff < best_diff) { best_diff = diff; best_ts = f.first; }
        }
        if (best_diff > slop_ns_) { ok = false; break; }
        chosen_ts[j] = best_ts;
      }

      if (!ok) continue;

      // All pairs within slop — emit
      double max_spread = 0.0;
      for (size_t i = 0; i < N_; ++i)
        for (size_t j = i + 1; j < N_; ++j) {
          double s = std::abs(chosen_ts[i] - chosen_ts[j]) / 1.0e6;
          pair_spreads_[{i, j}].push_back(s);
          max_spread = std::max(max_spread, s);
        }
      max_spreads_.push_back(max_spread);

      const auto wall = now().nanoseconds();
      csv_ << wall;
      for (int64_t ts : chosen_ts) csv_ << ',' << ts;
      csv_ << ',' << max_spread << '\n';

      last_match_ts_ = anchor;
      ++matched_;
      return;
    }
  }

  static double percentile(std::vector<double> v, double p) {
    if (v.empty()) return std::nan("");
    std::sort(v.begin(), v.end());
    size_t idx = std::min(v.size() - 1,
        static_cast<size_t>(std::ceil(p / 100.0 * v.size())) - 1);
    return v[idx];
  }

  void report() {
    const double elapsed = (now() - start_).seconds();

    if (max_spreads_.empty()) {
      std::string counts;
      for (size_t i = 0; i < N_; ++i)
        counts += labels_[i] + "=" + std::to_string(bufs_[i].total) + " ";
      RCLCPP_WARN(get_logger(), "[%.1fs] NO MATCHES. %sslop=%.3fms",
                  elapsed, counts.c_str(), slop_ns_ / 1e6);
      return;
    }

    // Window since last report
    std::vector<double> window_max(
        max_spreads_.begin() + window_start_, max_spreads_.end());
    RCLCPP_INFO(get_logger(),
        "[%.1fs] matched=%zu (+%zu) | overall max_spread p50=%.2f p95=%.2f p99=%.2f ms",
        elapsed, matched_, window_max.size(),
        percentile(window_max, 50), percentile(window_max, 95), percentile(window_max, 99));

    for (auto& [key, vec] : pair_spreads_) {
      std::vector<double> w(vec.begin() + pair_window_start_[key], vec.end());
      if (w.empty()) continue;
      RCLCPP_INFO(get_logger(), "    %s <-> %s  p50=%.2f p95=%.2f p99=%.2f ms",
                  labels_[key.first].c_str(), labels_[key.second].c_str(),
                  percentile(w, 50), percentile(w, 95), percentile(w, 99));
      pair_window_start_[key] = vec.size();
    }

    window_start_ = max_spreads_.size();
  }

  // ── state ────────────────────────────────────────────────────────────────
  size_t N_;
  std::vector<std::string> topics_, labels_;
  int64_t slop_ns_;
  int queue_size_;
  double print_period_s_;
  std::string csv_path_;

  std::vector<rclcpp::Subscription<Image>::SharedPtr> subs_;
  std::vector<FrameBuf> bufs_;

  int64_t last_match_ts_ = 0;
  size_t matched_        = 0;

  std::vector<double> max_spreads_;
  size_t window_start_ = 0;

  std::map<std::pair<size_t,size_t>, std::vector<double>> pair_spreads_;
  std::map<std::pair<size_t,size_t>, size_t>              pair_window_start_;

  rclcpp::Time start_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::ofstream csv_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SyncBench>());
  rclcpp::shutdown();
  return 0;
}
