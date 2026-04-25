// Benchmark inter-camera timestamp spread for N ∈ {2,3,4} sensor_msgs/Image streams.
// N is selected at runtime from the topics parameter length; the synchronizer type
// is resolved at compile time via template specialisation.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <cam_sync_msgs/msg/synced_frames.hpp>

using Image        = sensor_msgs::msg::Image;
using SyncedFrames = cam_sync_msgs::msg::SyncedFrames;
namespace mf = message_filters;

// ── Threaded bounded-queue CSV writer ────────────────────────────────────────
// Disk I/O happens exclusively on the drain thread; the executor thread only
// enqueues a pre-formatted string. If the queue is full the row is dropped
// and dropped_ is incremented so report() can surface it.
class CsvWriter {
public:
  CsvWriter(const std::string& path, const std::string& header,
            size_t max_queue = 10000)
      : max_queue_(max_queue), dropped_(0), shutdown_(false)
  {
    file_.open(path);
    file_ << header << '\n';
    thread_ = std::thread([this]() { drain(); });
  }

  ~CsvWriter() {
    { std::unique_lock<std::mutex> lk(mu_); shutdown_ = true; }
    cv_.notify_one();
    thread_.join();
  }

  void push(std::string row) {
    std::unique_lock<std::mutex> lk(mu_);
    if (queue_.size() >= max_queue_) { ++dropped_; return; }
    queue_.push(std::move(row));
    lk.unlock();
    cv_.notify_one();
  }

  uint64_t dropped() const { return dropped_.load(std::memory_order_relaxed); }

private:
  void drain() {
    for (;;) {
      std::vector<std::string> batch;
      {
        std::unique_lock<std::mutex> lk(mu_);
        cv_.wait(lk, [this]() { return !queue_.empty() || shutdown_; });
        while (!queue_.empty()) {
          batch.push_back(std::move(queue_.front()));
          queue_.pop();
        }
      }
      for (auto& s : batch) file_ << s << '\n';
      if (!batch.empty()) file_.flush();
      {
        std::unique_lock<std::mutex> lk(mu_);
        if (shutdown_ && queue_.empty()) break;
      }
    }
  }

  size_t max_queue_;
  std::atomic<uint64_t> dropped_;
  bool shutdown_;
  std::ofstream file_;
  std::queue<std::string> queue_;
  std::mutex mu_;
  std::condition_variable cv_;
  std::thread thread_;
};

// ── ApproximateTime policy for N topics ──────────────────────────────────────
template<size_t N> struct PolicyHelper;
template<> struct PolicyHelper<2> {
  using type = mf::sync_policies::ApproximateTime<Image, Image>;
};
template<> struct PolicyHelper<3> {
  using type = mf::sync_policies::ApproximateTime<Image, Image, Image>;
};
template<> struct PolicyHelper<4> {
  using type = mf::sync_policies::ApproximateTime<Image, Image, Image, Image>;
};

template<> struct PolicyHelper<5> {
  using type = mf::sync_policies::ApproximateTime<Image, Image, Image, Image, Image>;
};

template<> struct PolicyHelper<6> {
  using type = mf::sync_policies::ApproximateTime<Image, Image, Image, Image, Image, Image>;
};

template<> struct PolicyHelper<7> {
  using type = mf::sync_policies::ApproximateTime<Image, Image, Image, Image, Image, Image, Image>;
};

template<> struct PolicyHelper<8> {
  using type = mf::sync_policies::ApproximateTime<Image, Image, Image, Image, Image, Image, Image, Image>;
};

// ── N-topic benchmark node ────────────────────────────────────────────────────
template<size_t N>
class BenchN : public rclcpp::Node {
  static_assert(N >= 2 && N <= 8, "N must be 2, 3, or 4 (or 5,6,7,8 with more policy specialisations)");
  static constexpr size_t P = N * (N - 1) / 2;   // number of unique pairs

  using Policy = typename PolicyHelper<N>::type;
  using Sync   = mf::Synchronizer<Policy>;

public:
  explicit BenchN(const std::vector<std::string>& topics)
      : Node("sync_bench"), topics_(topics)
  {
    // Declare topics so ROS2 doesn't warn about an undeclared launch parameter.
    declare_parameter<std::vector<std::string>>("topics", topics_);
    slop_s_         = declare_parameter<double>("slop_s", 0.034);
    queue_size_     = declare_parameter<int>("queue_size", 30);
    print_period_s_ = declare_parameter<double>("print_period_s", 2.0);
    csv_path_       = declare_parameter<std::string>("csv_path", "/tmp/sync_bench.csv");

    RCLCPP_INFO(get_logger(), "N=%zu slop=%.3fs queue=%d csv=%s",
                N, slop_s_, queue_size_, csv_path_.c_str());
    for (size_t i = 0; i < N; ++i)
      RCLCPP_INFO(get_logger(), "  [%zu] %s", i, topics_[i].c_str());

    auto qos = rclcpp::SensorDataQoS();
    for (size_t i = 0; i < N; ++i) {
      subs_[i].subscribe(this, topics_[i], qos.get_rmw_qos_profile());
      // Count via the sync subscriber's own pipeline — not a second subscription.
      subs_[i].registerCallback(
          [this, i](const Image::ConstSharedPtr&) { ++counts_[i]; });
    }

    pub_ = create_publisher<SyncedFrames>("synced_frames", rclcpp::SensorDataQoS());

    create_sync();
    register_callback();

    // Dynamic CSV header: t_wall_ns, t_0_ns…t_{N-1}_ns, delta_I_J_ms for each pair
    std::ostringstream hdr;
    hdr << "t_wall_ns";
    for (size_t i = 0; i < N; ++i) hdr << ",t_" << i << "_ns";
    for (size_t i = 0; i < N; ++i)
      for (size_t j = i + 1; j < N; ++j)
        hdr << ",delta_" << i << "_" << j << "_ms";
    csv_ = std::make_unique<CsvWriter>(csv_path_, hdr.str());

    timer_ = create_wall_timer(
        std::chrono::duration<double>(print_period_s_),
        [this]() { report(); });
    start_ = now();
  }

  ~BenchN() override {
    RCLCPP_INFO(get_logger(), "final: matched=%zu dropped_csv=%lu",
                spreads_.size(), csv_->dropped());
    for (size_t i = 0; i < N; ++i)
      RCLCPP_INFO(get_logger(), "  cam[%zu]=%lu", i, counts_[i].load());
  }

private:
  // N-specific: pass N subscribers to the Synchronizer constructor.
  void create_sync() {
    if constexpr (N == 2)
      sync_ = std::make_shared<Sync>(Policy(queue_size_), subs_[0], subs_[1]);
    else if constexpr (N == 3)
      sync_ = std::make_shared<Sync>(Policy(queue_size_), subs_[0], subs_[1], subs_[2]);
    else if constexpr (N == 4)
      sync_ = std::make_shared<Sync>(Policy(queue_size_), subs_[0], subs_[1], subs_[2], subs_[3]);
    else if constexpr (N == 5)
      sync_ = std::make_shared<Sync>(Policy(queue_size_), subs_[0], subs_[1], subs_[2], subs_[3], subs_[4]);
    else if constexpr (N == 6)
      sync_ = std::make_shared<Sync>(Policy(queue_size_), subs_[0], subs_[1], subs_[2], subs_[3], subs_[4], subs_[5]);
    else if constexpr (N == 7)
      sync_ = std::make_shared<Sync>(Policy(queue_size_), subs_[0], subs_[1], subs_[2], subs_[3], subs_[4], subs_[5], subs_[6]);
    else if constexpr (N == 8)
      sync_ = std::make_shared<Sync>(Policy(queue_size_), subs_[0], subs_[1], subs_[2], subs_[3], subs_[4], subs_[5], subs_[6], subs_[7]);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(slop_s_));
  }

  // Named callbacks for std::bind — Signal9::addCallback rejects raw lambdas.
  // Each is guarded with if constexpr so the array-size mismatch doesn't
  // compile for the wrong N (e.g. on_frame_3 in BenchN<2> is a no-op body).
  void on_frame_2(const Image::ConstSharedPtr& a, const Image::ConstSharedPtr& b) {
    if constexpr (N == 2) process({a, b});
  }
  void on_frame_3(const Image::ConstSharedPtr& a, const Image::ConstSharedPtr& b,
                  const Image::ConstSharedPtr& c) {
    if constexpr (N == 3) process({a, b, c});
  }
  void on_frame_4(const Image::ConstSharedPtr& a, const Image::ConstSharedPtr& b,
                  const Image::ConstSharedPtr& c, const Image::ConstSharedPtr& d) {
    if constexpr (N == 4) process({a, b, c, d});
  }
  void on_frame_5(const Image::ConstSharedPtr& a, const Image::ConstSharedPtr& b,
                  const Image::ConstSharedPtr& c, const Image::ConstSharedPtr& d,
                  const Image::ConstSharedPtr& e) {
    if constexpr (N == 5) process({a, b, c, d, e});
  }
  void on_frame_6(const Image::ConstSharedPtr& a, const Image::ConstSharedPtr& b,
                  const Image::ConstSharedPtr& c, const Image::ConstSharedPtr& d,
                  const Image::ConstSharedPtr& e, const Image::ConstSharedPtr& f) {
    if constexpr (N == 6) process({a, b, c, d, e, f});
  }
  void on_frame_7(const Image::ConstSharedPtr& a, const Image::ConstSharedPtr& b,
                  const Image::ConstSharedPtr& c, const Image::ConstSharedPtr& d,
                  const Image::ConstSharedPtr& e, const Image::ConstSharedPtr& f,
                  const Image::ConstSharedPtr& g) {
    if constexpr (N == 7) process({a, b, c, d, e, f, g});
  }
  void on_frame_8(const Image::ConstSharedPtr& a, const Image::ConstSharedPtr& b,
                  const Image::ConstSharedPtr& c, const Image::ConstSharedPtr& d,
                  const Image::ConstSharedPtr& e, const Image::ConstSharedPtr& f,
                  const Image::ConstSharedPtr& g, const Image::ConstSharedPtr& h) {
    if constexpr (N == 8) process({a, b, c, d, e, f, g, h});
  }

  void register_callback() {
    using namespace std::placeholders;
    if constexpr (N == 2)
      sync_->registerCallback(std::bind(&BenchN<N>::on_frame_2, this, _1, _2));
    else if constexpr (N == 3)
      sync_->registerCallback(std::bind(&BenchN<N>::on_frame_3, this, _1, _2, _3));
    else if constexpr (N == 4)
      sync_->registerCallback(std::bind(&BenchN<N>::on_frame_4, this, _1, _2, _3, _4));
    else if constexpr (N == 5)
      sync_->registerCallback(std::bind(&BenchN<N>::on_frame_5, this, _1, _2, _3, _4, _5));
    else if constexpr (N == 6)
      sync_->registerCallback(std::bind(&BenchN<N>::on_frame_6, this, _1, _2, _3, _4, _5, _6));
    else if constexpr (N == 7)
      sync_->registerCallback(std::bind(&BenchN<N>::on_frame_7, this, _1, _2, _3, _4, _5, _6, _7));
    else if constexpr (N == 8)
      sync_->registerCallback(std::bind(&BenchN<N>::on_frame_8, this, _1, _2, _3, _4, _5, _6, _7, _8));
  }

  // N-agnostic: all data handling below here works for any N.
  void process(const std::array<Image::ConstSharedPtr, N>& frames) {
    std::array<int64_t, N> ts;
    for (size_t i = 0; i < N; ++i)
      ts[i] = static_cast<int64_t>(frames[i]->header.stamp.sec) * 1000000000LL
             + static_cast<int64_t>(frames[i]->header.stamp.nanosec);

    std::array<double, P> deltas;
    size_t k = 0;
    double max_spread = 0.0;
    for (size_t i = 0; i < N; ++i)
      for (size_t j = i + 1; j < N; ++j) {
        deltas[k] = (ts[i] - ts[j]) / 1.0e6;   // signed ms
        max_spread = std::max(max_spread, std::abs(deltas[k]));
        ++k;
      }

    spreads_.push_back(max_spread);
    pair_deltas_.push_back(deltas);

    std::ostringstream row;
    row << now().nanoseconds();
    for (size_t i = 0; i < N; ++i) row << ',' << ts[i];
    for (size_t p = 0; p < P; ++p) row << ',' << deltas[p];
    csv_->push(row.str());

    SyncedFrames bundle;
    bundle.stamp = now();
    bundle.frames.reserve(N);
    bundle.labels.reserve(N);
    for (size_t i = 0; i < N; ++i) {
      bundle.frames.push_back(*frames[i]);
      bundle.labels.push_back(topics_[i]);
    }
    bundle.pairwise_spreads_ms.assign(deltas.begin(), deltas.end());
    pub_->publish(bundle);
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

    std::string cam_counts;
    for (size_t i = 0; i < N; ++i)
      cam_counts += "cam[" + std::to_string(i) + "]=" +
                    std::to_string(counts_[i].load()) + " ";

    if (spreads_.empty()) {
      RCLCPP_WARN(get_logger(),
          "[%.1fs] NO MATCHES. %sCheck QoS, topic names, slop=%.3fs.",
          elapsed, cam_counts.c_str(), slop_s_);
      return;
    }

    std::vector<double> win(spreads_.begin() + window_start_, spreads_.end());

    const double p50    = percentile(win, 50);
    const double p95    = percentile(win, 95);
    const double p99    = percentile(win, 99);
    const double mx     = *std::max_element(win.begin(), win.end());

    double sum = 0, sumsq = 0;
    for (double x : win) { sum += x; sumsq += x * x; }
    const double mean   = sum / win.size();
    const double stddev = std::sqrt(sumsq / win.size() - mean * mean);

    RCLCPP_INFO(get_logger(),
        "[%.1fs] matched=%zu (+%zu) %s"
        "| max-pair spread ms p50=%.2f p95=%.2f p99=%.2f max=%.2f "
        "mean=%.2f stddev=%.2f | dropped_csv=%lu",
        elapsed, spreads_.size(), win.size(), cam_counts.c_str(),
        p50, p95, p99, mx, mean, stddev, csv_->dropped());

    // Per-pair means help identify which pair is the bottleneck.
    for (size_t p = 0, i = 0; i < N; ++i)
      for (size_t j = i + 1; j < N; ++j, ++p) {
        double psum = 0;
        for (size_t r = window_start_; r < pair_deltas_.size(); ++r)
          psum += pair_deltas_[r][p];
        RCLCPP_INFO(get_logger(),
            "    pair[%zu,%zu] mean=%.2f ms", i, j, psum / win.size());
      }

    window_start_ = spreads_.size();
  }

  std::vector<std::string> topics_;
  double slop_s_;
  int queue_size_;
  double print_period_s_;
  std::string csv_path_;

  rclcpp::Publisher<SyncedFrames>::SharedPtr pub_;
  std::array<mf::Subscriber<Image>, N> subs_;
  std::shared_ptr<Sync> sync_;
  std::unique_ptr<CsvWriter> csv_;

  std::array<std::atomic<uint64_t>, N> counts_{};

  std::vector<double>                 spreads_;      // max pairwise |delta| per match
  std::vector<std::array<double, P>>  pair_deltas_;  // signed per-pair deltas
  size_t window_start_ = 0;

  rclcpp::Time start_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// ── main: bootstrap-read topics, dispatch to correct template ─────────────────
// Two nodes named "sync_bench" are created sequentially (not simultaneously).
// The bootstrap uses automatically_declare_parameters_from_overrides so it picks
// up the topics list from the launch file without knowing its type in advance.
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  std::vector<std::string> topics;
  {
    rclcpp::NodeOptions opts;
    opts.automatically_declare_parameters_from_overrides(true);
    auto tmp = std::make_shared<rclcpp::Node>("sync_bench", opts);
    if (tmp->has_parameter("topics"))
      topics = tmp->get_parameter("topics").as_string_array();
    else
      topics = {"/cam_uvc/image_raw", "/camera/camera/color/image_raw"};
  }  // bootstrap destroyed before BenchN is created

  std::shared_ptr<rclcpp::Node> node;
  switch (topics.size()) {
    case 2: node = std::make_shared<BenchN<2>>(topics); break;
    case 3: node = std::make_shared<BenchN<3>>(topics); break;
    case 4: node = std::make_shared<BenchN<4>>(topics); break;
    case 5: node = std::make_shared<BenchN<5>>(topics); break;
    case 6: node = std::make_shared<BenchN<6>>(topics); break;
    case 7: node = std::make_shared<BenchN<7>>(topics); break;
    case 8: node = std::make_shared<BenchN<8>>(topics); break;
    default:
      RCLCPP_FATAL(rclcpp::get_logger("sync_bench"),
          "topics must have 2–8 entries (got %zu)", topics.size());
      rclcpp::shutdown();
      return 1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
