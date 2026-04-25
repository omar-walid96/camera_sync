// Viewer for /synced_frames (cam_sync_msgs/SyncedFrames).
// Single subscription replaces 6 individual topic subscriptions.
// Grid layout adapts to N: 1×N for N≤3, 2×2 for N=4.

#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cam_sync_msgs/msg/synced_frames.hpp>

using SyncedFrames = cam_sync_msgs::msg::SyncedFrames;
using Image        = sensor_msgs::msg::Image;
using Clock        = std::chrono::steady_clock;

// ── Rolling FPS + bandwidth tracker ─────────────────────────────────────────
struct StreamTracker {
  explicit StreamTracker(double window_s = 2.0) : window_s_(window_s) {}

  void tick(size_t bytes) {
    auto now = Clock::now();
    samples_.push_back({now, bytes});
    while (!samples_.empty() &&
           std::chrono::duration<double>(now - samples_.front().first).count() > window_s_)
      samples_.pop_front();
  }

  double fps() const {
    if (samples_.size() < 2) return 0.0;
    double dt = std::chrono::duration<double>(
        samples_.back().first - samples_.front().first).count();
    return dt > 0.0 ? (samples_.size() - 1) / dt : 0.0;
  }

  double mbps() const {
    if (samples_.size() < 2) return 0.0;
    double total = 0.0;
    for (auto& s : samples_) total += static_cast<double>(s.second);
    double dt = std::chrono::duration<double>(
        samples_.back().first - samples_.front().first).count();
    return dt > 0.0 ? (total / dt) / 1e6 : 0.0;
  }

private:
  double window_s_;
  std::deque<std::pair<Clock::time_point, size_t>> samples_;
};

// ── Overlay helpers ──────────────────────────────────────────────────────────
static void put_shadow(cv::Mat& frame, const std::string& txt,
                       cv::Point pos, cv::Scalar color,
                       double scale = 0.50, int thick = 1)
{
  cv::putText(frame, txt, pos + cv::Point(1, 1),
              cv::FONT_HERSHEY_SIMPLEX, scale, {0, 0, 0}, thick + 1);
  cv::putText(frame, txt, pos,
              cv::FONT_HERSHEY_SIMPLEX, scale, color, thick);
}

static cv::Scalar spread_color(double ms) {
  if (ms < 17.0) return {80, 220, 80};
  if (ms < 34.0) return {40, 200, 220};
  return {60, 60, 220};
}

// pairs: list of (other_label, signed_delta_ms) for every peer of this camera.
static void draw_overlay(cv::Mat& panel, const std::string& label,
                         double stamp_s, double fps, double mbps,
                         const std::vector<std::pair<std::string, double>>& pairs)
{
  char buf[128];
  int y = 20;
  std::snprintf(buf, sizeof(buf), "%s  %.1f fps  %.1f MB/s",
                label.c_str(), fps, mbps);
  put_shadow(panel, buf, {8, y}, {255, 255, 255}); y += 20;

  std::snprintf(buf, sizeof(buf), "t = %.3f s", stamp_s);
  put_shadow(panel, buf, {8, y}, {200, 200, 200}); y += 20;

  if (pairs.empty()) {
    put_shadow(panel, "anchor", {8, y}, {180, 180, 180});
  } else {
    for (const auto& [peer, delta] : pairs) {
      const double abs_ms = std::abs(delta);
      std::snprintf(buf, sizeof(buf), "vs %s  %+.1f ms", peer.c_str(), delta);
      put_shadow(panel, buf, {8, y}, spread_color(abs_ms));
      y += 18;
    }
  }
}

static bool is_depth(const std::string& encoding) {
  return encoding == "16UC1" || encoding == "mono16";
}

static cv::Mat to_bgr(const Image& msg) {
  if (is_depth(msg.encoding)) {
    auto cv_img = cv_bridge::toCvShare(
        std::make_shared<Image>(msg), "16UC1");
    cv::Mat scaled, colored;
    cv_img->image.convertTo(scaled, CV_8UC1, 255.0 / 8000.0);
    cv::applyColorMap(scaled, colored, cv::COLORMAP_INFERNO);
    return colored;
  }
  return cv_bridge::toCvCopy(std::make_shared<Image>(msg), "bgr8")->image;
}

// ── Grid builder — layout driven entirely by N from the incoming message ─────
// N≤3 → single row.  N≥4 → 2 rows, ceil(N/2) cols.
// Empty slots (last row when N is odd and ≥5) are filled with black panels.
static cv::Mat make_grid(std::vector<cv::Mat>& panels) {
  if (panels.empty()) return {};
  const int N = static_cast<int>(panels.size());

  for (int i = 1; i < N; ++i)
    if (panels[i].size() != panels[0].size())
      cv::resize(panels[i], panels[i], panels[0].size());

  const int rows = (N <= 3) ? 1 : 2;
  const int cols = (N + rows - 1) / rows;

  // Pad with black to fill the rectangular grid.
  cv::Mat blank = cv::Mat::zeros(panels[0].size(), panels[0].type());
  while (static_cast<int>(panels.size()) < rows * cols)
    panels.push_back(blank);

  std::vector<cv::Mat> row_imgs;
  for (int r = 0; r < rows; ++r) {
    std::vector<cv::Mat> row_panels(
        panels.begin() + r * cols,
        panels.begin() + (r + 1) * cols);
    cv::Mat row;
    cv::hconcat(row_panels, row);
    row_imgs.push_back(row);
  }

  cv::Mat grid;
  cv::vconcat(row_imgs, grid);
  return grid;
}

// ── Node ─────────────────────────────────────────────────────────────────────
class SyncedViewer : public rclcpp::Node {
public:
  SyncedViewer() : Node("synced_viewer") {
    auto qos = rclcpp::SensorDataQoS();
    sub_ = create_subscription<SyncedFrames>(
        "synced_frames", qos,
        std::bind(&SyncedViewer::on_bundle, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Subscribed to /synced_frames — press 'q' to quit.");
  }

  cv::Mat latest_frame() {
    std::lock_guard<std::mutex> lk(mu_);
    return latest_.clone();
  }

private:
  static int64_t stamp_ns(const Image& img) {
    return static_cast<int64_t>(img.header.stamp.sec) * 1'000'000'000LL
           + img.header.stamp.nanosec;
  }

  void on_bundle(const SyncedFrames::ConstSharedPtr& msg) {
    const size_t N = msg->frames.size();
    if (N == 0) return;

    if (trk_.size() != N) trk_.assign(N, StreamTracker{});

    // Build short label per camera for cross-panel references.
    std::vector<std::string> labels(N);
    for (size_t i = 0; i < N; ++i)
      labels[i] = i < msg->labels.size() ? msg->labels[i]
                                          : "cam[" + std::to_string(i) + "]";

    // Decode pairwise_spreads_ms: pairs are ordered (0,1),(0,2),...,(N-2,N-1).
    // For panel i, collect signed delta vs every other camera j.
    // delta(i,j) = +spread means i is later than j.
    const auto& ps = msg->pairwise_spreads_ms;
    auto pair_idx = [&](size_t a, size_t b) -> size_t {
      if (a > b) std::swap(a, b);
      return a * (2 * N - a - 1) / 2 + (b - a - 1);
    };

    std::vector<cv::Mat> panels(N);
    for (size_t i = 0; i < N; ++i) {
      const Image& frame = msg->frames[i];
      trk_[i].tick(frame.data.size());
      panels[i] = to_bgr(frame);

      // For each peer j, recover signed delta from i's perspective.
      // pairwise_spreads_ms stores (t_a - t_b) for pair (a,b) with a<b.
      // So from i's view vs j: sign flips if i > j.
      std::vector<std::pair<std::string, double>> pairs;
      for (size_t j = 0; j < N; ++j) {
        if (j == i || pair_idx(i, j) >= ps.size()) continue;
        const double raw = ps[pair_idx(i, j)];
        const double delta = (i < j) ? raw : -raw;
        pairs.push_back({labels[j], delta});
      }

      draw_overlay(panels[i], labels[i],
                   stamp_ns(frame) / 1.0e9,
                   trk_[i].fps(), trk_[i].mbps(),
                   pairs);
    }

    cv::Mat grid = make_grid(panels);
    std::lock_guard<std::mutex> lk(mu_);
    latest_ = std::move(grid);
  }

  rclcpp::Subscription<SyncedFrames>::SharedPtr sub_;
  std::vector<StreamTracker> trk_;
  std::mutex mu_;
  cv::Mat    latest_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SyncedViewer>();

  std::thread ros_thread([&node]() { rclcpp::spin(node); });

  cv::namedWindow("synced_viewer", cv::WINDOW_AUTOSIZE);

  bool    paused = false;
  cv::Mat frozen;

  while (rclcpp::ok()) {
    const int key = cv::waitKey(30) & 0xFF;
    if (key == 'q') { rclcpp::shutdown(); break; }
    if (key == ' ') {
      paused = !paused;
      if (paused && !frozen.empty()) {
        cv::putText(frozen, "PAUSED",
                    {frozen.cols / 2 - 60, frozen.rows / 2},
                    cv::FONT_HERSHEY_SIMPLEX, 1.4, {0, 0, 0},    5);
        cv::putText(frozen, "PAUSED",
                    {frozen.cols / 2 - 60, frozen.rows / 2},
                    cv::FONT_HERSHEY_SIMPLEX, 1.4, {0, 200, 255}, 2);
      }
    }

    if (!paused) {
      cv::Mat live = node->latest_frame();
      if (!live.empty()) { frozen = live; cv::imshow("synced_viewer", live); }
    } else {
      if (!frozen.empty()) cv::imshow("synced_viewer", frozen);
    }
  }

  ros_thread.join();
  cv::destroyAllWindows();
  return 0;
}
