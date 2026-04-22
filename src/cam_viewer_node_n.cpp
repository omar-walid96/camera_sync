// Live side-by-side viewer for N configurable camera topics.
// Each topic gets an independent subscription; a 33 ms display timer
// renders all panels with FPS, bandwidth, timestamp, and pairwise spreads.
// Depth streams (16UC1 / 32FC1) are auto-detected and rendered as INFERNO colormaps.

#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

using Image = sensor_msgs::msg::Image;
using Clock  = std::chrono::steady_clock;

// ── Rolling FPS + bandwidth ──────────────────────────────────────────────────
struct StreamTracker {
  explicit StreamTracker(double window_s = 2.0) : window_s_(window_s) {}

  void tick(size_t bytes) {
    auto t = Clock::now();
    samples_.push_back({t, bytes});
    while (std::chrono::duration<double>(t - samples_.front().first).count() > window_s_)
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
    for (auto& s : samples_) total += s.second;
    double dt = std::chrono::duration<double>(
        samples_.back().first - samples_.front().first).count();
    return dt > 0.0 ? (total / dt) / 1e6 : 0.0;
  }

private:
  double window_s_;
  std::deque<std::pair<Clock::time_point, size_t>> samples_;
};

// ── Overlay helpers ──────────────────────────────────────────────────────────
static void put_shadow(cv::Mat& f, const std::string& txt, cv::Point p,
                       cv::Scalar col, double scale = 0.52, int thick = 1) {
  cv::putText(f, txt, p + cv::Point(1,1), cv::FONT_HERSHEY_SIMPLEX, scale, {0,0,0}, thick+1);
  cv::putText(f, txt, p,                  cv::FONT_HERSHEY_SIMPLEX, scale, col,     thick);
}

static cv::Scalar spread_color(double ms) {
  return ms < 17.0 ? cv::Scalar{80,220,80} :
         ms < 34.0 ? cv::Scalar{40,200,220} :
                     cv::Scalar{60,60,220};
}

// ── Per-stream state (shared between subscription and display timer) ─────────
struct StreamState {
  std::mutex         mtx;
  cv::Mat            frame;       // latest decoded frame (BGR or colormap)
  int64_t            ts_ns = 0;   // latest hardware timestamp
  std::string        label;
  StreamTracker      tracker;

  void update(const Image::ConstSharedPtr& msg, const std::string& lbl) {
    // Auto-detect depth encoding
    cv::Mat decoded;
    if (msg->encoding == "16UC1") {
      auto cv_img = cv_bridge::toCvShare(msg, "16UC1");
      cv::Mat scaled;
      cv_img->image.convertTo(scaled, CV_8UC1, 255.0 / 8000.0);
      cv::applyColorMap(scaled, decoded, cv::COLORMAP_INFERNO);
    } else if (msg->encoding == "32FC1") {
      auto cv_img = cv_bridge::toCvShare(msg, "32FC1");
      cv::Mat scaled;
      cv::convertScaleAbs(cv_img->image, scaled, 255.0 / 8.0);
      cv::applyColorMap(scaled, decoded, cv::COLORMAP_INFERNO);
    } else {
      decoded = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }

    const int64_t ts = static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL
                     + static_cast<int64_t>(msg->header.stamp.nanosec);

    std::lock_guard lock(mtx);
    label = lbl;
    frame = decoded;
    ts_ns = ts;
    tracker.tick(msg->data.size());
  }
};

// ── Node ─────────────────────────────────────────────────────────────────────
class CamViewer : public rclcpp::Node {
public:
  CamViewer() : Node("cam_viewer") {
    topics_ = declare_parameter<std::vector<std::string>>("topics",
        {"/cam_uvc/image_raw", "/camera/camera/color/image_raw",
         "/camera/camera/depth/image_rect_raw"});
    auto lp = declare_parameter<std::vector<std::string>>("labels", std::vector<std::string>{});
    panel_w_ = declare_parameter<int>("panel_width", 640);

    N_ = topics_.size();
    streams_.resize(N_);
    labels_.resize(N_);

    for (size_t i = 0; i < N_; ++i) {
      labels_[i] = (i < lp.size()) ? lp[i] : short_label(topics_[i]);
      streams_[i] = std::make_shared<StreamState>();
      streams_[i]->label = labels_[i];

      auto qos = rclcpp::SensorDataQoS();
      subs_.push_back(create_subscription<Image>(topics_[i], qos,
          [this, i](const Image::ConstSharedPtr& msg) {
            streams_[i]->update(msg, labels_[i]);
          }));
      RCLCPP_INFO(get_logger(), "  [%zu] %s (%s)", i, topics_[i].c_str(), labels_[i].c_str());
    }

    cv::namedWindow("cam_viewer", cv::WINDOW_AUTOSIZE);
    display_timer_ = create_wall_timer(std::chrono::milliseconds(33),
        [this]() { render(); });

    RCLCPP_INFO(get_logger(), "CamViewer ready: %zu topics. Press 'q' to quit.", N_);
  }

private:
  static std::string short_label(const std::string& topic) {
    // Use the second-to-last path segment: /ns/cam/color/image_raw → "color"
    auto parts = std::vector<std::string>{};
    std::stringstream ss(topic);
    std::string seg;
    while (std::getline(ss, seg, '/'))
      if (!seg.empty()) parts.push_back(seg);
    if (parts.size() >= 2) return parts[parts.size() - 2];
    if (!parts.empty())    return parts.back();
    return topic;
  }

  void draw_overlay(cv::Mat& panel, const StreamState& s,
                    const std::vector<int64_t>& all_ts) {
    char buf[96];
    int y = 22;

    std::snprintf(buf, sizeof(buf), "%s  %.1f fps  %.1f MB/s",
                  s.label.c_str(), s.tracker.fps(), s.tracker.mbps());
    put_shadow(panel, buf, {8, y}, {255,255,255}); y += 22;

    std::snprintf(buf, sizeof(buf), "t = %.3f s", s.ts_ns / 1.0e9);
    put_shadow(panel, buf, {8, y}, {200,200,200}); y += 22;

    // Spread vs every other stream
    for (size_t j = 0; j < N_; ++j) {
      if (all_ts[j] == 0 || &s == streams_[j].get()) continue;
      double ms = std::abs(s.ts_ns - all_ts[j]) / 1.0e6;
      std::snprintf(buf, sizeof(buf), "↔ %s  %.1f ms",
                    labels_[j].c_str(), ms);
      put_shadow(panel, buf, {8, y}, spread_color(ms)); y += 22;
    }
  }

  void render() {
    // Snapshot all stream states under their individual locks
    std::vector<cv::Mat> panels;
    std::vector<int64_t> ts(N_, 0);

    for (size_t i = 0; i < N_; ++i) {
      std::lock_guard lock(streams_[i]->mtx);
      ts[i] = streams_[i]->ts_ns;
    }

    for (size_t i = 0; i < N_; ++i) {
      cv::Mat panel;
      {
        std::lock_guard lock(streams_[i]->mtx);
        if (streams_[i]->frame.empty()) {
          // Placeholder until first frame arrives
          panel = cv::Mat(480, panel_w_, CV_8UC3, cv::Scalar(30,30,30));
          put_shadow(panel, "waiting for " + labels_[i], {8, 240}, {180,180,180});
          panels.push_back(panel);
          continue;
        }
        panel = streams_[i]->frame.clone();
      }

      // Scale to target panel width
      if (panel.cols != panel_w_) {
        double s = static_cast<double>(panel_w_) / panel.cols;
        cv::resize(panel, panel, {panel_w_, static_cast<int>(panel.rows * s)});
      }

      draw_overlay(panel, *streams_[i], ts);
      panels.push_back(panel);
    }

    // Equalise heights and hconcat
    int max_h = 0;
    for (auto& p : panels) max_h = std::max(max_h, p.rows);
    for (auto& p : panels)
      if (p.rows != max_h)
        cv::resize(p, p, {p.cols, max_h});

    cv::Mat combined;
    cv::hconcat(panels, combined);
    cv::imshow("cam_viewer", combined);

    if (cv::waitKey(1) == 'q') rclcpp::shutdown();
  }

  size_t N_;
  int panel_w_;
  std::vector<std::string> topics_, labels_;
  std::vector<std::shared_ptr<StreamState>> streams_;
  std::vector<rclcpp::Subscription<Image>::SharedPtr> subs_;
  rclcpp::TimerBase::SharedPtr display_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CamViewer>());
  cv::destroyAllWindows();
  rclcpp::shutdown();
  return 0;
}
