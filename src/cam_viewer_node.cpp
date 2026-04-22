#include <array>
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using Image = sensor_msgs::msg::Image;
// 6 streams: uvc_0 | rs_0_color | rs_0_depth | uvc_1 | rs_1_color | rs_1_depth
using ApproxPolicy6 = message_filters::sync_policies::ApproximateTime<
    Image, Image, Image, Image, Image, Image>;
using Clock = std::chrono::steady_clock;

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

// ── Overlay renderer ─────────────────────────────────────────────────────────
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

static void draw_overlay(cv::Mat& frame, const std::string& label,
                         double stamp_s, double fps, double mbps,
                         double spread_vs_ref_ms, bool is_ref)
{
  char buf[128];
  int y = 20;

  std::snprintf(buf, sizeof(buf), "%s  %.1f fps  %.1f MB/s",
                label.c_str(), fps, mbps);
  put_shadow(frame, buf, {8, y}, {255, 255, 255}); y += 20;

  std::snprintf(buf, sizeof(buf), "t = %.3f s", stamp_s);
  put_shadow(frame, buf, {8, y}, {200, 200, 200}); y += 20;

  if (is_ref) {
    put_shadow(frame, "spread ref", {8, y}, {180, 180, 180});
  } else {
    std::snprintf(buf, sizeof(buf), "vs cam[0]  %.1f ms", spread_vs_ref_ms);
    put_shadow(frame, buf, {8, y}, spread_color(spread_vs_ref_ms));
  }
}

static cv::Mat to_bgr(const Image::ConstSharedPtr& msg, bool is_depth) {
  if (is_depth) {
    auto cv_img = cv_bridge::toCvShare(msg, "16UC1");
    cv::Mat scaled, colored;
    cv_img->image.convertTo(scaled, CV_8UC1, 255.0 / 8000.0);
    cv::applyColorMap(scaled, colored, cv::COLORMAP_INFERNO);
    return colored;
  }
  return cv_bridge::toCvCopy(msg, "bgr8")->image;
}

// ── Node ─────────────────────────────────────────────────────────────────────
class CamViewer : public rclcpp::Node {
public:
  CamViewer() : Node("cam_viewer") {
    // Layout: row0 = [uvc_0, rs_0_color, rs_0_depth]
    //         row1 = [uvc_1, rs_1_color, rs_1_depth]
    const std::array<std::string, 6> default_topics = {
      "/cam_uvc_0/image_raw",
      "/camera_0/camera/color/image_raw",
      "/camera_0/camera/depth/image_rect_raw",
      "/cam_uvc_1/image_raw",
      "/camera_1/camera/color/image_raw",
      "/camera_1/camera/depth/image_rect_raw",
    };
    const std::array<std::string, 6> default_labels = {
      "UVC 0", "RS 0 Color", "RS 0 Depth",
      "UVC 1", "RS 1 Color", "RS 1 Depth",
    };
    // Depth panels: indices 2 and 5 → rendered with INFERNO colormap.
    const std::array<bool, 6> default_is_depth = {
      false, false, true, false, false, true
    };

    for (int i = 0; i < 6; ++i) {
      topics_[i]   = declare_parameter<std::string>(
          "topic_" + std::to_string(i), default_topics[i]);
      labels_[i]   = declare_parameter<std::string>(
          "label_" + std::to_string(i), default_labels[i]);
      is_depth_[i] = declare_parameter<bool>(
          "is_depth_" + std::to_string(i), default_is_depth[i]);
    }
    slop_s_     = declare_parameter<double>("slop_s",     0.034);
    queue_size_ = declare_parameter<int>   ("queue_size", 30);

    auto qos = rclcpp::SensorDataQoS();
    for (int i = 0; i < 6; ++i)
      subs_[i].subscribe(this, topics_[i], qos.get_rmw_qos_profile());

    sync_ = std::make_shared<message_filters::Synchronizer<ApproxPolicy6>>(
        ApproxPolicy6(queue_size_),
        subs_[0], subs_[1], subs_[2], subs_[3], subs_[4], subs_[5]);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(slop_s_));
    sync_->registerCallback(std::bind(&CamViewer::cb, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5, std::placeholders::_6));

    RCLCPP_INFO(get_logger(), "Subscribing to 6 streams:");
    for (int i = 0; i < 6; ++i)
      RCLCPP_INFO(get_logger(), "  [%d] %s%s", i, topics_[i].c_str(),
                  is_depth_[i] ? "  (depth)" : "");
    RCLCPP_INFO(get_logger(), "Viewer ready — press 'q' to quit.");
  }

  cv::Mat latest_frame() {
    std::lock_guard<std::mutex> lk(frame_mu_);
    return latest_frame_.clone();
  }

private:
  static int64_t to_ns(const std_msgs::msg::Header& h) {
    return static_cast<int64_t>(h.stamp.sec) * 1'000'000'000LL + h.stamp.nanosec;
  }
  static double spread_ms(int64_t a, int64_t b) {
    return std::abs(a - b) / 1.0e6;
  }

  void cb(const Image::ConstSharedPtr& m0, const Image::ConstSharedPtr& m1,
          const Image::ConstSharedPtr& m2, const Image::ConstSharedPtr& m3,
          const Image::ConstSharedPtr& m4, const Image::ConstSharedPtr& m5)
  {
    const std::array<const Image::ConstSharedPtr*, 6> msgs = {
      &m0, &m1, &m2, &m3, &m4, &m5
    };
    const int64_t t_ref = to_ns(m0->header);

    std::array<cv::Mat, 6> panels;
    for (int i = 0; i < 6; ++i) {
      const auto& msg = *msgs[i];
      trk_[i].tick(msg->data.size());
      panels[i] = to_bgr(msg, is_depth_[i]);

      const double spread = (i == 0) ? 0.0 : spread_ms(t_ref, to_ns(msg->header));
      draw_overlay(panels[i], labels_[i],
                   to_ns(msg->header) / 1.0e9,
                   trk_[i].fps(), trk_[i].mbps(),
                   spread, i == 0);
    }

    // Normalise all panels to the size of panel[0].
    for (int i = 1; i < 6; ++i) {
      if (panels[i].size() != panels[0].size())
        cv::resize(panels[i], panels[i], panels[0].size());
    }

    // 2×3 grid: top row = [0,1,2], bottom row = [3,4,5]
    cv::Mat top, bot, grid;
    cv::hconcat(std::vector<cv::Mat>{panels[0], panels[1], panels[2]}, top);
    cv::hconcat(std::vector<cv::Mat>{panels[3], panels[4], panels[5]}, bot);
    cv::vconcat(std::vector<cv::Mat>{top, bot}, grid);

    std::lock_guard<std::mutex> lk(frame_mu_);
    latest_frame_ = std::move(grid);
  }

  std::array<std::string, 6> topics_, labels_;
  std::array<bool, 6>        is_depth_;
  double slop_s_;
  int queue_size_;

  std::array<message_filters::Subscriber<Image>, 6> subs_;
  std::shared_ptr<message_filters::Synchronizer<ApproxPolicy6>> sync_;
  std::array<StreamTracker, 6> trk_;

  std::mutex frame_mu_;
  cv::Mat    latest_frame_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CamViewer>();

  // Spin ROS on a background thread — OpenCV GUI must live on the main thread.
  std::thread ros_thread([&node]() { rclcpp::spin(node); });

  cv::namedWindow("cam_viewer", cv::WINDOW_AUTOSIZE);

  bool    paused = false;
  cv::Mat frozen;

  while (rclcpp::ok()) {
    const int key = cv::waitKey(30) & 0xFF;
    if (key == 'q') {
      rclcpp::shutdown();
      break;
    }
    if (key == ' ') {
      paused = !paused;
      if (paused && !frozen.empty()) {
        // Stamp the frozen frame once so the user knows it's paused.
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
      if (!live.empty()) {
        frozen = live;           // keep a copy ready for the moment we pause
        cv::imshow("cam_viewer", live);
      }
    } else {
      if (!frozen.empty())
        cv::imshow("cam_viewer", frozen);
    }
  }

  ros_thread.join();
  cv::destroyAllWindows();
  return 0;
}
