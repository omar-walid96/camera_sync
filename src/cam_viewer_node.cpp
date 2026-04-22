#include <chrono>
#include <deque>
#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using Image = sensor_msgs::msg::Image;
using ApproxPolicy3 = message_filters::sync_policies::ApproximateTime<Image, Image, Image>;
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
    for (auto& s : samples_) total += s.second;
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
                       double scale = 0.55, int thick = 1)
{
  cv::putText(frame, txt, pos + cv::Point(1, 1),
              cv::FONT_HERSHEY_SIMPLEX, scale, {0, 0, 0}, thick + 1);
  cv::putText(frame, txt, pos,
              cv::FONT_HERSHEY_SIMPLEX, scale, color, thick);
}

static cv::Scalar spread_color(double ms) {
  if (ms < 17.0) return {80, 220, 80};    // green: well within budget
  if (ms < 34.0) return {40, 200, 220};   // yellow: within budget
  return {60, 60, 220};                   // red: over budget
}

static void draw_overlay(cv::Mat& frame, const std::string& label,
                         double stamp_s, double fps, double mbps,
                         double spread_uvc_color_ms,
                         double spread_color_depth_ms,
                         double spread_uvc_depth_ms)
{
  char buf[80];
  int y = 22;

  std::snprintf(buf, sizeof(buf), "%s  %.1f fps  %.1f MB/s", label.c_str(), fps, mbps);
  put_shadow(frame, buf, {8, y}, {255, 255, 255}); y += 22;

  std::snprintf(buf, sizeof(buf), "t = %.3f s", stamp_s);
  put_shadow(frame, buf, {8, y}, {200, 200, 200}); y += 22;

  std::snprintf(buf, sizeof(buf), "UVC-Color  %.1f ms", spread_uvc_color_ms);
  put_shadow(frame, buf, {8, y}, spread_color(spread_uvc_color_ms)); y += 22;

  std::snprintf(buf, sizeof(buf), "Color-Depth %.1f ms", spread_color_depth_ms);
  put_shadow(frame, buf, {8, y}, spread_color(spread_color_depth_ms)); y += 22;

  std::snprintf(buf, sizeof(buf), "UVC-Depth  %.1f ms", spread_uvc_depth_ms);
  put_shadow(frame, buf, {8, y}, spread_color(spread_uvc_depth_ms));
}

// ── Node ─────────────────────────────────────────────────────────────────────
class CamViewer : public rclcpp::Node {
public:
  CamViewer() : Node("cam_viewer") {
    topic_uvc_   = declare_parameter<std::string>("topic_uvc",   "/cam_uvc/image_raw");
    topic_color_ = declare_parameter<std::string>("topic_color", "/camera/camera/color/image_raw");
    topic_depth_ = declare_parameter<std::string>("topic_depth", "/camera/camera/depth/image_rect_raw");
    slop_s_      = declare_parameter<double>("slop_s",    0.034);
    queue_size_  = declare_parameter<int>   ("queue_size", 30);

    auto qos = rclcpp::SensorDataQoS();
    sub_uvc_.subscribe  (this, topic_uvc_,   qos.get_rmw_qos_profile());
    sub_color_.subscribe(this, topic_color_, qos.get_rmw_qos_profile());
    sub_depth_.subscribe(this, topic_depth_, qos.get_rmw_qos_profile());

    sync_ = std::make_shared<message_filters::Synchronizer<ApproxPolicy3>>(
        ApproxPolicy3(queue_size_), sub_uvc_, sub_color_, sub_depth_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(slop_s_));
    sync_->registerCallback(std::bind(&CamViewer::cb, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    cv::namedWindow("cam_viewer", cv::WINDOW_AUTOSIZE);
    RCLCPP_INFO(get_logger(), "Viewer ready — press 'q' to quit.");
  }

private:
  static int64_t to_ns(const std_msgs::msg::Header& h) {
    return (int64_t)h.stamp.sec * 1000000000LL + h.stamp.nanosec;
  }

  static double spread_ms(int64_t a, int64_t b) {
    return std::abs(a - b) / 1.0e6;
  }

  static cv::Mat depth_colormap(const Image::ConstSharedPtr& msg) {
    auto cv_depth = cv_bridge::toCvShare(msg, "16UC1");
    cv::Mat scaled, colored;
    // map 0–8000 mm → 0–255, then apply colormap
    cv_depth->image.convertTo(scaled, CV_8UC1, 255.0 / 8000.0);
    cv::applyColorMap(scaled, colored, cv::COLORMAP_INFERNO);
    return colored;
  }

  void cb(const Image::ConstSharedPtr& uvc,
          const Image::ConstSharedPtr& color,
          const Image::ConstSharedPtr& depth)
  {
    const size_t bytes_uvc   = uvc->data.size();
    const size_t bytes_color = color->data.size();
    const size_t bytes_depth = depth->data.size();

    trk_uvc_.tick(bytes_uvc);
    trk_color_.tick(bytes_color);
    trk_depth_.tick(bytes_depth);

    const int64_t t_uvc   = to_ns(uvc->header);
    const int64_t t_color = to_ns(color->header);
    const int64_t t_depth = to_ns(depth->header);

    const double sp_uvc_color  = spread_ms(t_uvc,   t_color);
    const double sp_color_depth= spread_ms(t_color, t_depth);
    const double sp_uvc_depth  = spread_ms(t_uvc,   t_depth);

    cv::Mat frame_uvc   = cv_bridge::toCvCopy(uvc,   "bgr8")->image;
    cv::Mat frame_color = cv_bridge::toCvCopy(color, "bgr8")->image;
    cv::Mat frame_depth = depth_colormap(depth);

    draw_overlay(frame_uvc,   "UVC",       t_uvc   / 1.0e9,
                 trk_uvc_.fps(),   trk_uvc_.mbps(),
                 sp_uvc_color, sp_color_depth, sp_uvc_depth);

    draw_overlay(frame_color, "RS Color",  t_color / 1.0e9,
                 trk_color_.fps(), trk_color_.mbps(),
                 sp_uvc_color, sp_color_depth, sp_uvc_depth);

    draw_overlay(frame_depth, "RS Depth",  t_depth / 1.0e9,
                 trk_depth_.fps(), trk_depth_.mbps(),
                 sp_uvc_color, sp_color_depth, sp_uvc_depth);

    // Equalise heights, then concat side-by-side
    auto match_height = [](cv::Mat& a, const cv::Mat& ref) {
      if (a.rows == ref.rows) return;
      double s = (double)ref.rows / a.rows;
      cv::resize(a, a, {(int)(a.cols * s), ref.rows});
    };
    match_height(frame_color, frame_uvc);
    match_height(frame_depth, frame_uvc);

    cv::Mat row;
    cv::hconcat(std::vector<cv::Mat>{frame_uvc, frame_color, frame_depth}, row);
    cv::imshow("cam_viewer", row);

    if (cv::waitKey(1) == 'q') rclcpp::shutdown();
  }

  std::string topic_uvc_, topic_color_, topic_depth_;
  double slop_s_;
  int queue_size_;

  message_filters::Subscriber<Image> sub_uvc_, sub_color_, sub_depth_;
  std::shared_ptr<message_filters::Synchronizer<ApproxPolicy3>> sync_;

  StreamTracker trk_uvc_, trk_color_, trk_depth_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CamViewer>());
  cv::destroyAllWindows();
  rclcpp::shutdown();
  return 0;
}
