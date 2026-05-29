#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "ev_ads_runtime_cpp/common.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace ev_ads_runtime_cpp {

class CameraNodeCpp final : public rclcpp::Node {
 public:
  CameraNodeCpp() : Node("camera_node_cpp") {
    name_ = declare_parameter<std::string>("name", "front");
    mode_ = declare_parameter<std::string>("mode", "device");
    device_ = declare_parameter<std::string>("device", "/dev/video0");
    file_ = declare_parameter<std::string>("file", "");
    width_ = declare_parameter<int>("width", 1280);
    height_ = declare_parameter<int>("height", 720);
    fps_ = declare_parameter<double>("fps", 30.0);
    pixel_format_ = declare_parameter<std::string>("pixel_format", "MJPG");
    jpeg_quality_ = declare_parameter<int>("jpeg_quality", 80);
    const std::string topic_ns =
        declare_parameter<std::string>("topic_ns", "").empty()
            ? "/camera/" + name_
            : get_parameter("topic_ns").as_string();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(2)).best_effort();
    pub_image_ = create_publisher<sensor_msgs::msg::CompressedImage>(
        topic_ns + "/image_raw/compressed", qos);
    pub_health_ = create_publisher<std_msgs::msg::UInt8>(topic_ns + "/health", rclcpp::QoS(1));
    health_timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&CameraNodeCpp::health_tick, this));

    worker_ = std::thread([this]() { capture_loop(); });
    RCLCPP_INFO(
        get_logger(),
        "摄像头节点[%s] 模式=%s %dx%d@%.1f -> %s/image_raw/compressed",
        name_.c_str(),
        mode_.c_str(),
        width_,
        height_,
        fps_,
        topic_ns.c_str());
  }

  ~CameraNodeCpp() override {
    stop_.store(true);
    if (worker_.joinable()) {
      worker_.join();
    }
  }

 private:
  void capture_loop() {
    cv::VideoCapture cap;
    int fake_idx = 0;
    auto last_fake = std::chrono::steady_clock::now();
    while (rclcpp::ok() && !stop_.load()) {
      if (mode_ == "fake") {
        const auto target = std::chrono::duration<double>(1.0 / std::max(1.0, fps_));
        const auto now = std::chrono::steady_clock::now();
        const auto elapsed = now - last_fake;
        if (elapsed < target) {
          std::this_thread::sleep_for(target - elapsed);
        }
        last_fake = std::chrono::steady_clock::now();
        cv::Mat frame = make_fake_frame(fake_idx++);
        publish_frame(frame);
        continue;
      }

      if (!cap.isOpened()) {
        const std::string src = mode_ == "file" ? file_ : device_;
        if (src.empty()) {
          set_health(HEALTH_ERROR);
          std::this_thread::sleep_for(std::chrono::seconds(1));
          continue;
        }
        if (mode_ == "device") {
          cap.open(src, cv::CAP_V4L2);
          if (cap.isOpened()) {
            const int fourcc = cv::VideoWriter::fourcc(
                pixel_format_.size() > 0 ? pixel_format_[0] : 'M',
                pixel_format_.size() > 1 ? pixel_format_[1] : 'J',
                pixel_format_.size() > 2 ? pixel_format_[2] : 'P',
                pixel_format_.size() > 3 ? pixel_format_[3] : 'G');
            cap.set(cv::CAP_PROP_FOURCC, fourcc);
            cap.set(cv::CAP_PROP_FRAME_WIDTH, width_);
            cap.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
            cap.set(cv::CAP_PROP_FPS, fps_);
          }
        } else {
          cap.open(src);
        }
        if (!cap.isOpened()) {
          set_health(HEALTH_DISCONNECTED);
          RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 3000, "摄像头[%s] 打开失败: %s", name_.c_str(), src.c_str());
          std::this_thread::sleep_for(std::chrono::seconds(1));
          continue;
        }
        RCLCPP_INFO(get_logger(), "摄像头[%s] 已打开", name_.c_str());
      }

      cv::Mat frame;
      if (!cap.read(frame) || frame.empty()) {
        set_health(HEALTH_STALE);
        if (mode_ == "file") {
          cap.set(cv::CAP_PROP_POS_FRAMES, 0);
        } else {
          cap.release();
          std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        continue;
      }
      publish_frame(frame);
    }
  }

  cv::Mat make_fake_frame(int idx) const {
    cv::Mat frame(height_, width_, CV_8UC3, cv::Scalar(20, 20, 20));
    for (int y = 0; y < frame.rows; ++y) {
      const uint8_t g = static_cast<uint8_t>(20 + (180 * y) / std::max(1, frame.rows - 1));
      frame.row(y).setTo(cv::Scalar(40, g, 80));
    }
    cv::putText(
        frame,
        name_ + " #" + std::to_string(idx),
        cv::Point(20, 45),
        cv::FONT_HERSHEY_SIMPLEX,
        1.0,
        cv::Scalar(255, 255, 255),
        2,
        cv::LINE_AA);
    const int x = 40 + (idx * 7) % std::max(60, width_ - 80);
    cv::circle(frame, cv::Point(x, height_ / 2), 22, cv::Scalar(0, 230, 255), -1, cv::LINE_AA);
    return frame;
  }

  void publish_frame(const cv::Mat& frame) {
    std::vector<uchar> encoded;
    const std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, ev_ads_runtime_cpp::clamp(jpeg_quality_, 1, 100)};
    if (!cv::imencode(".jpg", frame, encoded, params)) {
      set_health(HEALTH_ERROR);
      return;
    }
    sensor_msgs::msg::CompressedImage msg;
    msg.header.stamp = now();
    msg.header.frame_id = "camera_" + name_;
    msg.format = "jpeg";
    msg.data.assign(encoded.begin(), encoded.end());
    pub_image_->publish(msg);

    {
      std::lock_guard<std::mutex> lk(stats_mutex_);
      frame_count_++;
      last_frame_steady_ = std::chrono::steady_clock::now();
      health_ = HEALTH_OK;
    }
  }

  void set_health(uint8_t h) {
    std::lock_guard<std::mutex> lk(stats_mutex_);
    health_ = h;
  }

  void health_tick() {
    std_msgs::msg::UInt8 msg;
    {
      std::lock_guard<std::mutex> lk(stats_mutex_);
      const auto age = std::chrono::steady_clock::now() - last_frame_steady_;
      if (frame_count_ == 0 || age > std::chrono::seconds(3)) {
        msg.data = HEALTH_DISCONNECTED;
      } else if (age > std::chrono::seconds(1)) {
        msg.data = HEALTH_STALE;
      } else {
        msg.data = health_;
      }
    }
    pub_health_->publish(msg);
  }

  std::string name_;
  std::string mode_;
  std::string device_;
  std::string file_;
  std::string pixel_format_;
  int width_{1280};
  int height_{720};
  int jpeg_quality_{80};
  double fps_{30.0};
  std::atomic<bool> stop_{false};
  std::thread worker_;
  std::mutex stats_mutex_;
  uint64_t frame_count_{0};
  uint8_t health_{HEALTH_DISCONNECTED};
  std::chrono::steady_clock::time_point last_frame_steady_{std::chrono::steady_clock::now()};
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_image_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_health_;
  rclcpp::TimerBase::SharedPtr health_timer_;
};

}  // 命名空间 ev_ads_runtime_cpp

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ev_ads_runtime_cpp::CameraNodeCpp>());
  rclcpp::shutdown();
  return 0;
}
