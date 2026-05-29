#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "ev_ads_interfaces/msg/blind_spot_state.hpp"
#include "ev_ads_runtime_cpp/common.hpp"
#include "ev_ads_runtime_cpp/runtime_config.hpp"
#include "ev_ads_runtime_cpp/yolo_onnx.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace ev_ads_runtime_cpp {
namespace {

std::vector<int> to_int_vector(const std::vector<int64_t>& values) {
  std::vector<int> out;
  out.reserve(values.size());
  for (const auto value : values) {
    out.push_back(static_cast<int>(value));
  }
  return out;
}

std::vector<int64_t> to_int64_vector(const std::vector<int>& values) {
  std::vector<int64_t> out;
  out.reserve(values.size());
  for (const auto value : values) {
    out.push_back(value);
  }
  return out;
}

RearPerceptionConfig read_rear_config(rclcpp::Node* node) {
  RearPerceptionConfig cfg;
  cfg.model.class_ids = {0, 1, 2, 3, 5, 7};
  cfg.publish_rate_hz = node->declare_parameter<double>("publish_rate_hz", cfg.publish_rate_hz);
  cfg.mode = node->declare_parameter<std::string>("fake_mode", cfg.mode);
  cfg.health.require_camera_health =
      node->declare_parameter<bool>("require_camera_health", cfg.health.require_camera_health);
  cfg.health.camera_timeout_ms =
      node->declare_parameter<int>("camera_timeout_ms", cfg.health.camera_timeout_ms);
  cfg.health.model_timeout_ms =
      node->declare_parameter<int>("model_timeout_ms", cfg.health.model_timeout_ms);
  cfg.model.path = node->declare_parameter<std::string>("model_path", cfg.model.path);
  cfg.model.input_width = node->declare_parameter<int>("model_input_width", cfg.model.input_width);
  cfg.model.input_height = node->declare_parameter<int>("model_input_height", cfg.model.input_height);
  cfg.model.confidence_threshold =
      node->declare_parameter<double>("model_confidence_threshold", cfg.model.confidence_threshold);
  cfg.model.nms_threshold =
      node->declare_parameter<double>("model_nms_threshold", cfg.model.nms_threshold);
  cfg.model.has_objectness =
      node->declare_parameter<bool>("model_has_objectness", cfg.model.has_objectness);
  cfg.model.class_ids = to_int_vector(node->declare_parameter<std::vector<int64_t>>(
      "model_class_ids", to_int64_vector(cfg.model.class_ids)));
  cfg.distance_focal_px =
      node->declare_parameter<double>("distance_focal_px", cfg.distance_focal_px);
  cfg.present_max_m = node->declare_parameter<double>("present_max_m", cfg.present_max_m);
  cfg.approaching_speed_mps =
      node->declare_parameter<double>("approaching_speed_mps", cfg.approaching_speed_mps);
  cfg.fisheye_undistort =
      node->declare_parameter<bool>("fisheye_undistort", cfg.fisheye_undistort);
  cfg.fisheye_k = node->declare_parameter<std::vector<double>>("fisheye_k", cfg.fisheye_k);
  cfg.fisheye_d = node->declare_parameter<std::vector<double>>("fisheye_d", cfg.fisheye_d);
  cfg.fisheye_balance = node->declare_parameter<double>("fisheye_balance", cfg.fisheye_balance);
  cfg.fisheye_fov_scale =
      node->declare_parameter<double>("fisheye_fov_scale", cfg.fisheye_fov_scale);
  return cfg;
}

}  // 匿名命名空间

struct RearObservation {
  double ld = -1.0;
  double cd = -1.0;
  double rd = -1.0;
  double lv = 0.0;
  double cv = 0.0;
  double rv = 0.0;
};

class RearNodeCpp final : public rclcpp::Node {
 public:
  RearNodeCpp() : Node("rear_perception_node_cpp") {
    config_ = read_rear_config(this);

    pub_ = create_publisher<ev_ads_interfaces::msg::BlindSpotState>(
        topics_.blind_spot, rclcpp::QoS(10));
    sim_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        topics_.sim_rear_zones,
        rclcpp::QoS(10),
        [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
          if (msg->data.size() < 6) {
            return;
          }
          injected_ = {
              msg->data[0], msg->data[1], msg->data[2],
              msg->data[3], msg->data[4], msg->data[5]};
          injected_stamp_ = now();
          has_injected_ = true;
        });
    camera_health_sub_ = create_subscription<std_msgs::msg::UInt8>(
        RuntimeTopics::health_topic(topics_.camera_rear_ns),
        rclcpp::QoS(5),
        [this](std_msgs::msg::UInt8::SharedPtr msg) {
          camera_health_ = msg->data;
          camera_health_stamp_ = now();
          has_camera_health_ = true;
        });

    if (config_.mode == "model") {
      load_model();
      image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
          RuntimeTopics::image_topic(topics_.camera_rear_ns),
          rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
          std::bind(&RearNodeCpp::image_callback, this, std::placeholders::_1));
    }

    t0_ = now();
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(1.0, config_.publish_rate_hz)),
        std::bind(&RearNodeCpp::tick, this));
    RCLCPP_INFO(
        get_logger(),
        "后置感知节点启动，模式=%s 模型=%s",
        config_.mode.c_str(),
        config_.model.path.empty() ? "<empty>" : config_.model.path.c_str());
  }

 private:
  void load_model() {
    if (config_.model.path.empty()) {
      RCLCPP_WARN(get_logger(), "后置模型模式已启用，但 model_path 为空");
      return;
    }
    YoloOnnxDetector::Config config;
    config.model_path = config_.model.path;
    config.input_size = cv::Size(config_.model.input_width, config_.model.input_height);
    config.confidence_threshold = static_cast<float>(config_.model.confidence_threshold);
    config.nms_threshold = static_cast<float>(config_.model.nms_threshold);
    config.has_objectness = config_.model.has_objectness;
    config.class_allowlist = config_.model.class_ids;

    std::string error;
    model_ready_ = detector_.load(config, &error);
    if (!model_ready_) {
      RCLCPP_ERROR(get_logger(), "后置 YOLO ONNX 加载失败: %s", error.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "后置 YOLO ONNX 已加载: %s", config_.model.path.c_str());
    }
  }

  void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    if (!model_ready_) {
      return;
    }
    cv::Mat encoded(1, static_cast<int>(msg->data.size()), CV_8UC1, const_cast<uint8_t*>(msg->data.data()));
    cv::Mat frame = cv::imdecode(encoded, cv::IMREAD_COLOR);
    if (frame.empty()) {
      return;
    }
    cv::Mat model_frame = undistort_if_needed(frame);
    model_obs_ = observation_from_detections(
        detector_.infer(model_frame), model_frame.size(), now().seconds());
    model_stamp_ = now();
    has_model_obs_ = true;
  }

  cv::Mat undistort_if_needed(const cv::Mat& frame) {
    if (!config_.fisheye_undistort) {
      return frame;
    }
    if (config_.fisheye_k.size() != 4 || config_.fisheye_d.size() != 4) {
      if (!fisheye_config_warned_) {
        RCLCPP_WARN(
            get_logger(),
            "已开启后置鱼眼去畸变，但 fisheye_k/fisheye_d 参数不完整，暂时使用原图");
        fisheye_config_warned_ = true;
      }
      return frame;
    }
    if (!fisheye_maps_ready_ || fisheye_map_size_ != frame.size()) {
      rebuild_fisheye_maps(frame.size());
    }
    if (!fisheye_maps_ready_) {
      return frame;
    }
    cv::Mat corrected;
    cv::remap(
        frame,
        corrected,
        fisheye_map1_,
        fisheye_map2_,
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT);
    return corrected;
  }

  void rebuild_fisheye_maps(const cv::Size& image_size) {
    try {
      const cv::Mat k = (cv::Mat_<double>(3, 3) <<
          config_.fisheye_k[0], 0.0, config_.fisheye_k[2],
          0.0, config_.fisheye_k[1], config_.fisheye_k[3],
          0.0, 0.0, 1.0);
      const cv::Mat d = (cv::Mat_<double>(4, 1) <<
          config_.fisheye_d[0], config_.fisheye_d[1], config_.fisheye_d[2], config_.fisheye_d[3]);
      cv::Mat new_k;
      cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
          k,
          d,
          image_size,
          cv::Matx33d::eye(),
          new_k,
          clamp(config_.fisheye_balance, 0.0, 1.0),
          image_size,
          std::max(0.1, config_.fisheye_fov_scale));
      cv::fisheye::initUndistortRectifyMap(
          k,
          d,
          cv::Matx33d::eye(),
          new_k,
          image_size,
          CV_16SC2,
          fisheye_map1_,
          fisheye_map2_);
      fisheye_map_size_ = image_size;
      fisheye_maps_ready_ = true;
      RCLCPP_INFO(
          get_logger(),
          "后置鱼眼去畸变映射已建立，尺寸=%dx%d balance=%.2f fov_scale=%.2f",
          image_size.width,
          image_size.height,
          config_.fisheye_balance,
          config_.fisheye_fov_scale);
    } catch (const cv::Exception& e) {
      fisheye_maps_ready_ = false;
      RCLCPP_ERROR(get_logger(), "后置鱼眼去畸变映射建立失败: %s", e.what());
    }
  }

  RearObservation observation_from_detections(
      const std::vector<YoloDetection>& detections,
      const cv::Size& image_size,
      double stamp_s) {
    RearObservation obs;
    std::array<double, 3> best_distance{
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity()};

    for (const auto& detection : detections) {
      if (detection.box.height < 2.0f || detection.box.width < 2.0f) {
        continue;
      }
      const float cx = detection.box.x + detection.box.width * 0.5f;
      const int zone = cx < image_size.width / 3.0f ? 0 : (cx > image_size.width * 2.0f / 3.0f ? 2 : 1);
      const double distance = estimate_distance_m(detection);
      if (distance < best_distance[zone]) {
        best_distance[zone] = distance;
      }
    }

    assign_zone(0, best_distance[0], stamp_s, obs.ld, obs.lv);
    assign_zone(1, best_distance[1], stamp_s, obs.cd, obs.cv);
    assign_zone(2, best_distance[2], stamp_s, obs.rd, obs.rv);
    return obs;
  }

  void assign_zone(
      int zone,
      double distance,
      double stamp_s,
      double& out_distance,
      double& out_closing) {
    if (!std::isfinite(distance)) {
      out_distance = -1.0;
      out_closing = 0.0;
      previous_distance_[zone] = -1.0;
      previous_stamp_s_[zone] = stamp_s;
      previous_closing_[zone] = 0.0;
      return;
    }

    double closing = 0.0;
    const double prev_distance = previous_distance_[zone];
    const double dt = stamp_s - previous_stamp_s_[zone];
    if (prev_distance > 0.0 && dt > 0.03 && dt < 2.0) {
      closing = std::max(0.0, (prev_distance - distance) / dt);
      closing = 0.6 * closing + 0.4 * previous_closing_[zone];
    }

    previous_distance_[zone] = distance;
    previous_stamp_s_[zone] = stamp_s;
    previous_closing_[zone] = closing;
    out_distance = distance;
    out_closing = closing;
  }

  double estimate_distance_m(const YoloDetection& detection) const {
    const double height = std::max(2.0f, detection.box.height);
    const double nominal_height = nominal_target_height_m(detection.class_id);
    return clamp(config_.distance_focal_px * nominal_height / height, 0.3, config_.present_max_m * 1.5);
  }

  double nominal_target_height_m(int class_id) const {
    switch (class_id) {
      case 0:
        return 1.70;  // 行人
      case 1:
      case 3:
        return 1.20;  // 自行车或摩托车
      case 5:
      case 7:
        return 2.60;  // 公交车或卡车
      default:
        return 1.50;  // 小汽车或未知车辆类
    }
  }

  RearObservation observe() {
    const auto n = now();
    if (has_injected_ && (n - injected_stamp_).seconds() < 1.0) {
      return injected_;
    }
    if (config_.mode == "model") {
      if (has_model_obs_ && (n - model_stamp_).seconds() * 1000.0 <= config_.health.model_timeout_ms) {
        return model_obs_;
      }
      return {};
    }
    if (config_.mode == "idle") {
      return {};
    }
    const double t = std::fmod((n - t0_).seconds(), 30.0);
    RearObservation obs;
    obs.rd = std::max(1.5, 8.0 - t * 0.25);
    obs.rv = 0.25;
    return obs;
  }

  uint8_t camera_health() const {
    if (!config_.health.require_camera_health) {
      return HEALTH_OK;
    }
    if (!has_camera_health_) {
      return HEALTH_DISCONNECTED;
    }
    const auto age_ms = (now() - camera_health_stamp_).seconds() * 1000.0;
    if (age_ms > config_.health.camera_timeout_ms) {
      return HEALTH_DISCONNECTED;
    }
    return camera_health_;
  }

  uint8_t perception_health(uint8_t camera_health) const {
    if (camera_health != HEALTH_OK) {
      return camera_health;
    }
    if (config_.mode != "model") {
      return camera_health;
    }
    if (!model_ready_) {
      return HEALTH_ERROR;
    }
    if (!has_model_obs_) {
      return HEALTH_STALE;
    }
    const auto age_ms = (now() - model_stamp_).seconds() * 1000.0;
    if (age_ms > config_.health.model_timeout_ms) {
      return HEALTH_STALE;
    }
    return HEALTH_OK;
  }

  void tick() {
    const auto obs = observe();
    const uint8_t health = perception_health(camera_health());
    const uint8_t ls = bad_health(health) ? ZONE_CLEAR :
        zone_state(obs.ld, obs.lv, config_.approaching_speed_mps, config_.present_max_m);
    const uint8_t cs = bad_health(health) ? ZONE_CLEAR :
        zone_state(obs.cd, obs.cv, config_.approaching_speed_mps, config_.present_max_m);
    const uint8_t rs = bad_health(health) ? ZONE_CLEAR :
        zone_state(obs.rd, obs.rv, config_.approaching_speed_mps, config_.present_max_m);
    const double score = bad_health(health) ? 0.0 : aggregate_rear_risk(ls, cs, rs);

    ev_ads_interfaces::msg::BlindSpotState msg;
    msg.header.stamp = now();
    msg.header.frame_id = "camera_rear";
    msg.zone_left = ls;
    msg.zone_center = cs;
    msg.zone_right = rs;
    msg.left_distance = static_cast<float>(obs.ld);
    msg.center_distance = static_cast<float>(obs.cd);
    msg.right_distance = static_cast<float>(obs.rd);
    msg.left_closing = static_cast<float>(obs.lv);
    msg.center_closing = static_cast<float>(obs.cv);
    msg.right_closing = static_cast<float>(obs.rv);
    msg.risk_score = static_cast<float>(score);
    msg.health = health;
    pub_->publish(msg);
  }

  RuntimeTopics topics_;
  RearPerceptionConfig config_;
  bool model_ready_{false};
  bool has_model_obs_{false};
  bool has_injected_{false};
  bool has_camera_health_{false};
  bool fisheye_maps_ready_{false};
  bool fisheye_config_warned_{false};
  uint8_t camera_health_{HEALTH_DISCONNECTED};
  RearObservation injected_;
  RearObservation model_obs_;
  cv::Size fisheye_map_size_;
  cv::Mat fisheye_map1_;
  cv::Mat fisheye_map2_;
  std::array<double, 3> previous_distance_{{-1.0, -1.0, -1.0}};
  std::array<double, 3> previous_stamp_s_{{0.0, 0.0, 0.0}};
  std::array<double, 3> previous_closing_{{0.0, 0.0, 0.0}};
  rclcpp::Time injected_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time camera_health_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time model_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time t0_{0, 0, RCL_ROS_TIME};
  YoloOnnxDetector detector_;
  rclcpp::Publisher<ev_ads_interfaces::msg::BlindSpotState>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sim_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr camera_health_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // 命名空间 ev_ads_runtime_cpp

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ev_ads_runtime_cpp::RearNodeCpp>());
  rclcpp::shutdown();
  return 0;
}
