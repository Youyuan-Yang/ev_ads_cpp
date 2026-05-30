// 驾驶员注意力节点：YuNet 人脸检测 + DMS YOLO，输出疲劳与分心状态。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>

#include "ev_ads_runtime_cpp/msg/driver_state.hpp"
#include "ev_ads_runtime_cpp/risk_math.hpp"
#include "ev_ads_runtime_cpp/runtime_config.hpp"
#include "ev_ads_runtime_cpp/onnx_yolo_detector.hpp"
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

DriverMonitorConfig read_driver_config(rclcpp::Node* node) {
  DriverMonitorConfig cfg;
  cfg.publish_rate_hz = node->declare_parameter<double>("publish_rate_hz", cfg.publish_rate_hz);
  cfg.mode = node->declare_parameter<std::string>("fake_mode", cfg.mode);
  cfg.health.require_camera_health =
      node->declare_parameter<bool>("require_camera_health", cfg.health.require_camera_health);
  cfg.health.camera_timeout_ms =
      node->declare_parameter<int>("camera_timeout_ms", cfg.health.camera_timeout_ms);
  cfg.health.model_timeout_ms =
      node->declare_parameter<int>("model_timeout_ms", cfg.health.model_timeout_ms);
  cfg.dms_model.path = node->declare_parameter<std::string>("model_path", cfg.dms_model.path);
  cfg.face_model_path = node->declare_parameter<std::string>("face_model_path", cfg.face_model_path);
  cfg.dms_model.input_width =
      node->declare_parameter<int>("model_input_width", cfg.dms_model.input_width);
  cfg.dms_model.input_height =
      node->declare_parameter<int>("model_input_height", cfg.dms_model.input_height);
  cfg.dms_model.confidence_threshold =
      node->declare_parameter<double>("model_confidence_threshold", cfg.dms_model.confidence_threshold);
  cfg.dms_model.nms_threshold =
      node->declare_parameter<double>("model_nms_threshold", cfg.dms_model.nms_threshold);
  cfg.dms_model.has_objectness =
      node->declare_parameter<bool>("model_has_objectness", cfg.dms_model.has_objectness);
  cfg.face_input_width = node->declare_parameter<int>("face_input_width", cfg.face_input_width);
  cfg.face_input_height = node->declare_parameter<int>("face_input_height", cfg.face_input_height);
  cfg.face_score_threshold =
      node->declare_parameter<double>("face_score_threshold", cfg.face_score_threshold);
  cfg.face_nms_threshold =
      node->declare_parameter<double>("face_nms_threshold", cfg.face_nms_threshold);
  cfg.face_top_k = node->declare_parameter<int>("face_top_k", cfg.face_top_k);
  cfg.face_absence_warning_ms =
      node->declare_parameter<int>("face_absence_warning_ms", cfg.face_absence_warning_ms);
  cfg.face_absence_score =
      node->declare_parameter<double>("face_absence_score", cfg.face_absence_score);
  cfg.dms_model.class_ids = to_int_vector(node->declare_parameter<std::vector<int64_t>>(
      "model_class_ids", to_int64_vector(cfg.dms_model.class_ids)));
  cfg.face_class_ids = to_int_vector(node->declare_parameter<std::vector<int64_t>>(
      "face_class_ids", to_int64_vector(cfg.face_class_ids)));
  cfg.open_eye_class_ids = to_int_vector(node->declare_parameter<std::vector<int64_t>>(
      "open_eye_class_ids", to_int64_vector(cfg.open_eye_class_ids)));
  cfg.half_eye_class_ids = to_int_vector(node->declare_parameter<std::vector<int64_t>>(
      "half_eye_class_ids", to_int64_vector(cfg.half_eye_class_ids)));
  cfg.closed_eye_class_ids = to_int_vector(node->declare_parameter<std::vector<int64_t>>(
      "closed_eye_class_ids", to_int64_vector(cfg.closed_eye_class_ids)));
  cfg.yawn_class_ids = to_int_vector(node->declare_parameter<std::vector<int64_t>>(
      "yawn_class_ids", to_int64_vector(cfg.yawn_class_ids)));
  cfg.phone_class_ids = to_int_vector(node->declare_parameter<std::vector<int64_t>>(
      "phone_class_ids", to_int64_vector(cfg.phone_class_ids)));
  cfg.distracted_class_ids = to_int_vector(node->declare_parameter<std::vector<int64_t>>(
      "distracted_class_ids", to_int64_vector(cfg.distracted_class_ids)));
  cfg.fatigue_class_ids = to_int_vector(node->declare_parameter<std::vector<int64_t>>(
      "fatigue_class_ids", to_int64_vector(cfg.fatigue_class_ids)));
  return cfg;
}

bool contains_id(const std::vector<int>& ids, int class_id) {
  return std::find(ids.begin(), ids.end(), class_id) != ids.end();
}

double max_confidence_for(const std::vector<YoloDetection>& detections, const std::vector<int>& ids) {
  double score = 0.0;
  for (const auto& detection : detections) {
    if (contains_id(ids, detection.class_id)) {
      score = std::max(score, static_cast<double>(detection.confidence));
    }
  }
  return score;
}

const YoloDetection* best_detection_for(
    const std::vector<YoloDetection>& detections,
    const std::vector<int>& ids) {
  const YoloDetection* best = nullptr;
  for (const auto& detection : detections) {
    if (!contains_id(ids, detection.class_id)) {
      continue;
    }
    if (best == nullptr || detection.confidence > best->confidence) {
      best = &detection;
    }
  }
  return best;
}

}  // 匿名命名空间

struct FaceObservation {
  bool evaluated = false;
  bool found = false;
  double confidence = 0.0;
  cv::Rect2f box;
  cv::Point2f nose;
  bool has_nose = false;
};

struct DriverObservation {
  uint8_t face_visible = FACE_UNKNOWN;
  double eye = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  double distraction = 0.0;
  double confidence = 0.0;
};

class DriverAttentionNode final : public rclcpp::Node {
 public:
  DriverAttentionNode() : Node("driver_attention_node") {
    config_ = read_driver_config(this);

    pub_ = create_publisher<ev_ads_runtime_cpp::msg::DriverState>(
        topics_.driver_state, rclcpp::QoS(10));
    sim_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        topics_.sim_driver_observation,
        rclcpp::QoS(10),
        [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
          if (msg->data.size() < 5) {
            return;
          }
          injected_.face_visible = static_cast<uint8_t>(msg->data[0]);
          injected_.eye = msg->data[1];
          injected_.pitch = msg->data[2];
          injected_.yaw = msg->data[3];
          injected_.distraction = msg->data[4];
          injected_.confidence = 1.0;
          injected_stamp_ = now();
          has_injected_ = true;
        });
    camera_health_sub_ = create_subscription<std_msgs::msg::UInt8>(
        RuntimeTopics::health_topic(topics_.camera_driver_ns),
        rclcpp::QoS(5),
        [this](std_msgs::msg::UInt8::SharedPtr msg) {
          camera_health_ = msg->data;
          camera_health_stamp_ = now();
          has_camera_health_ = true;
        });

    if (config_.mode == "model") {
      load_models();
      image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
          RuntimeTopics::image_topic(topics_.camera_driver_ns),
          rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
          std::bind(&DriverAttentionNode::image_callback, this, std::placeholders::_1));
    }

    t0_ = now();
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(1.0, config_.publish_rate_hz)),
        std::bind(&DriverAttentionNode::tick, this));
    RCLCPP_INFO(
        get_logger(),
        "驾驶员监测节点启动，模式=%s DMS模型=%s 人脸模型=%s",
        config_.mode.c_str(),
        config_.dms_model.path.empty() ? "<empty>" : config_.dms_model.path.c_str(),
        config_.face_model_path.empty() ? "<empty>" : config_.face_model_path.c_str());
  }

 private:
  void load_models() {
    load_face_model();
    load_dms_model();
    if (!face_ready_ && !dms_ready_) {
      RCLCPP_WARN(get_logger(), "驾驶员模型模式已启用，但人脸模型和 DMS 模型都未就绪");
    }
  }

  void load_face_model() {
    if (config_.face_model_path.empty()) {
      RCLCPP_WARN(get_logger(), "YuNet 人脸模型路径为空，将只使用 DMS YOLO 线索");
      return;
    }
    try {
      face_detector_ = cv::FaceDetectorYN::create(
          config_.face_model_path,
          "",
          cv::Size(config_.face_input_width, config_.face_input_height),
          static_cast<float>(config_.face_score_threshold),
          static_cast<float>(config_.face_nms_threshold),
          config_.face_top_k);
      face_ready_ = !face_detector_.empty();
    } catch (const cv::Exception& e) {
      RCLCPP_ERROR(get_logger(), "YuNet 人脸模型加载失败: %s", e.what());
      face_ready_ = false;
    }
    if (face_ready_) {
      RCLCPP_INFO(get_logger(), "YuNet 人脸模型已加载: %s", config_.face_model_path.c_str());
    }
  }

  void load_dms_model() {
    if (config_.dms_model.path.empty()) {
      RCLCPP_WARN(get_logger(), "DMS YOLO 模型路径为空，将只使用 YuNet 人脸线索");
      return;
    }
    YoloOnnxDetector::Config config;
    config.model_path = config_.dms_model.path;
    config.input_size = cv::Size(config_.dms_model.input_width, config_.dms_model.input_height);
    config.confidence_threshold = static_cast<float>(config_.dms_model.confidence_threshold);
    config.nms_threshold = static_cast<float>(config_.dms_model.nms_threshold);
    config.has_objectness = config_.dms_model.has_objectness;
    config.class_allowlist = config_.dms_model.class_ids;

    std::string error;
    dms_ready_ = dms_detector_.load(config, &error);
    if (!dms_ready_) {
      RCLCPP_ERROR(get_logger(), "驾驶员 YOLO ONNX 加载失败: %s", error.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "驾驶员 YOLO ONNX 已加载: %s", config_.dms_model.path.c_str());
    }
  }

  void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    if (!pipeline_ready()) {
      return;
    }
    cv::Mat encoded(1, static_cast<int>(msg->data.size()), CV_8UC1, const_cast<uint8_t*>(msg->data.data()));
    cv::Mat frame = cv::imdecode(encoded, cv::IMREAD_COLOR);
    if (frame.empty()) {
      return;
    }
    model_obs_ = observation_from_frame(frame);
    apply_face_absence_gate(&model_obs_);
    model_stamp_ = now();
    has_model_obs_ = true;
  }

  bool pipeline_ready() const {
    return face_ready_ || dms_ready_;
  }

  FaceObservation detect_face(const cv::Mat& frame) {
    FaceObservation result;
    if (!face_ready_ || face_detector_.empty() || frame.empty()) {
      return result;
    }

    try {
      cv::Mat face_input;
      const cv::Size input_size(
          std::max(32, config_.face_input_width),
          std::max(32, config_.face_input_height));
      const double scale_x = static_cast<double>(frame.cols) / input_size.width;
      const double scale_y = static_cast<double>(frame.rows) / input_size.height;
      if (frame.size() == input_size) {
        face_input = frame;
      } else {
        cv::resize(frame, face_input, input_size, 0.0, 0.0, cv::INTER_LINEAR);
      }

      face_detector_->setInputSize(input_size);
      cv::Mat faces;
      face_detector_->detect(face_input, faces);
      result.evaluated = true;

      int best_index = -1;
      double best_score = -1.0;
      for (int i = 0; i < faces.rows; ++i) {
        const float x = faces.at<float>(i, 0);
        const float y = faces.at<float>(i, 1);
        const float w = faces.at<float>(i, 2);
        const float h = faces.at<float>(i, 3);
        const float score = faces.cols > 14 ? faces.at<float>(i, 14) : 0.0f;
        const double area_weight = std::max(0.0f, w * h) /
            std::max(1.0, static_cast<double>(frame.cols * frame.rows));
        const double rank_score = static_cast<double>(score) + 0.05 * area_weight;
        if (rank_score > best_score) {
          best_score = rank_score;
          best_index = i;
        }
      }

      if (best_index >= 0) {
        result.found = true;
        result.box = cv::Rect2f(
            static_cast<float>(faces.at<float>(best_index, 0) * scale_x),
            static_cast<float>(faces.at<float>(best_index, 1) * scale_y),
            static_cast<float>(faces.at<float>(best_index, 2) * scale_x),
            static_cast<float>(faces.at<float>(best_index, 3) * scale_y));
        result.confidence = faces.cols > 14 ? faces.at<float>(best_index, 14) : 0.0f;
        if (faces.cols > 9) {
          result.nose = cv::Point2f(
              static_cast<float>(faces.at<float>(best_index, 8) * scale_x),
              static_cast<float>(faces.at<float>(best_index, 9) * scale_y));
          result.has_nose = true;
        }
      }
    } catch (const cv::Exception& e) {
      RCLCPP_WARN(get_logger(), "YuNet 推理失败: %s", e.what());
    }
    return result;
  }

  DriverObservation observation_from_frame(const cv::Mat& frame) {
    const FaceObservation face = detect_face(frame);
    const std::vector<YoloDetection> detections =
        dms_ready_ ? dms_detector_.infer(frame) : std::vector<YoloDetection>{};
    return observation_from_detections(detections, frame.size(), face);
  }

  DriverObservation observation_from_detections(
      const std::vector<YoloDetection>& detections,
      const cv::Size& image_size,
      const FaceObservation& face_obs) const {
    DriverObservation obs;
    const YoloDetection* yolo_face = best_detection_for(detections, config_.face_class_ids);
    const double closed_eye = max_confidence_for(detections, config_.closed_eye_class_ids);
    const double half_eye = max_confidence_for(detections, config_.half_eye_class_ids);
    const double open_eye = max_confidence_for(detections, config_.open_eye_class_ids);
    const double yawn = max_confidence_for(detections, config_.yawn_class_ids);
    const double phone = max_confidence_for(detections, config_.phone_class_ids);
    const double distracted = max_confidence_for(detections, config_.distracted_class_ids);
    const double fatigue = max_confidence_for(detections, config_.fatigue_class_ids);

    const bool yolo_facial_evidence =
        yolo_face != nullptr || closed_eye > 0.0 || half_eye > 0.0 || open_eye > 0.0 || yawn > 0.0;
    if (face_obs.evaluated) {
      obs.face_visible = (face_obs.found || yolo_facial_evidence) ? FACE_YES : FACE_NO;
    } else {
      obs.face_visible = yolo_facial_evidence ? FACE_YES : FACE_UNKNOWN;
    }
    obs.confidence = std::max({face_obs.found ? face_obs.confidence : 0.0,
                               yolo_face == nullptr ? 0.0 : static_cast<double>(yolo_face->confidence),
                               closed_eye,
                               half_eye,
                               open_eye,
                               yawn,
                               phone,
                               distracted,
                               fatigue});

    if (closed_eye > 0.0 || half_eye > 0.0 || open_eye > 0.0) {
      obs.eye = (closed_eye + 0.5 * half_eye) /
          std::max(1e-6, closed_eye + half_eye + open_eye);
    }
    obs.eye = clamp(std::max({obs.eye, closed_eye, 0.55 * half_eye, fatigue, 0.55 * yawn}), 0.0, 1.0);
    obs.distraction = clamp(std::max({phone, distracted, 0.45 * yawn}), 0.0, 1.0);

    cv::Rect2f face_box;
    bool has_face_box = false;
    if (face_obs.found) {
      face_box = face_obs.box;
      has_face_box = true;
    } else if (yolo_face != nullptr) {
      face_box = yolo_face->box;
      has_face_box = true;
    }

    if (has_face_box && image_size.width > 0 && image_size.height > 0) {
      const double cx = face_box.x + face_box.width * 0.5;
      const double cy = face_box.y + face_box.height * 0.5;
      const double yaw_norm = clamp((cx - image_size.width * 0.5) / (image_size.width * 0.35), -1.0, 1.0);
      const double pitch_norm = clamp((cy - image_size.height * 0.45) / (image_size.height * 0.25), -1.0, 1.0);
      double landmark_yaw_norm = yaw_norm;
      if (face_obs.found && face_obs.has_nose && face_box.width > 1.0f) {
        landmark_yaw_norm = clamp(
            (face_obs.nose.x - (face_box.x + face_box.width * 0.5)) /
                std::max(1e-6, static_cast<double>(face_box.width) * 0.25),
            -1.0,
            1.0);
      }
      obs.yaw = (0.70 * yaw_norm + 0.30 * landmark_yaw_norm) * 35.0 * M_PI / 180.0;
      obs.pitch = pitch_norm * 25.0 * M_PI / 180.0;
    }
    return obs;
  }

  void apply_face_absence_gate(DriverObservation* obs) {
    if (obs->face_visible == FACE_YES) {
      face_missing_active_ = false;
      return;
    }
    if (obs->face_visible != FACE_NO) {
      return;
    }

    const auto n = now();
    if (!face_missing_active_) {
      face_missing_active_ = true;
      face_missing_since_ = n;
    }
    const double missing_ms = (n - face_missing_since_).seconds() * 1000.0;
    if (missing_ms < config_.face_absence_warning_ms) {
      obs->face_visible = FACE_UNKNOWN;
      obs->confidence = std::max(obs->confidence, 0.30);
    } else {
      obs->confidence = std::max(obs->confidence, 0.70);
    }
  }

  DriverObservation observe() {
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
      return {FACE_YES, 0.05, 0.0, 0.0, 0.0, 1.0};
    }
    const double t = std::fmod((n - t0_).seconds(), 30.0);
    if (t >= 20.0 && t <= 26.0) {
      return {FACE_YES, 0.85, 28.0 * M_PI / 180.0, 0.0, 0.7, 1.0};
    }
    return {FACE_YES, 0.05, 0.0, 0.0, 0.0, 1.0};
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
    if (!pipeline_ready()) {
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
    double score = 0.0;
    if (!bad_health(health)) {
      if (obs.face_visible == FACE_NO) {
        score = clamp(config_.face_absence_score, 0.0, 1.0);
      } else {
        score = fatigue_score(obs.face_visible, obs.eye, obs.pitch, obs.yaw, obs.distraction);
      }
    }

    ev_ads_runtime_cpp::msg::DriverState msg;
    msg.header.stamp = now();
    msg.header.frame_id = "camera_driver";
    msg.face_visible = bad_health(health) ? FACE_UNKNOWN : obs.face_visible;
    msg.eye_closure_ratio = static_cast<float>(obs.eye);
    msg.head_pitch = static_cast<float>(obs.pitch);
    msg.head_yaw = static_cast<float>(obs.yaw);
    msg.distraction_ratio = static_cast<float>(obs.distraction);
    msg.fatigue_score = static_cast<float>(score);
    msg.confidence = bad_health(health) ? 0.0f : static_cast<float>(obs.confidence);
    msg.health = health;
    pub_->publish(msg);
  }

  RuntimeTopics topics_;
  DriverMonitorConfig config_;
  bool face_ready_{false};
  bool dms_ready_{false};
  bool has_model_obs_{false};
  bool has_injected_{false};
  bool has_camera_health_{false};
  bool face_missing_active_{false};
  uint8_t camera_health_{HEALTH_DISCONNECTED};
  DriverObservation injected_;
  DriverObservation model_obs_;
  rclcpp::Time injected_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time camera_health_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time model_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time face_missing_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time t0_{0, 0, RCL_ROS_TIME};
  cv::Ptr<cv::FaceDetectorYN> face_detector_;
  YoloOnnxDetector dms_detector_;
  rclcpp::Publisher<ev_ads_runtime_cpp::msg::DriverState>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sim_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr camera_health_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // 命名空间 ev_ads_runtime_cpp

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ev_ads_runtime_cpp::DriverAttentionNode>());
  rclcpp::shutdown();
  return 0;
}
