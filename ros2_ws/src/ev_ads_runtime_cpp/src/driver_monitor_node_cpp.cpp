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

#include "ev_ads_interfaces/msg/driver_state.hpp"
#include "ev_ads_runtime_cpp/common.hpp"
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

class DriverMonitorNodeCpp final : public rclcpp::Node {
 public:
  DriverMonitorNodeCpp() : Node("driver_monitor_node_cpp") {
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
    fake_mode_ = declare_parameter<std::string>("fake_mode", "scripted");
    require_camera_health_ = declare_parameter<bool>("require_camera_health", false);
    camera_timeout_ms_ = declare_parameter<int>("camera_timeout_ms", 1000);
    model_path_ = declare_parameter<std::string>("model_path", "");
    face_model_path_ = declare_parameter<std::string>("face_model_path", "");
    model_timeout_ms_ = declare_parameter<int>("model_timeout_ms", 500);
    model_input_width_ = declare_parameter<int>("model_input_width", 640);
    model_input_height_ = declare_parameter<int>("model_input_height", 640);
    model_confidence_threshold_ = declare_parameter<double>("model_confidence_threshold", 0.35);
    model_nms_threshold_ = declare_parameter<double>("model_nms_threshold", 0.45);
    model_has_objectness_ = declare_parameter<bool>("model_has_objectness", false);
    face_input_width_ = declare_parameter<int>("face_input_width", 320);
    face_input_height_ = declare_parameter<int>("face_input_height", 320);
    face_score_threshold_ = declare_parameter<double>("face_score_threshold", 0.60);
    face_nms_threshold_ = declare_parameter<double>("face_nms_threshold", 0.30);
    face_top_k_ = declare_parameter<int>("face_top_k", 5000);
    face_absence_warning_ms_ = declare_parameter<int>("face_absence_warning_ms", 1500);
    face_absence_score_ = declare_parameter<double>("face_absence_score", 0.65);
    model_class_ids_ = to_int_vector(declare_parameter<std::vector<int64_t>>(
        "model_class_ids", std::vector<int64_t>{}));
    face_class_ids_ = to_int_vector(declare_parameter<std::vector<int64_t>>(
        "face_class_ids", std::vector<int64_t>{}));
    half_eye_class_ids_ = to_int_vector(declare_parameter<std::vector<int64_t>>(
        "half_eye_class_ids", std::vector<int64_t>{1}));
    closed_eye_class_ids_ = to_int_vector(declare_parameter<std::vector<int64_t>>(
        "closed_eye_class_ids", std::vector<int64_t>{2}));
    open_eye_class_ids_ = to_int_vector(declare_parameter<std::vector<int64_t>>(
        "open_eye_class_ids", std::vector<int64_t>{0}));
    yawn_class_ids_ = to_int_vector(declare_parameter<std::vector<int64_t>>(
        "yawn_class_ids", std::vector<int64_t>{3}));
    phone_class_ids_ = to_int_vector(declare_parameter<std::vector<int64_t>>(
        "phone_class_ids", std::vector<int64_t>{5}));
    distracted_class_ids_ = to_int_vector(declare_parameter<std::vector<int64_t>>(
        "distracted_class_ids", std::vector<int64_t>{6}));
    fatigue_class_ids_ = to_int_vector(declare_parameter<std::vector<int64_t>>(
        "fatigue_class_ids", std::vector<int64_t>{}));

    pub_ = create_publisher<ev_ads_interfaces::msg::DriverState>(
        "/perception/driver_state", rclcpp::QoS(10));
    sim_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "/sim/driver_observation",
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
        "/camera/driver/health",
        rclcpp::QoS(5),
        [this](std_msgs::msg::UInt8::SharedPtr msg) {
          camera_health_ = msg->data;
          camera_health_stamp_ = now();
          has_camera_health_ = true;
        });

    if (fake_mode_ == "model") {
      load_models();
      image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
          "/camera/driver/image_raw/compressed",
          rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
          std::bind(&DriverMonitorNodeCpp::image_callback, this, std::placeholders::_1));
    }

    t0_ = now();
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_)),
        std::bind(&DriverMonitorNodeCpp::tick, this));
    RCLCPP_INFO(
        get_logger(),
        "驾驶员监测节点启动，模式=%s DMS模型=%s 人脸模型=%s",
        fake_mode_.c_str(),
        model_path_.empty() ? "<empty>" : model_path_.c_str(),
        face_model_path_.empty() ? "<empty>" : face_model_path_.c_str());
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
    if (face_model_path_.empty()) {
      RCLCPP_WARN(get_logger(), "YuNet 人脸模型路径为空，将只使用 DMS YOLO 线索");
      return;
    }
    try {
      face_detector_ = cv::FaceDetectorYN::create(
          face_model_path_,
          "",
          cv::Size(face_input_width_, face_input_height_),
          static_cast<float>(face_score_threshold_),
          static_cast<float>(face_nms_threshold_),
          face_top_k_);
      face_ready_ = !face_detector_.empty();
    } catch (const cv::Exception& e) {
      RCLCPP_ERROR(get_logger(), "YuNet 人脸模型加载失败: %s", e.what());
      face_ready_ = false;
    }
    if (face_ready_) {
      RCLCPP_INFO(get_logger(), "YuNet 人脸模型已加载: %s", face_model_path_.c_str());
    }
  }

  void load_dms_model() {
    if (model_path_.empty()) {
      RCLCPP_WARN(get_logger(), "DMS YOLO 模型路径为空，将只使用 YuNet 人脸线索");
      return;
    }
    YoloOnnxDetector::Config config;
    config.model_path = model_path_;
    config.input_size = cv::Size(model_input_width_, model_input_height_);
    config.confidence_threshold = static_cast<float>(model_confidence_threshold_);
    config.nms_threshold = static_cast<float>(model_nms_threshold_);
    config.has_objectness = model_has_objectness_;
    config.class_allowlist = model_class_ids_;

    std::string error;
    dms_ready_ = dms_detector_.load(config, &error);
    if (!dms_ready_) {
      RCLCPP_ERROR(get_logger(), "驾驶员 YOLO ONNX 加载失败: %s", error.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "驾驶员 YOLO ONNX 已加载: %s", model_path_.c_str());
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
          std::max(32, face_input_width_),
          std::max(32, face_input_height_));
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
    const YoloDetection* yolo_face = best_detection_for(detections, face_class_ids_);
    const double closed_eye = max_confidence_for(detections, closed_eye_class_ids_);
    const double half_eye = max_confidence_for(detections, half_eye_class_ids_);
    const double open_eye = max_confidence_for(detections, open_eye_class_ids_);
    const double yawn = max_confidence_for(detections, yawn_class_ids_);
    const double phone = max_confidence_for(detections, phone_class_ids_);
    const double distracted = max_confidence_for(detections, distracted_class_ids_);
    const double fatigue = max_confidence_for(detections, fatigue_class_ids_);

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
    if (missing_ms < face_absence_warning_ms_) {
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
    if (fake_mode_ == "model") {
      if (has_model_obs_ && (n - model_stamp_).seconds() * 1000.0 <= model_timeout_ms_) {
        return model_obs_;
      }
      return {};
    }
    if (fake_mode_ == "idle") {
      return {FACE_YES, 0.05, 0.0, 0.0, 0.0, 1.0};
    }
    const double t = std::fmod((n - t0_).seconds(), 30.0);
    if (t >= 20.0 && t <= 26.0) {
      return {FACE_YES, 0.85, 28.0 * M_PI / 180.0, 0.0, 0.7, 1.0};
    }
    return {FACE_YES, 0.05, 0.0, 0.0, 0.0, 1.0};
  }

  uint8_t camera_health() const {
    if (!require_camera_health_) {
      return HEALTH_OK;
    }
    if (!has_camera_health_) {
      return HEALTH_DISCONNECTED;
    }
    const auto age_ms = (now() - camera_health_stamp_).seconds() * 1000.0;
    if (age_ms > camera_timeout_ms_) {
      return HEALTH_DISCONNECTED;
    }
    return camera_health_;
  }

  uint8_t perception_health(uint8_t camera_health) const {
    if (camera_health != HEALTH_OK) {
      return camera_health;
    }
    if (fake_mode_ != "model") {
      return camera_health;
    }
    if (!pipeline_ready()) {
      return HEALTH_ERROR;
    }
    if (!has_model_obs_) {
      return HEALTH_STALE;
    }
    const auto age_ms = (now() - model_stamp_).seconds() * 1000.0;
    if (age_ms > model_timeout_ms_) {
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
        score = clamp(face_absence_score_, 0.0, 1.0);
      } else {
        score = fatigue_score(obs.face_visible, obs.eye, obs.pitch, obs.yaw, obs.distraction);
      }
    }

    ev_ads_interfaces::msg::DriverState msg;
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

  double publish_rate_hz_{10.0};
  std::string fake_mode_{"scripted"};
  bool require_camera_health_{false};
  int camera_timeout_ms_{1000};
  std::string model_path_;
  std::string face_model_path_;
  int model_timeout_ms_{500};
  int model_input_width_{640};
  int model_input_height_{640};
  double model_confidence_threshold_{0.35};
  double model_nms_threshold_{0.45};
  bool model_has_objectness_{false};
  int face_input_width_{320};
  int face_input_height_{320};
  double face_score_threshold_{0.60};
  double face_nms_threshold_{0.30};
  int face_top_k_{5000};
  int face_absence_warning_ms_{1500};
  double face_absence_score_{0.65};
  std::vector<int> model_class_ids_;
  std::vector<int> face_class_ids_;
  std::vector<int> half_eye_class_ids_;
  std::vector<int> closed_eye_class_ids_;
  std::vector<int> open_eye_class_ids_;
  std::vector<int> yawn_class_ids_;
  std::vector<int> phone_class_ids_;
  std::vector<int> distracted_class_ids_;
  std::vector<int> fatigue_class_ids_;
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
  rclcpp::Publisher<ev_ads_interfaces::msg::DriverState>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sim_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr camera_health_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // 命名空间 ev_ads_runtime_cpp

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ev_ads_runtime_cpp::DriverMonitorNodeCpp>());
  rclcpp::shutdown();
  return 0;
}
