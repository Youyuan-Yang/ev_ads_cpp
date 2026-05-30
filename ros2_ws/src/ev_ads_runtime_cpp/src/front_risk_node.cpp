// 前向风险节点：输出前车、行人、障碍物、坑洼等前向风险统一接口。
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "ev_ads_runtime_cpp/msg/front_risk.hpp"
#include "ev_ads_runtime_cpp/risk_math.hpp"
#include "ev_ads_runtime_cpp/topics.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"

using namespace std::chrono_literals;

namespace ev_ads_runtime_cpp {

struct FrontObservation {
  uint8_t cls = CLASS_NONE;
  double distance_m = 0.0;
  double closing_speed_mps = 0.0;
  double lateral_offset_m = 0.0;
};

class FrontRiskNode final : public rclcpp::Node {
 public:
  FrontRiskNode() : Node("front_risk_node") {
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
    fake_mode_ = declare_parameter<std::string>("fake_mode", "scripted");
    require_camera_health_ = declare_parameter<bool>("require_camera_health", false);
    camera_timeout_ms_ = declare_parameter<int>("camera_timeout_ms", 1000);

    pub_ = create_publisher<ev_ads_runtime_cpp::msg::FrontRisk>(
        topics_.front_risk, rclcpp::QoS(10));
    sim_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        topics_.sim_front_observation,
        rclcpp::QoS(10),
        [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
          if (msg->data.size() < 3) {
            return;
          }
          injected_.cls = static_cast<uint8_t>(msg->data[0]);
          injected_.distance_m = msg->data[1];
          injected_.closing_speed_mps = msg->data[2];
          injected_.lateral_offset_m = msg->data.size() > 3 ? msg->data[3] : 0.0;
          injected_stamp_ = now();
          has_injected_ = true;
        });
    camera_health_sub_ = create_subscription<std_msgs::msg::UInt8>(
        RuntimeTopics::health_topic(topics_.camera_front_ns),
        rclcpp::QoS(5),
        [this](std_msgs::msg::UInt8::SharedPtr msg) {
          camera_health_ = msg->data;
          camera_health_stamp_ = now();
          has_camera_health_ = true;
        });

    t0_ = now();
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_)),
        std::bind(&FrontRiskNode::tick, this));
    RCLCPP_INFO(
        get_logger(),
        "前向感知节点启动，模式=%s 需要摄像头健康=%s",
        fake_mode_.c_str(),
        require_camera_health_ ? "true" : "false");
  }

 private:
  FrontObservation observe() {
    const auto n = now();
    if (has_injected_ && (n - injected_stamp_).seconds() < 1.0) {
      return injected_;
    }
    if (fake_mode_ == "idle" || fake_mode_ == "model") {
      return {};
    }
    const double t = std::fmod((n - t0_).seconds(), 30.0);
    if (t > 25.0) {
      return {};
    }
    FrontObservation obs;
    obs.cls = CLASS_PEDESTRIAN;
    obs.distance_m = std::max(2.0, 30.0 - t * 1.12);
    obs.closing_speed_mps = 1.12;
    obs.lateral_offset_m = 0.2;
    return obs;
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

  void tick() {
    const auto obs = observe();
    const uint8_t health = camera_health();
    const double ttc = estimate_ttc(obs.distance_m, obs.closing_speed_mps);
    double score = front_risk_score(
        obs.cls, obs.distance_m, obs.closing_speed_mps, obs.lateral_offset_m);
    if (bad_health(health)) {
      score = 0.0;
    }

    ev_ads_runtime_cpp::msg::FrontRisk msg;
    msg.header.stamp = now();
    msg.header.frame_id = "camera_front";
    msg.primary_class = obs.cls;
    msg.ttc = std::isfinite(ttc) ? static_cast<float>(ttc) : 1e6f;
    msg.distance = static_cast<float>(obs.distance_m);
    msg.closing_speed = static_cast<float>(obs.closing_speed_mps);
    msg.lateral_offset = static_cast<float>(obs.lateral_offset_m);
    msg.risk_score = static_cast<float>(score);
    msg.confidence = obs.cls == CLASS_NONE ? 1.0f : 0.6f;
    msg.health = health;
    pub_->publish(msg);
  }

  double publish_rate_hz_{10.0};
  std::string fake_mode_{"scripted"};
  bool require_camera_health_{false};
  int camera_timeout_ms_{1000};
  RuntimeTopics topics_;
  bool has_injected_{false};
  bool has_camera_health_{false};
  uint8_t camera_health_{HEALTH_DISCONNECTED};
  FrontObservation injected_;
  rclcpp::Time injected_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time camera_health_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time t0_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<ev_ads_runtime_cpp::msg::FrontRisk>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sim_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr camera_health_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // 命名空间 ev_ads_runtime_cpp

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ev_ads_runtime_cpp::FrontRiskNode>());
  rclcpp::shutdown();
  return 0;
}
