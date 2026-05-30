// 多模态风险融合节点：汇总前向、后向、DMS、IMU、毫米波并输出告警/制动门控。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "ev_ads_runtime_cpp/msg/blind_spot_state.hpp"
#include "ev_ads_runtime_cpp/msg/brake_command.hpp"
#include "ev_ads_runtime_cpp/msg/driver_state.hpp"
#include "ev_ads_runtime_cpp/msg/front_risk.hpp"
#include "ev_ads_runtime_cpp/msg/mm_wave_vital.hpp"
#include "ev_ads_runtime_cpp/msg/risk_state.hpp"
#include "ev_ads_runtime_cpp/msg/vehicle_motion.hpp"
#include "ev_ads_runtime_cpp/msg/warning_command.hpp"
#include "ev_ads_runtime_cpp/risk_math.hpp"
#include "ev_ads_runtime_cpp/risk_fusion_core.hpp"
#include "ev_ads_runtime_cpp/runtime_config.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ev_ads_runtime_cpp {
namespace {

FusionConfig read_fusion_config(rclcpp::Node* node) {
  FusionConfig cfg;
  cfg.publish_rate_hz = node->declare_parameter<double>("publish_rate_hz", cfg.publish_rate_hz);
  cfg.enable_real_brake = node->declare_parameter<bool>("enable_real_brake", cfg.enable_real_brake);
  cfg.profile = node->declare_parameter<std::string>("profile", cfg.profile);
  cfg.weights.front = node->declare_parameter<double>("w_front", cfg.weights.front);
  cfg.weights.rear = node->declare_parameter<double>("w_rear", cfg.weights.rear);
  cfg.weights.driver = node->declare_parameter<double>("w_driver", cfg.weights.driver);
  cfg.weights.imu = node->declare_parameter<double>("w_imu", cfg.weights.imu);
  cfg.weights.mmwave = node->declare_parameter<double>("w_mmwave", cfg.weights.mmwave);
  cfg.thr_l1 = node->declare_parameter<double>("thr_l1", cfg.thr_l1);
  cfg.thr_l2 = node->declare_parameter<double>("thr_l2", cfg.thr_l2);
  cfg.thr_l3 = node->declare_parameter<double>("thr_l3", cfg.thr_l3);
  cfg.hysteresis = node->declare_parameter<double>("hysteresis", cfg.hysteresis);
  cfg.front_ttc_l3 = node->declare_parameter<double>("front_ttc_l3", cfg.front_ttc_l3);
  cfg.zero_stale = node->declare_parameter<bool>("zero_stale", cfg.zero_stale);
  cfg.stale_reliability = node->declare_parameter<double>("stale_reliability", cfg.stale_reliability);
  cfg.weighted_gain = node->declare_parameter<double>("weighted_gain", cfg.weighted_gain);
  cfg.probabilistic_gain =
      node->declare_parameter<double>("probabilistic_gain", cfg.probabilistic_gain);
  cfg.synergy_gain = node->declare_parameter<double>("synergy_gain", cfg.synergy_gain);
  cfg.consensus_bonus = node->declare_parameter<double>("consensus_bonus", cfg.consensus_bonus);
  cfg.front_emergency_ttc =
      node->declare_parameter<double>("front_emergency_ttc", cfg.front_emergency_ttc);
  cfg.front_warning_ttc =
      node->declare_parameter<double>("front_warning_ttc", cfg.front_warning_ttc);
  cfg.front_l3_floor = node->declare_parameter<double>("front_l3_floor", cfg.front_l3_floor);

  cfg.timeouts.front_stale_ms =
      node->declare_parameter<int>("front_stale_ms", cfg.timeouts.front_stale_ms);
  cfg.timeouts.front_disc_ms =
      node->declare_parameter<int>("front_disc_ms", cfg.timeouts.front_disc_ms);
  cfg.timeouts.rear_stale_ms =
      node->declare_parameter<int>("rear_stale_ms", cfg.timeouts.rear_stale_ms);
  cfg.timeouts.rear_disc_ms =
      node->declare_parameter<int>("rear_disc_ms", cfg.timeouts.rear_disc_ms);
  cfg.timeouts.driver_stale_ms =
      node->declare_parameter<int>("driver_stale_ms", cfg.timeouts.driver_stale_ms);
  cfg.timeouts.driver_disc_ms =
      node->declare_parameter<int>("driver_disc_ms", cfg.timeouts.driver_disc_ms);
  cfg.timeouts.imu_stale_ms =
      node->declare_parameter<int>("imu_stale_ms", cfg.timeouts.imu_stale_ms);
  cfg.timeouts.imu_disc_ms =
      node->declare_parameter<int>("imu_disc_ms", cfg.timeouts.imu_disc_ms);
  cfg.timeouts.mmwave_stale_ms =
      node->declare_parameter<int>("mmwave_stale_ms", cfg.timeouts.mmwave_stale_ms);
  cfg.timeouts.mmwave_disc_ms =
      node->declare_parameter<int>("mmwave_disc_ms", cfg.timeouts.mmwave_disc_ms);
  return cfg;
}

}  // namespace

class RiskFusionNode final : public rclcpp::Node {
 public:
  RiskFusionNode() : Node("risk_fusion_node"), config_(read_fusion_config(this)), core_(config_) {
    sub_front_ = create_subscription<ev_ads_runtime_cpp::msg::FrontRisk>(
        topics_.front_risk, rclcpp::QoS(10),
        [this](ev_ads_runtime_cpp::msg::FrontRisk::SharedPtr msg) {
          snap_.front = msg->risk_score;
          snap_.front_health = health_from_ros(msg->health);
          snap_.front_ttc = msg->ttc >= 1e5f ? std::numeric_limits<double>::infinity() : msg->ttc;
          snap_.front_class = object_class_from_ros(msg->primary_class);
          seen_.front = now();
        });
    sub_rear_ = create_subscription<ev_ads_runtime_cpp::msg::BlindSpotState>(
        topics_.blind_spot, rclcpp::QoS(10),
        [this](ev_ads_runtime_cpp::msg::BlindSpotState::SharedPtr msg) {
          snap_.rear = msg->risk_score;
          snap_.rear_health = health_from_ros(msg->health);
          snap_.rear_zone_left = zone_from_ros(msg->zone_left);
          snap_.rear_zone_right = zone_from_ros(msg->zone_right);
          seen_.rear = now();
        });
    sub_driver_ = create_subscription<ev_ads_runtime_cpp::msg::DriverState>(
        topics_.driver_state, rclcpp::QoS(10),
        [this](ev_ads_runtime_cpp::msg::DriverState::SharedPtr msg) {
          snap_.driver = msg->fatigue_score;
          snap_.driver_health = health_from_ros(msg->health);
          seen_.driver = now();
        });
    sub_motion_ = create_subscription<ev_ads_runtime_cpp::msg::VehicleMotion>(
        topics_.vehicle_motion, rclcpp::QoS(50),
        [this](ev_ads_runtime_cpp::msg::VehicleMotion::SharedPtr msg) {
          snap_.imu = imu_score(msg->motion_flags);
          snap_.imu_health = health_from_ros(msg->health);
          seen_.imu = now();
        });
    sub_mmwave_ = create_subscription<ev_ads_runtime_cpp::msg::MmWaveVital>(
        topics_.mmwave_vital, rclcpp::QoS(10),
        [this](ev_ads_runtime_cpp::msg::MmWaveVital::SharedPtr msg) {
          snap_.mmwave = mmwave_score(msg->heart_rate, msg->breath_rate, msg->confidence);
          snap_.mmwave_health = health_from_ros(msg->health);
          seen_.mmwave = now();
        });

    pub_risk_ = create_publisher<ev_ads_runtime_cpp::msg::RiskState>(topics_.risk_state, rclcpp::QoS(10));
    pub_warn_ =
        create_publisher<ev_ads_runtime_cpp::msg::WarningCommand>(topics_.warning_cmd, rclcpp::QoS(10));
    pub_brake_ =
        create_publisher<ev_ads_runtime_cpp::msg::BrakeCommand>(topics_.brake_cmd, rclcpp::QoS(1));

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(1.0, config_.publish_rate_hz)),
        std::bind(&RiskFusionNode::tick, this));

    RCLCPP_INFO(
        get_logger(),
        "融合节点启动，配置=%s 模型=FusionCore 真实制动=%s 过期置零=%s",
        config_.profile.c_str(),
        config_.enable_real_brake ? "true" : "false",
        config_.zero_stale ? "true" : "false");
  }

 private:
  struct SeenTimes {
    rclcpp::Time front{0, 0, RCL_ROS_TIME};
    rclcpp::Time rear{0, 0, RCL_ROS_TIME};
    rclcpp::Time driver{0, 0, RCL_ROS_TIME};
    rclcpp::Time imu{0, 0, RCL_ROS_TIME};
    rclcpp::Time mmwave{0, 0, RCL_ROS_TIME};
  };

  Health health_from_age(const rclcpp::Time& seen, Health current, int stale_ms, int disc_ms) const {
    if (seen.nanoseconds() == 0) {
      return Health::kDisconnected;
    }
    const double age_ms = (now() - seen).seconds() * 1000.0;
    if (age_ms >= disc_ms) {
      return Health::kDisconnected;
    }
    if (age_ms >= stale_ms) {
      return Health::kStale;
    }
    return current;
  }

  FusionSnapshot fresh_snapshot() const {
    FusionSnapshot s = snap_;
    s.front_health = health_from_age(
        seen_.front, s.front_health, config_.timeouts.front_stale_ms, config_.timeouts.front_disc_ms);
    s.rear_health = health_from_age(
        seen_.rear, s.rear_health, config_.timeouts.rear_stale_ms, config_.timeouts.rear_disc_ms);
    s.driver_health = health_from_age(
        seen_.driver, s.driver_health, config_.timeouts.driver_stale_ms, config_.timeouts.driver_disc_ms);
    s.imu_health =
        health_from_age(seen_.imu, s.imu_health, config_.timeouts.imu_stale_ms, config_.timeouts.imu_disc_ms);
    s.mmwave_health = health_from_age(
        seen_.mmwave, s.mmwave_health, config_.timeouts.mmwave_stale_ms, config_.timeouts.mmwave_disc_ms);
    return s;
  }

  void publish_warning_if_needed(const FusionDecision& decision, const std_msgs::msg::Header& header) {
    if (to_ros(decision.level) <= to_ros(last_voice_level_) || decision.level == WarningLevel::kL0) {
      last_voice_level_ = decision.level;
      return;
    }

    ev_ads_runtime_cpp::msg::WarningCommand msg;
    msg.header = header;
    msg.level = to_ros(decision.level);
    msg.voice_text = voice_text_for_reason(decision.primary_reason);
    msg.vibration_pattern =
        decision.level == WarningLevel::kL1 ? 1 : (decision.level == WarningLevel::kL2 ? 2 : 3);
    msg.beep_freq_hz =
        decision.level == WarningLevel::kL1 ? 0 : (decision.level == WarningLevel::kL2 ? 880 : 1600);
    msg.duration_ms = decision.level == WarningLevel::kL3 ? 2000 : 600;
    msg.screen_icon = decision.primary_reason;
    pub_warn_->publish(msg);
    last_voice_level_ = decision.level;
  }

  void publish_brake(const FusionDecision& decision, const FusionSnapshot& s, const std_msgs::msg::Header& header) {
    ev_ads_runtime_cpp::msg::BrakeCommand msg;
    msg.header = header;
    msg.source = decision.primary_reason;
    uint8_t gates = 0;
    if (s.front_health == Health::kOk) {
      gates |= 0x01;
    }
    if (s.imu_health == Health::kOk) {
      gates |= 0x02;
    }
    if (config_.enable_real_brake) {
      gates |= 0x04;
    }
    msg.safety_gates_passed = gates;
    if (decision.level == WarningLevel::kL3 && gates == 0x07 &&
        contains_action(decision.allowed_actions, "brake_demand_request")) {
      msg.enable = true;
      msg.demand = 0.5f;
      msg.max_duration_ms = 1500;
    } else {
      msg.enable = false;
      msg.demand = 0.0f;
      msg.max_duration_ms = 0;
    }
    pub_brake_->publish(msg);
  }

  void tick() {
    const FusionSnapshot s = fresh_snapshot();
    const FusionDecision decision = core_.decide(s);

    ev_ads_runtime_cpp::msg::RiskState risk;
    risk.header.stamp = now();
    risk.header.frame_id = "vehicle";
    risk.level = to_ros(decision.level);
    risk.score = static_cast<float>(decision.score);
    risk.primary_reason = decision.primary_reason;
    risk.reasons = decision.reasons;
    risk.allowed_actions = decision.allowed_actions;
    risk.sensor_health_summary = decision.sensor_health_summary;
    pub_risk_->publish(risk);

    publish_warning_if_needed(decision, risk.header);
    publish_brake(decision, s, risk.header);
  }

  RuntimeTopics topics_;
  FusionConfig config_;
  FusionCore core_;
  FusionSnapshot snap_;
  SeenTimes seen_;
  WarningLevel last_voice_level_{WarningLevel::kL0};

  rclcpp::Publisher<ev_ads_runtime_cpp::msg::RiskState>::SharedPtr pub_risk_;
  rclcpp::Publisher<ev_ads_runtime_cpp::msg::WarningCommand>::SharedPtr pub_warn_;
  rclcpp::Publisher<ev_ads_runtime_cpp::msg::BrakeCommand>::SharedPtr pub_brake_;
  rclcpp::Subscription<ev_ads_runtime_cpp::msg::FrontRisk>::SharedPtr sub_front_;
  rclcpp::Subscription<ev_ads_runtime_cpp::msg::BlindSpotState>::SharedPtr sub_rear_;
  rclcpp::Subscription<ev_ads_runtime_cpp::msg::DriverState>::SharedPtr sub_driver_;
  rclcpp::Subscription<ev_ads_runtime_cpp::msg::VehicleMotion>::SharedPtr sub_motion_;
  rclcpp::Subscription<ev_ads_runtime_cpp::msg::MmWaveVital>::SharedPtr sub_mmwave_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ev_ads_runtime_cpp

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ev_ads_runtime_cpp::RiskFusionNode>());
  rclcpp::shutdown();
  return 0;
}
