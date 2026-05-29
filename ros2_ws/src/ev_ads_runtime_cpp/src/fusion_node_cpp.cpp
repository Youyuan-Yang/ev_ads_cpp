#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "ev_ads_interfaces/msg/blind_spot_state.hpp"
#include "ev_ads_interfaces/msg/brake_command.hpp"
#include "ev_ads_interfaces/msg/driver_state.hpp"
#include "ev_ads_interfaces/msg/front_risk.hpp"
#include "ev_ads_interfaces/msg/mm_wave_vital.hpp"
#include "ev_ads_interfaces/msg/risk_state.hpp"
#include "ev_ads_interfaces/msg/vehicle_motion.hpp"
#include "ev_ads_interfaces/msg/warning_command.hpp"
#include "ev_ads_runtime_cpp/common.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ev_ads_runtime_cpp {

struct Snapshot {
  double front = 0.0;
  double rear = 0.0;
  double driver = 0.0;
  double imu = 0.0;
  double mmwave = 0.0;

  uint8_t front_health = HEALTH_DISCONNECTED;
  uint8_t rear_health = HEALTH_DISCONNECTED;
  uint8_t driver_health = HEALTH_DISCONNECTED;
  uint8_t imu_health = HEALTH_DISCONNECTED;
  uint8_t mmwave_health = HEALTH_DISCONNECTED;

  double front_ttc = std::numeric_limits<double>::infinity();
  uint8_t front_class = CLASS_NONE;
  uint8_t rear_zone_left = ZONE_CLEAR;
  uint8_t rear_zone_right = ZONE_CLEAR;
};

struct Weights {
  double front = 0.35;
  double rear = 0.25;
  double driver = 0.20;
  double imu = 0.15;
  double mmwave = 0.05;
};

struct FusionResult {
  uint8_t level = LEVEL_L0;
  double score = 0.0;
  std::string primary_reason = "none";
  std::vector<std::string> reasons;
  std::vector<std::string> allowed_actions;
  uint8_t sensor_health_summary = 0;
};

class FusionNodeCpp final : public rclcpp::Node {
 public:
  FusionNodeCpp() : Node("fusion_node_cpp") {
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);
    enable_real_brake_ = declare_parameter<bool>("enable_real_brake", false);
    profile_ = declare_parameter<std::string>("profile", "urban_day");
    weights_.front = declare_parameter<double>("w_front", 0.35);
    weights_.rear = declare_parameter<double>("w_rear", 0.25);
    weights_.driver = declare_parameter<double>("w_driver", 0.20);
    weights_.imu = declare_parameter<double>("w_imu", 0.15);
    weights_.mmwave = declare_parameter<double>("w_mmwave", 0.05);
    thr_l1_ = declare_parameter<double>("thr_l1", 0.30);
    thr_l2_ = declare_parameter<double>("thr_l2", 0.55);
    thr_l3_ = declare_parameter<double>("thr_l3", 0.75);
    hysteresis_ = declare_parameter<double>("hysteresis", 0.05);
    front_ttc_l3_ = declare_parameter<double>("front_ttc_l3", 1.2);
    zero_stale_ = declare_parameter<bool>("zero_stale", true);
    stale_reliability_ = declare_parameter<double>("stale_reliability", 0.25);
    weighted_gain_ = declare_parameter<double>("weighted_gain", 0.72);
    probabilistic_gain_ = declare_parameter<double>("probabilistic_gain", 0.28);
    synergy_gain_ = declare_parameter<double>("synergy_gain", 0.12);
    consensus_bonus_ = declare_parameter<double>("consensus_bonus", 0.08);
    front_emergency_ttc_ = declare_parameter<double>("front_emergency_ttc", 0.8);
    front_warning_ttc_ = declare_parameter<double>("front_warning_ttc", 2.5);
    front_l3_floor_ = declare_parameter<double>("front_l3_floor", 0.78);

    front_stale_ms_ = declare_parameter<int>("front_stale_ms", 500);
    front_disc_ms_ = declare_parameter<int>("front_disc_ms", 3000);
    rear_stale_ms_ = declare_parameter<int>("rear_stale_ms", 500);
    rear_disc_ms_ = declare_parameter<int>("rear_disc_ms", 3000);
    driver_stale_ms_ = declare_parameter<int>("driver_stale_ms", 500);
    driver_disc_ms_ = declare_parameter<int>("driver_disc_ms", 3000);
    imu_stale_ms_ = declare_parameter<int>("imu_stale_ms", 200);
    imu_disc_ms_ = declare_parameter<int>("imu_disc_ms", 1000);
    mmwave_stale_ms_ = declare_parameter<int>("mmwave_stale_ms", 500);
    mmwave_disc_ms_ = declare_parameter<int>("mmwave_disc_ms", 3000);

    sub_front_ = create_subscription<ev_ads_interfaces::msg::FrontRisk>(
        "/perception/front_risk", rclcpp::QoS(10),
        [this](ev_ads_interfaces::msg::FrontRisk::SharedPtr msg) {
          snap_.front = msg->risk_score;
          snap_.front_health = msg->health;
          snap_.front_ttc = msg->ttc >= 1e5f ? std::numeric_limits<double>::infinity() : msg->ttc;
          snap_.front_class = msg->primary_class;
          front_seen_ = now();
        });
    sub_rear_ = create_subscription<ev_ads_interfaces::msg::BlindSpotState>(
        "/perception/blind_spot", rclcpp::QoS(10),
        [this](ev_ads_interfaces::msg::BlindSpotState::SharedPtr msg) {
          snap_.rear = msg->risk_score;
          snap_.rear_health = msg->health;
          snap_.rear_zone_left = msg->zone_left;
          snap_.rear_zone_right = msg->zone_right;
          rear_seen_ = now();
        });
    sub_driver_ = create_subscription<ev_ads_interfaces::msg::DriverState>(
        "/perception/driver_state", rclcpp::QoS(10),
        [this](ev_ads_interfaces::msg::DriverState::SharedPtr msg) {
          snap_.driver = msg->fatigue_score;
          snap_.driver_health = msg->health;
          driver_seen_ = now();
        });
    sub_motion_ = create_subscription<ev_ads_interfaces::msg::VehicleMotion>(
        "/vehicle/motion", rclcpp::QoS(50),
        [this](ev_ads_interfaces::msg::VehicleMotion::SharedPtr msg) {
          snap_.imu = imu_score(msg->motion_flags);
          snap_.imu_health = msg->health;
          imu_seen_ = now();
        });
    sub_mmwave_ = create_subscription<ev_ads_interfaces::msg::MmWaveVital>(
        "/sensor/mmwave/vital", rclcpp::QoS(10),
        [this](ev_ads_interfaces::msg::MmWaveVital::SharedPtr msg) {
          snap_.mmwave = mmwave_score(msg->heart_rate, msg->breath_rate, msg->confidence);
          snap_.mmwave_health = msg->health;
          mmwave_seen_ = now();
        });

    pub_risk_ = create_publisher<ev_ads_interfaces::msg::RiskState>(
        "/decision/risk_state", rclcpp::QoS(10));
    pub_warn_ = create_publisher<ev_ads_interfaces::msg::WarningCommand>(
        "/decision/warning_cmd", rclcpp::QoS(10));
    pub_brake_ = create_publisher<ev_ads_interfaces::msg::BrakeCommand>(
        "/decision/brake_cmd", rclcpp::QoS(1));

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_)),
        std::bind(&FusionNodeCpp::tick, this));

    RCLCPP_INFO(
        get_logger(),
        "融合节点启动，配置=%s 模型=v2 真实制动=%s 过期置零=%s "
        "增益(加权=%.2f 概率=%.2f 协同=%.2f)",
        profile_.c_str(),
        enable_real_brake_ ? "true" : "false",
        zero_stale_ ? "true" : "false",
        weighted_gain_,
        probabilistic_gain_,
        synergy_gain_);
  }

 private:
  uint8_t health_from_age(const rclcpp::Time& seen, uint8_t current, int stale_ms, int disc_ms) const {
    if (seen.nanoseconds() == 0) {
      return HEALTH_DISCONNECTED;
    }
    const double age_ms = (now() - seen).seconds() * 1000.0;
    if (age_ms >= disc_ms) {
      return HEALTH_DISCONNECTED;
    }
    if (age_ms >= stale_ms) {
      return HEALTH_STALE;
    }
    return current;
  }

  Snapshot fresh_snapshot() const {
    Snapshot s = snap_;
    s.front_health = health_from_age(front_seen_, s.front_health, front_stale_ms_, front_disc_ms_);
    s.rear_health = health_from_age(rear_seen_, s.rear_health, rear_stale_ms_, rear_disc_ms_);
    s.driver_health = health_from_age(driver_seen_, s.driver_health, driver_stale_ms_, driver_disc_ms_);
    s.imu_health = health_from_age(imu_seen_, s.imu_health, imu_stale_ms_, imu_disc_ms_);
    s.mmwave_health = health_from_age(mmwave_seen_, s.mmwave_health, mmwave_stale_ms_, mmwave_disc_ms_);
    return s;
  }

  double health_reliability(uint8_t health) const {
    if (health == HEALTH_OK) {
      return 1.0;
    }
    if (health == HEALTH_STALE) {
      return zero_stale_ ? 0.0 : ev_ads_runtime_cpp::clamp(stale_reliability_, 0.0, 1.0);
    }
    return 0.0;
  }

  bool unavailable(uint8_t health) const {
    return health_reliability(health) <= 0.0;
  }

  Weights adaptive_weights(const Snapshot& s, std::vector<std::string>* degraded) const {
    Weights w = weights_;
    const double rf = health_reliability(s.front_health);
    const double rr = health_reliability(s.rear_health);
    const double rd = health_reliability(s.driver_health);
    const double ri = health_reliability(s.imu_health);
    const double rm = health_reliability(s.mmwave_health);

    if (rf <= 0.0) {
      w.front = 0.0;
      degraded->push_back("front_cam_down");
    } else {
      w.front *= rf;
    }
    if (rr <= 0.0) {
      w.rear = 0.0;
      degraded->push_back("rear_cam_down");
    } else {
      w.rear *= rr;
    }
    if (rd <= 0.0) {
      w.driver = 0.0;
      degraded->push_back("driver_cam_down");
    } else {
      w.driver *= rd;
    }
    if (ri <= 0.0) {
      w.imu = 0.0;
      degraded->push_back("imu_down");
    } else {
      w.imu *= ri;
    }
    if (rm <= 0.0) {
      w.mmwave = 0.0;
      degraded->push_back("mmwave_down");
    } else {
      w.mmwave *= rm;
    }
    const double total = w.front + w.rear + w.driver + w.imu + w.mmwave;
    if (total > 0.0) {
      w.front /= total;
      w.rear /= total;
      w.driver /= total;
      w.imu /= total;
      w.mmwave /= total;
    }
    return w;
  }

  double ttc_urgency(double ttc) const {
    if (!std::isfinite(ttc) || ttc <= 0.0) {
      return 0.0;
    }
    if (ttc <= front_emergency_ttc_) {
      return 1.0;
    }
    if (ttc <= front_ttc_l3_) {
      const double span = std::max(1e-6, front_ttc_l3_ - front_emergency_ttc_);
      return 0.85 + 0.15 * (front_ttc_l3_ - ttc) / span;
    }
    if (ttc <= front_warning_ttc_) {
      const double span = std::max(1e-6, front_warning_ttc_ - front_ttc_l3_);
      return 0.25 + 0.55 * (front_warning_ttc_ - ttc) / span;
    }
    return 0.0;
  }

  double probabilistic_evidence(const Snapshot& s) const {
    const double rf = health_reliability(s.front_health);
    const double rr = health_reliability(s.rear_health);
    const double rd = health_reliability(s.driver_health);
    const double ri = health_reliability(s.imu_health);
    const double rm = health_reliability(s.mmwave_health);
    const std::vector<double> evidences = {
        ev_ads_runtime_cpp::clamp(s.front * rf * 0.55, 0.0, 0.95),
        ev_ads_runtime_cpp::clamp(s.rear * rr * 0.40, 0.0, 0.85),
        ev_ads_runtime_cpp::clamp(s.driver * rd * 0.30, 0.0, 0.75),
        ev_ads_runtime_cpp::clamp(s.imu * ri * 0.25, 0.0, 0.70),
        ev_ads_runtime_cpp::clamp(s.mmwave * rm * 0.12, 0.0, 0.20),
    };
    double p_no_risk = 1.0;
    for (double evidence : evidences) {
      p_no_risk *= 1.0 - evidence;
    }
    return 1.0 - p_no_risk;
  }

  double synergy_bonus(const Snapshot& s, std::vector<std::string>* reasons) const {
    const double rf = health_reliability(s.front_health);
    const double rr = health_reliability(s.rear_health);
    const double rd = health_reliability(s.driver_health);
    const double ri = health_reliability(s.imu_health);
    const double rm = health_reliability(s.mmwave_health);
    double bonus = 0.0;

    const double front_driver = std::min(s.front * rf, s.driver * rd);
    if (front_driver > 0.35) {
      bonus += synergy_gain_ * front_driver;
      reasons->push_back("synergy_front_driver");
    }
    const double front_imu = std::min(s.front * rf, s.imu * ri);
    if (front_imu > 0.35) {
      bonus += synergy_gain_ * 0.8 * front_imu;
      reasons->push_back("synergy_front_imu");
    }
    const double rear_imu = std::min(s.rear * rr, s.imu * ri);
    if (rear_imu > 0.35) {
      bonus += synergy_gain_ * 0.6 * rear_imu;
      reasons->push_back("synergy_rear_imu");
    }
    const double driver_mmwave = std::min(s.driver * rd, s.mmwave * rm);
    if (driver_mmwave > 0.35) {
      bonus += synergy_gain_ * 0.45 * driver_mmwave;
      reasons->push_back("synergy_driver_vital");
    }

    const int active =
        (s.front * rf > 0.45 ? 1 : 0) +
        (s.rear * rr > 0.45 ? 1 : 0) +
        (s.driver * rd > 0.45 ? 1 : 0) +
        (s.imu * ri > 0.45 ? 1 : 0) +
        (s.mmwave * rm > 0.45 ? 1 : 0);
    if (active >= 3) {
      bonus += consensus_bonus_;
      reasons->push_back("multi_modal_consensus");
    }
    return ev_ads_runtime_cpp::clamp(bonus, 0.0, 0.22);
  }

  static std::vector<std::string> unique_reasons(const std::vector<std::string>& values) {
    std::vector<std::string> out;
    for (const auto& value : values) {
      if (!value.empty() && std::find(out.begin(), out.end(), value) == out.end()) {
        out.push_back(value);
      }
    }
    return out;
  }

  std::string primary_for_l3(const Snapshot& s) const {
    if (s.front_health == HEALTH_OK && s.front_ttc < front_ttc_l3_ && s.front > 0.5) {
      return "front_ttc_low";
    }
    return "";
  }

  uint8_t sensor_summary(const Snapshot& s) const {
    uint8_t bits = 0;
    if (s.front_health == HEALTH_OK) {
      bits |= 0x01;
    }
    if (s.rear_health == HEALTH_OK) {
      bits |= 0x02;
    }
    if (s.driver_health == HEALTH_OK) {
      bits |= 0x04;
    }
    if (s.imu_health == HEALTH_OK) {
      bits |= 0x08;
    }
    if (s.mmwave_health == HEALTH_OK) {
      bits |= 0x10;
    }
    return bits;
  }

  std::vector<std::string> actions(uint8_t level, const Snapshot& s) const {
    std::vector<std::string> out;
    if (level >= LEVEL_L1) {
      out.push_back("screen_hint");
    }
    if (level >= LEVEL_L2) {
      out.push_back("warn_voice");
      out.push_back("warn_vibration");
    }
    if (level == LEVEL_L3) {
      if (s.front_health == HEALTH_OK && s.imu_health == HEALTH_OK) {
        out.push_back("brake_demand_request");
      } else {
        out.push_back("brake_blocked_safety_gate");
      }
    }
    return out;
  }

  FusionResult decide(const Snapshot& s) {
    std::vector<std::string> degraded;
    const Weights w = adaptive_weights(s, &degraded);
    const double f = unavailable(s.front_health) ? 0.0 : s.front;
    const double r = unavailable(s.rear_health) ? 0.0 : s.rear;
    const double d = unavailable(s.driver_health) ? 0.0 : s.driver;
    const double i = unavailable(s.imu_health) ? 0.0 : s.imu;
    const double m = unavailable(s.mmwave_health) ? 0.0 : s.mmwave;

    struct Contribution {
      std::string name;
      double value;
    };
    std::vector<std::string> reasons;
    std::vector<Contribution> contrib = {
        {s.front_ttc < front_ttc_l3_ ? "front_ttc_low" : "front_risk", w.front * f},
        {s.rear_zone_right == ZONE_APPROACHING
             ? "rear_right_approaching"
             : (s.rear_zone_left == ZONE_APPROACHING ? "rear_left_approaching" : "rear_risk"),
         w.rear * r},
        {"driver_fatigue", w.driver * d},
        {"imu_lean_or_brake", w.imu * i},
        {"mmwave_vital_abnormal", w.mmwave * m},
    };
    std::sort(contrib.begin(), contrib.end(), [](const Contribution& a, const Contribution& b) {
      return a.value > b.value;
    });
    for (const auto& c : contrib) {
      if (c.value > 0.01) {
        reasons.push_back(c.name);
      }
    }

    const double weighted_score = w.front * f + w.rear * r + w.driver * d + w.imu * i + w.mmwave * m;
    const double probabilistic_score = probabilistic_evidence(s);
    std::vector<std::string> synergy_reasons;
    const double synergy = synergy_bonus(s, &synergy_reasons);
    double score = weighted_gain_ * weighted_score + probabilistic_gain_ * probabilistic_score + synergy;

    const double urgency = ttc_urgency(s.front_ttc);
    if (s.front_health == HEALTH_OK && s.front > 0.5 && urgency > 0.0) {
      if (s.front_ttc <= front_emergency_ttc_) {
        score = std::max(score, front_l3_floor_);
      } else if (s.front_ttc <= front_ttc_l3_) {
        score = std::max(score, 0.70 + 0.08 * s.front);
      } else {
        score = std::max(score, 0.45 + 0.25 * urgency * s.front);
      }
    }
    score = ev_ads_runtime_cpp::clamp(score, 0.0, 1.0);

    reasons.insert(reasons.end(), synergy_reasons.begin(), synergy_reasons.end());
    reasons.insert(reasons.end(), degraded.begin(), degraded.end());
    reasons = unique_reasons(reasons);

    uint8_t candidate = LEVEL_L0;
    if (score >= thr_l3_) {
      candidate = LEVEL_L3;
    } else if (score >= thr_l2_) {
      candidate = LEVEL_L2;
    } else if (score >= thr_l1_) {
      candidate = LEVEL_L1;
    }

    if (candidate == LEVEL_L3 && primary_for_l3(s).empty()) {
      candidate = LEVEL_L2;
      reasons.push_back("l3_demoted_no_primary_front");
    }

    uint8_t level = candidate;
    if (candidate < last_level_) {
      if (candidate == LEVEL_L2 && score >= thr_l3_ - hysteresis_) {
        level = LEVEL_L3;
      } else if (candidate == LEVEL_L1 && score >= thr_l2_ - hysteresis_) {
        level = LEVEL_L2;
      } else if (candidate == LEVEL_L0 && score >= thr_l1_ - hysteresis_) {
        level = LEVEL_L1;
      }
    }
    last_level_ = level;

    FusionResult result;
    result.level = level;
    result.score = score;
    const auto l3_primary = primary_for_l3(s);
    result.primary_reason =
        !l3_primary.empty() ? l3_primary : (reasons.empty() ? "none" : reasons.front());
    result.reasons = std::move(reasons);
    result.allowed_actions = actions(level, s);
    result.sensor_health_summary = sensor_summary(s);
    return result;
  }

  std::string voice_text(const std::string& reason) const {
    if (reason == "front_ttc_low") {
      return "注意前方行人";
    }
    if (reason == "front_risk") {
      return "前方有目标";
    }
    if (reason == "rear_right_approaching") {
      return "右后方来车";
    }
    if (reason == "rear_left_approaching") {
      return "左后方来车";
    }
    if (reason == "driver_fatigue") {
      return "请保持注意力";
    }
    if (reason == "imu_lean_or_brake") {
      return "检测到侧倾，禁止自动减速";
    }
    if (reason == "mmwave_vital_abnormal") {
      return "建议靠边休息";
    }
    return "";
  }

  static bool contains(const std::vector<std::string>& values, const std::string& needle) {
    return std::find(values.begin(), values.end(), needle) != values.end();
  }

  void tick() {
    const Snapshot s = fresh_snapshot();
    const FusionResult result = decide(s);

    ev_ads_interfaces::msg::RiskState rs;
    rs.header.stamp = now();
    rs.header.frame_id = "vehicle";
    rs.level = result.level;
    rs.score = static_cast<float>(result.score);
    rs.primary_reason = result.primary_reason;
    rs.reasons = result.reasons;
    rs.allowed_actions = result.allowed_actions;
    rs.sensor_health_summary = result.sensor_health_summary;
    pub_risk_->publish(rs);

    if (result.level > last_voice_level_ && result.level >= LEVEL_L1) {
      ev_ads_interfaces::msg::WarningCommand wc;
      wc.header = rs.header;
      wc.level = result.level;
      wc.voice_text = voice_text(result.primary_reason);
      wc.vibration_pattern =
          result.level == LEVEL_L1 ? 1 : (result.level == LEVEL_L2 ? 2 : 3);
      wc.beep_freq_hz =
          result.level == LEVEL_L1 ? 0 : (result.level == LEVEL_L2 ? 880 : 1600);
      wc.duration_ms = result.level < LEVEL_L3 ? 600 : 2000;
      wc.screen_icon = result.primary_reason;
      pub_warn_->publish(wc);
    }
    last_voice_level_ = result.level;

    ev_ads_interfaces::msg::BrakeCommand bc;
    bc.header = rs.header;
    bc.source = result.primary_reason;
    uint8_t gates = 0;
    if (s.front_health == HEALTH_OK) {
      gates |= 0x01;
    }
    if (s.imu_health == HEALTH_OK) {
      gates |= 0x02;
    }
    if (enable_real_brake_) {
      gates |= 0x04;
    }
    bc.safety_gates_passed = gates;
    if (result.level == LEVEL_L3 && gates == 0x07 &&
        contains(result.allowed_actions, "brake_demand_request")) {
      bc.enable = true;
      bc.demand = 0.5f;
      bc.max_duration_ms = 1500;
    } else {
      bc.enable = false;
      bc.demand = 0.0f;
      bc.max_duration_ms = 0;
    }
    pub_brake_->publish(bc);
  }

  double publish_rate_hz_{20.0};
  bool enable_real_brake_{false};
  bool zero_stale_{true};
  double stale_reliability_{0.25};
  double weighted_gain_{0.72};
  double probabilistic_gain_{0.28};
  double synergy_gain_{0.12};
  double consensus_bonus_{0.08};
  double front_emergency_ttc_{0.8};
  double front_warning_ttc_{2.5};
  double front_l3_floor_{0.78};
  std::string profile_{"urban_day"};
  Weights weights_;
  double thr_l1_{0.30};
  double thr_l2_{0.55};
  double thr_l3_{0.75};
  double hysteresis_{0.05};
  double front_ttc_l3_{1.2};
  int front_stale_ms_{500};
  int front_disc_ms_{3000};
  int rear_stale_ms_{500};
  int rear_disc_ms_{3000};
  int driver_stale_ms_{500};
  int driver_disc_ms_{3000};
  int imu_stale_ms_{200};
  int imu_disc_ms_{1000};
  int mmwave_stale_ms_{500};
  int mmwave_disc_ms_{3000};

  Snapshot snap_;
  rclcpp::Time front_seen_{0, 0, RCL_ROS_TIME};
  rclcpp::Time rear_seen_{0, 0, RCL_ROS_TIME};
  rclcpp::Time driver_seen_{0, 0, RCL_ROS_TIME};
  rclcpp::Time imu_seen_{0, 0, RCL_ROS_TIME};
  rclcpp::Time mmwave_seen_{0, 0, RCL_ROS_TIME};
  uint8_t last_level_{LEVEL_L0};
  uint8_t last_voice_level_{LEVEL_L0};

  rclcpp::Publisher<ev_ads_interfaces::msg::RiskState>::SharedPtr pub_risk_;
  rclcpp::Publisher<ev_ads_interfaces::msg::WarningCommand>::SharedPtr pub_warn_;
  rclcpp::Publisher<ev_ads_interfaces::msg::BrakeCommand>::SharedPtr pub_brake_;
  rclcpp::Subscription<ev_ads_interfaces::msg::FrontRisk>::SharedPtr sub_front_;
  rclcpp::Subscription<ev_ads_interfaces::msg::BlindSpotState>::SharedPtr sub_rear_;
  rclcpp::Subscription<ev_ads_interfaces::msg::DriverState>::SharedPtr sub_driver_;
  rclcpp::Subscription<ev_ads_interfaces::msg::VehicleMotion>::SharedPtr sub_motion_;
  rclcpp::Subscription<ev_ads_interfaces::msg::MmWaveVital>::SharedPtr sub_mmwave_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // 命名空间 ev_ads_runtime_cpp

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ev_ads_runtime_cpp::FusionNodeCpp>());
  rclcpp::shutdown();
  return 0;
}
