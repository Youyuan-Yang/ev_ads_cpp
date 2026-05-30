#pragma once

// 不依赖 ROS 的融合核心：便于在 Mac 上单测，也便于后续移植到 RKNN/NPU 链路外。

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "ev_ads_runtime_cpp/risk_math.hpp"
#include "ev_ads_runtime_cpp/runtime_config.hpp"

namespace ev_ads_runtime_cpp {

struct FusionSnapshot {
  double front = 0.0;
  double rear = 0.0;
  double driver = 0.0;
  double imu = 0.0;
  double mmwave = 0.0;

  Health front_health = Health::kDisconnected;
  Health rear_health = Health::kDisconnected;
  Health driver_health = Health::kDisconnected;
  Health imu_health = Health::kDisconnected;
  Health mmwave_health = Health::kDisconnected;

  double front_ttc = std::numeric_limits<double>::infinity();
  ObjectClass front_class = ObjectClass::kNone;
  ZoneState rear_zone_left = ZoneState::kClear;
  ZoneState rear_zone_right = ZoneState::kClear;
};

struct FusionDecision {
  WarningLevel level = WarningLevel::kL0;
  double score = 0.0;
  std::string primary_reason = "none";
  std::vector<std::string> reasons;
  std::vector<std::string> allowed_actions;
  uint8_t sensor_health_summary = 0;
};

class FusionCore {
 public:
  explicit FusionCore(FusionConfig config = FusionConfig{}) : config_(std::move(config)) {}

  const FusionConfig& config() const { return config_; }
  void set_config(const FusionConfig& config) { config_ = config; }
  void reset_hysteresis() { last_level_ = WarningLevel::kL0; }

  FusionDecision decide(const FusionSnapshot& s) {
    std::vector<std::string> degraded;
    const FusionWeights w = adaptive_weights(s, &degraded);
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
        {s.front_ttc < config_.front_ttc_l3 ? "front_ttc_low" : "front_risk", w.front * f},
        {s.rear_zone_right == ZoneState::kApproaching
             ? "rear_right_approaching"
             : (s.rear_zone_left == ZoneState::kApproaching ? "rear_left_approaching" : "rear_risk"),
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
    double score = config_.weighted_gain * weighted_score +
        config_.probabilistic_gain * probabilistic_score + synergy;

    const double urgency = ttc_urgency(s.front_ttc);
    if (s.front_health == Health::kOk && s.front > 0.5 && urgency > 0.0) {
      if (s.front_ttc <= config_.front_emergency_ttc) {
        score = std::max(score, config_.front_l3_floor);
      } else if (s.front_ttc <= config_.front_ttc_l3) {
        score = std::max(score, 0.70 + 0.08 * s.front);
      } else {
        score = std::max(score, 0.45 + 0.25 * urgency * s.front);
      }
    }
    score = clamp(score, 0.0, 1.0);

    reasons.insert(reasons.end(), synergy_reasons.begin(), synergy_reasons.end());
    reasons.insert(reasons.end(), degraded.begin(), degraded.end());
    reasons = unique_reasons(reasons);

    WarningLevel candidate = WarningLevel::kL0;
    if (score >= config_.thr_l3) {
      candidate = WarningLevel::kL3;
    } else if (score >= config_.thr_l2) {
      candidate = WarningLevel::kL2;
    } else if (score >= config_.thr_l1) {
      candidate = WarningLevel::kL1;
    }

    if (candidate == WarningLevel::kL3 && primary_for_l3(s).empty()) {
      candidate = WarningLevel::kL2;
      reasons.push_back("l3_demoted_no_primary_front");
    }

    WarningLevel level = candidate;
    if (to_ros(candidate) < to_ros(last_level_)) {
      if (candidate == WarningLevel::kL2 && score >= config_.thr_l3 - config_.hysteresis) {
        level = WarningLevel::kL3;
      } else if (candidate == WarningLevel::kL1 && score >= config_.thr_l2 - config_.hysteresis) {
        level = WarningLevel::kL2;
      } else if (candidate == WarningLevel::kL0 && score >= config_.thr_l1 - config_.hysteresis) {
        level = WarningLevel::kL1;
      }
    }
    last_level_ = level;

    FusionDecision result;
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

 private:
  double health_reliability(Health health) const {
    if (health == Health::kOk) {
      return 1.0;
    }
    if (health == Health::kStale) {
      return config_.zero_stale ? 0.0 : clamp(config_.stale_reliability, 0.0, 1.0);
    }
    return 0.0;
  }

  bool unavailable(Health health) const {
    return health_reliability(health) <= 0.0;
  }

  FusionWeights adaptive_weights(const FusionSnapshot& s, std::vector<std::string>* degraded) const {
    FusionWeights w = config_.weights;
    const double rf = health_reliability(s.front_health);
    const double rr = health_reliability(s.rear_health);
    const double rd = health_reliability(s.driver_health);
    const double ri = health_reliability(s.imu_health);
    const double rm = health_reliability(s.mmwave_health);

    apply_sensor_weight("front_cam_down", rf, &w.front, degraded);
    apply_sensor_weight("rear_cam_down", rr, &w.rear, degraded);
    apply_sensor_weight("driver_cam_down", rd, &w.driver, degraded);
    apply_sensor_weight("imu_down", ri, &w.imu, degraded);
    apply_sensor_weight("mmwave_down", rm, &w.mmwave, degraded);

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

  static void apply_sensor_weight(
      const char* degraded_reason,
      double reliability,
      double* weight,
      std::vector<std::string>* degraded) {
    if (reliability <= 0.0) {
      *weight = 0.0;
      degraded->push_back(degraded_reason);
      return;
    }
    *weight *= reliability;
  }

  double ttc_urgency(double ttc) const {
    if (!std::isfinite(ttc) || ttc <= 0.0) {
      return 0.0;
    }
    if (ttc <= config_.front_emergency_ttc) {
      return 1.0;
    }
    if (ttc <= config_.front_ttc_l3) {
      const double span = std::max(1e-6, config_.front_ttc_l3 - config_.front_emergency_ttc);
      return 0.85 + 0.15 * (config_.front_ttc_l3 - ttc) / span;
    }
    if (ttc <= config_.front_warning_ttc) {
      const double span = std::max(1e-6, config_.front_warning_ttc - config_.front_ttc_l3);
      return 0.25 + 0.55 * (config_.front_warning_ttc - ttc) / span;
    }
    return 0.0;
  }

  double probabilistic_evidence(const FusionSnapshot& s) const {
    const double rf = health_reliability(s.front_health);
    const double rr = health_reliability(s.rear_health);
    const double rd = health_reliability(s.driver_health);
    const double ri = health_reliability(s.imu_health);
    const double rm = health_reliability(s.mmwave_health);
    const std::vector<double> evidences = {
        clamp(s.front * rf * 0.55, 0.0, 0.95),
        clamp(s.rear * rr * 0.40, 0.0, 0.85),
        clamp(s.driver * rd * 0.30, 0.0, 0.75),
        clamp(s.imu * ri * 0.25, 0.0, 0.70),
        clamp(s.mmwave * rm * 0.12, 0.0, 0.20),
    };
    double p_no_risk = 1.0;
    for (double evidence : evidences) {
      p_no_risk *= 1.0 - evidence;
    }
    return 1.0 - p_no_risk;
  }

  double synergy_bonus(const FusionSnapshot& s, std::vector<std::string>* reasons) const {
    const double rf = health_reliability(s.front_health);
    const double rr = health_reliability(s.rear_health);
    const double rd = health_reliability(s.driver_health);
    const double ri = health_reliability(s.imu_health);
    const double rm = health_reliability(s.mmwave_health);
    double bonus = 0.0;

    const double front_driver = std::min(s.front * rf, s.driver * rd);
    if (front_driver > 0.35) {
      bonus += config_.synergy_gain * front_driver;
      reasons->push_back("synergy_front_driver");
    }
    const double front_imu = std::min(s.front * rf, s.imu * ri);
    if (front_imu > 0.35) {
      bonus += config_.synergy_gain * 0.8 * front_imu;
      reasons->push_back("synergy_front_imu");
    }
    const double rear_imu = std::min(s.rear * rr, s.imu * ri);
    if (rear_imu > 0.35) {
      bonus += config_.synergy_gain * 0.6 * rear_imu;
      reasons->push_back("synergy_rear_imu");
    }
    const double driver_mmwave = std::min(s.driver * rd, s.mmwave * rm);
    if (driver_mmwave > 0.35) {
      bonus += config_.synergy_gain * 0.45 * driver_mmwave;
      reasons->push_back("synergy_driver_vital");
    }

    const int active =
        (s.front * rf > 0.45 ? 1 : 0) +
        (s.rear * rr > 0.45 ? 1 : 0) +
        (s.driver * rd > 0.45 ? 1 : 0) +
        (s.imu * ri > 0.45 ? 1 : 0) +
        (s.mmwave * rm > 0.45 ? 1 : 0);
    if (active >= 3) {
      bonus += config_.consensus_bonus;
      reasons->push_back("multi_modal_consensus");
    }
    return clamp(bonus, 0.0, 0.22);
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

  std::string primary_for_l3(const FusionSnapshot& s) const {
    if (s.front_health == Health::kOk && s.front_ttc < config_.front_ttc_l3 && s.front > 0.5) {
      return "front_ttc_low";
    }
    return "";
  }

  static uint8_t sensor_summary(const FusionSnapshot& s) {
    uint8_t bits = 0;
    if (s.front_health == Health::kOk) {
      bits |= to_mask(SensorHealthBit::kFront);
    }
    if (s.rear_health == Health::kOk) {
      bits |= to_mask(SensorHealthBit::kRear);
    }
    if (s.driver_health == Health::kOk) {
      bits |= to_mask(SensorHealthBit::kDriver);
    }
    if (s.imu_health == Health::kOk) {
      bits |= to_mask(SensorHealthBit::kImu);
    }
    if (s.mmwave_health == Health::kOk) {
      bits |= to_mask(SensorHealthBit::kMmWave);
    }
    return bits;
  }

  static std::vector<std::string> actions(WarningLevel level, const FusionSnapshot& s) {
    std::vector<std::string> out;
    if (to_ros(level) >= to_ros(WarningLevel::kL1)) {
      out.push_back("screen_hint");
    }
    if (to_ros(level) >= to_ros(WarningLevel::kL2)) {
      out.push_back("warn_voice");
      out.push_back("warn_vibration");
    }
    if (level == WarningLevel::kL3) {
      if (s.front_health == Health::kOk && s.imu_health == Health::kOk) {
        out.push_back("brake_demand_request");
      } else {
        out.push_back("brake_blocked_safety_gate");
      }
    }
    return out;
  }

  FusionConfig config_;
  WarningLevel last_level_{WarningLevel::kL0};
};

inline std::string voice_text_for_reason(const std::string& reason) {
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

inline bool contains_action(const std::vector<std::string>& values, const std::string& needle) {
  return std::find(values.begin(), values.end(), needle) != values.end();
}

}  // namespace ev_ads_runtime_cpp
