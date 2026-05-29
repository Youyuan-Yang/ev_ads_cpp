#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "ev_ads_runtime_cpp/types.hpp"

namespace ev_ads_runtime_cpp {

inline constexpr double kPi = 3.14159265358979323846;

#ifndef M_PI
#define M_PI ev_ads_runtime_cpp::kPi
#endif

template <typename T>
T clamp(T v, T lo, T hi) {
  return std::max(lo, std::min(v, hi));
}

inline bool bad_health(Health health, bool zero_stale = true) {
  if (health == Health::kError || health == Health::kDisconnected) {
    return true;
  }
  return zero_stale && health == Health::kStale;
}

inline bool bad_health(uint8_t health, bool zero_stale = true) {
  return bad_health(health_from_ros(health), zero_stale);
}

inline double class_weight(ObjectClass cls) {
  switch (cls) {
    case ObjectClass::kPedestrian:
      return 1.0;
    case ObjectClass::kVehicle:
    case ObjectClass::kBicycle:
      return 0.85;
    case ObjectClass::kRoadAnimal:
      return 0.70;
    case ObjectClass::kRoadObstacle:
      return 0.55;
    case ObjectClass::kPothole:
      return 0.35;
    default:
      return 0.0;
  }
}

inline double class_weight(uint8_t cls) {
  return class_weight(object_class_from_ros(cls));
}

inline double estimate_ttc(double distance_m, double closing_speed_mps) {
  if (closing_speed_mps <= 0.01 || distance_m <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return distance_m / closing_speed_mps;
}

inline double front_risk_score(
    ObjectClass primary_class,
    double distance_m,
    double closing_speed_mps,
    double lateral_offset_m) {
  if (primary_class == ObjectClass::kNone) {
    return 0.0;
  }
  const double ttc = estimate_ttc(distance_m, closing_speed_mps);
  double base = 0.0;
  if (std::isfinite(ttc)) {
    base = clamp(1.0 - (ttc - 0.5) / 4.5, 0.0, 1.0);
  } else {
    if (distance_m < 5.0) {
      base = 0.2;
    } else if (distance_m < 15.0) {
      base = 0.05;
    }
  }
  double lateral_penalty = 1.0;
  const double abs_lat = std::abs(lateral_offset_m);
  if (abs_lat > 1.0) {
    lateral_penalty = std::max(0.4, 1.0 - (abs_lat - 1.0) * 0.3);
  }
  return clamp(base * class_weight(primary_class) * lateral_penalty, 0.0, 1.0);
}

inline double front_risk_score(
    uint8_t primary_class,
    double distance_m,
    double closing_speed_mps,
    double lateral_offset_m) {
  return front_risk_score(
      object_class_from_ros(primary_class),
      distance_m,
      closing_speed_mps,
      lateral_offset_m);
}

inline ZoneState zone_state_enum(
    double distance_m,
    double closing_speed_mps,
    double approaching_speed = 3.0,
    double present_max_m = 8.0) {
  if (distance_m < 0.0 || distance_m > present_max_m) {
    return ZoneState::kClear;
  }
  if (closing_speed_mps >= approaching_speed) {
    return ZoneState::kApproaching;
  }
  return ZoneState::kPresent;
}

inline uint8_t zone_state(
    double distance_m,
    double closing_speed_mps,
    double approaching_speed = 3.0,
    double present_max_m = 8.0) {
  return to_ros(zone_state_enum(distance_m, closing_speed_mps, approaching_speed, present_max_m));
}

inline double aggregate_rear_risk(ZoneState left, ZoneState center, ZoneState right) {
  auto w = [](ZoneState state) {
    switch (state) {
      case ZoneState::kPresent:
        return 0.2;
      case ZoneState::kApproaching:
        return 0.7;
      default:
        return 0.0;
    }
  };
  return clamp(std::max(w(left), w(right)) + 0.5 * w(center), 0.0, 1.0);
}

inline double aggregate_rear_risk(uint8_t left, uint8_t center, uint8_t right) {
  return aggregate_rear_risk(zone_from_ros(left), zone_from_ros(center), zone_from_ros(right));
}

inline double fatigue_score(
    FaceVisibility face_visible,
    double eye_closure_ratio,
    double head_pitch_rad,
    double head_yaw_rad,
    double distraction_ratio) {
  if (face_visible != FaceVisibility::kYes) {
    return 0.0;
  }
  const double eye = clamp((eye_closure_ratio - 0.2) / 0.4, 0.0, 1.0);
  const double pitch =
      clamp((std::abs(head_pitch_rad) - M_PI / 12.0) / (5.0 * M_PI / 36.0), 0.0, 1.0);
  const double yaw =
      clamp((std::abs(head_yaw_rad) - M_PI / 9.0) / (M_PI / 6.0), 0.0, 1.0);
  const double distraction = clamp(distraction_ratio, 0.0, 1.0);
  return clamp(0.55 * eye + 0.25 * pitch + 0.10 * yaw + 0.10 * distraction, 0.0, 1.0);
}

inline double fatigue_score(
    uint8_t face_visible,
    double eye_closure_ratio,
    double head_pitch_rad,
    double head_yaw_rad,
    double distraction_ratio) {
  return fatigue_score(
      face_from_ros(face_visible),
      eye_closure_ratio,
      head_pitch_rad,
      head_yaw_rad,
      distraction_ratio);
}

inline double imu_score(uint32_t motion_flags) {
  double s = 0.0;
  if (motion_flags & MOTION_HARD_BRAKE) {
    s += 0.4;
  }
  if (motion_flags & MOTION_LEAN) {
    s += 0.4;
  }
  if (motion_flags & MOTION_BUMP) {
    s += 0.2;
  }
  if (motion_flags & MOTION_FALL) {
    s += 1.0;
  }
  return clamp(s, 0.0, 1.0);
}

inline double mmwave_score(double heart_rate, double breath_rate, double confidence) {
  if (confidence < 0.3) {
    return 0.0;
  }
  if (heart_rate > 0.0 && (heart_rate < 40.0 || heart_rate > 140.0)) {
    return clamp(std::abs(heart_rate - 80.0) / 80.0, 0.0, 1.0);
  }
  if (breath_rate > 0.0 && (breath_rate < 6.0 || breath_rate > 35.0)) {
    return 0.4;
  }
  return 0.0;
}

}  // 命名空间 ev_ads_runtime_cpp
