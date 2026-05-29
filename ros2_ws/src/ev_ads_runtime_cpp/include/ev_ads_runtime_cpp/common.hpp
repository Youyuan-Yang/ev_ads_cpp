#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ev_ads_runtime_cpp {

constexpr uint8_t HEALTH_OK = 0;
constexpr uint8_t HEALTH_STALE = 1;
constexpr uint8_t HEALTH_ERROR = 2;
constexpr uint8_t HEALTH_DISCONNECTED = 3;

constexpr uint8_t LEVEL_L0 = 0;
constexpr uint8_t LEVEL_L1 = 1;
constexpr uint8_t LEVEL_L2 = 2;
constexpr uint8_t LEVEL_L3 = 3;

constexpr uint8_t FACE_NO = 0;
constexpr uint8_t FACE_YES = 1;
constexpr uint8_t FACE_UNKNOWN = 2;

constexpr uint8_t CLASS_NONE = 0;
constexpr uint8_t CLASS_PEDESTRIAN = 1;
constexpr uint8_t CLASS_VEHICLE = 2;
constexpr uint8_t CLASS_BICYCLE = 3;
constexpr uint8_t CLASS_ANIMAL = 4;
constexpr uint8_t CLASS_ROAD_OBSTACLE = 5;
constexpr uint8_t CLASS_POTHOLE = 6;

constexpr uint8_t ZONE_CLEAR = 0;
constexpr uint8_t ZONE_PRESENT = 1;
constexpr uint8_t ZONE_APPROACHING = 2;

constexpr uint32_t MOTION_HARD_BRAKE = 0x01;
constexpr uint32_t MOTION_LEAN = 0x02;
constexpr uint32_t MOTION_BUMP = 0x04;
constexpr uint32_t MOTION_FALL = 0x08;

template <typename T>
T clamp(T v, T lo, T hi) {
  return std::max(lo, std::min(v, hi));
}

inline bool bad_health(uint8_t health, bool zero_stale = true) {
  if (health == HEALTH_ERROR || health == HEALTH_DISCONNECTED) {
    return true;
  }
  return zero_stale && health == HEALTH_STALE;
}

inline double class_weight(uint8_t cls) {
  switch (cls) {
    case CLASS_PEDESTRIAN:
      return 1.0;
    case CLASS_VEHICLE:
    case CLASS_BICYCLE:
      return 0.85;
    case CLASS_ANIMAL:
      return 0.70;
    case CLASS_ROAD_OBSTACLE:
      return 0.55;
    case CLASS_POTHOLE:
      return 0.35;
    default:
      return 0.0;
  }
}

inline double estimate_ttc(double distance_m, double closing_speed_mps) {
  if (closing_speed_mps <= 0.01 || distance_m <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return distance_m / closing_speed_mps;
}

inline double front_risk_score(
    uint8_t primary_class,
    double distance_m,
    double closing_speed_mps,
    double lateral_offset_m) {
  if (primary_class == CLASS_NONE) {
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

inline uint8_t zone_state(
    double distance_m,
    double closing_speed_mps,
    double approaching_speed = 3.0,
    double present_max_m = 8.0) {
  if (distance_m < 0.0 || distance_m > present_max_m) {
    return ZONE_CLEAR;
  }
  if (closing_speed_mps >= approaching_speed) {
    return ZONE_APPROACHING;
  }
  return ZONE_PRESENT;
}

inline double aggregate_rear_risk(uint8_t left, uint8_t center, uint8_t right) {
  auto w = [](uint8_t state) {
    switch (state) {
      case ZONE_PRESENT:
        return 0.2;
      case ZONE_APPROACHING:
        return 0.7;
      default:
        return 0.0;
    }
  };
  return clamp(std::max(w(left), w(right)) + 0.5 * w(center), 0.0, 1.0);
}

inline double fatigue_score(
    uint8_t face_visible,
    double eye_closure_ratio,
    double head_pitch_rad,
    double head_yaw_rad,
    double distraction_ratio) {
  if (face_visible != FACE_YES) {
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
