#pragma once

#include <cstdint>

namespace ev_ads_runtime_cpp {

enum class Health : uint8_t {
  kOk = 0,
  kStale = 1,
  kError = 2,
  kDisconnected = 3,
};

enum class WarningLevel : uint8_t {
  kL0 = 0,
  kL1 = 1,
  kL2 = 2,
  kL3 = 3,
};

enum class FaceVisibility : uint8_t {
  kNo = 0,
  kYes = 1,
  kUnknown = 2,
};

enum class ObjectClass : uint8_t {
  kNone = 0,
  kPedestrian = 1,
  kVehicle = 2,
  kBicycle = 3,
  kRoadAnimal = 4,
  kRoadObstacle = 5,
  kPothole = 6,
};

enum class ZoneState : uint8_t {
  kClear = 0,
  kPresent = 1,
  kApproaching = 2,
};

enum class MotionFlag : uint32_t {
  kHardBrake = 0x01,
  kLean = 0x02,
  kBump = 0x04,
  kFall = 0x08,
};

enum class SensorHealthBit : uint8_t {
  kFront = 0x01,
  kRear = 0x02,
  kDriver = 0x04,
  kImu = 0x08,
  kMmWave = 0x10,
};

constexpr uint8_t to_ros(Health value) {
  return static_cast<uint8_t>(value);
}

constexpr uint8_t to_ros(WarningLevel value) {
  return static_cast<uint8_t>(value);
}

constexpr uint8_t to_ros(FaceVisibility value) {
  return static_cast<uint8_t>(value);
}

constexpr uint8_t to_ros(ObjectClass value) {
  return static_cast<uint8_t>(value);
}

constexpr uint8_t to_ros(ZoneState value) {
  return static_cast<uint8_t>(value);
}

constexpr uint32_t to_mask(MotionFlag value) {
  return static_cast<uint32_t>(value);
}

constexpr uint8_t to_mask(SensorHealthBit value) {
  return static_cast<uint8_t>(value);
}

inline Health health_from_ros(uint8_t value) {
  switch (value) {
    case 0:
      return Health::kOk;
    case 1:
      return Health::kStale;
    case 2:
      return Health::kError;
    case 3:
      return Health::kDisconnected;
    default:
      return Health::kError;
  }
}

inline WarningLevel level_from_ros(uint8_t value) {
  switch (value) {
    case 0:
      return WarningLevel::kL0;
    case 1:
      return WarningLevel::kL1;
    case 2:
      return WarningLevel::kL2;
    case 3:
      return WarningLevel::kL3;
    default:
      return WarningLevel::kL0;
  }
}

inline FaceVisibility face_from_ros(uint8_t value) {
  switch (value) {
    case 0:
      return FaceVisibility::kNo;
    case 1:
      return FaceVisibility::kYes;
    case 2:
      return FaceVisibility::kUnknown;
    default:
      return FaceVisibility::kUnknown;
  }
}

inline ObjectClass object_class_from_ros(uint8_t value) {
  switch (value) {
    case 0:
      return ObjectClass::kNone;
    case 1:
      return ObjectClass::kPedestrian;
    case 2:
      return ObjectClass::kVehicle;
    case 3:
      return ObjectClass::kBicycle;
    case 4:
      return ObjectClass::kRoadAnimal;
    case 5:
      return ObjectClass::kRoadObstacle;
    case 6:
      return ObjectClass::kPothole;
    default:
      return ObjectClass::kNone;
  }
}

inline ZoneState zone_from_ros(uint8_t value) {
  switch (value) {
    case 0:
      return ZoneState::kClear;
    case 1:
      return ZoneState::kPresent;
    case 2:
      return ZoneState::kApproaching;
    default:
      return ZoneState::kClear;
  }
}

// ROS 消息字段仍是 uint8/uint32。以下别名只做边界兼容，枚举定义以 enum class 为准。
inline constexpr uint8_t HEALTH_OK = to_ros(Health::kOk);
inline constexpr uint8_t HEALTH_STALE = to_ros(Health::kStale);
inline constexpr uint8_t HEALTH_ERROR = to_ros(Health::kError);
inline constexpr uint8_t HEALTH_DISCONNECTED = to_ros(Health::kDisconnected);

inline constexpr uint8_t LEVEL_L0 = to_ros(WarningLevel::kL0);
inline constexpr uint8_t LEVEL_L1 = to_ros(WarningLevel::kL1);
inline constexpr uint8_t LEVEL_L2 = to_ros(WarningLevel::kL2);
inline constexpr uint8_t LEVEL_L3 = to_ros(WarningLevel::kL3);

inline constexpr uint8_t FACE_NO = to_ros(FaceVisibility::kNo);
inline constexpr uint8_t FACE_YES = to_ros(FaceVisibility::kYes);
inline constexpr uint8_t FACE_UNKNOWN = to_ros(FaceVisibility::kUnknown);

inline constexpr uint8_t CLASS_NONE = to_ros(ObjectClass::kNone);
inline constexpr uint8_t CLASS_PEDESTRIAN = to_ros(ObjectClass::kPedestrian);
inline constexpr uint8_t CLASS_VEHICLE = to_ros(ObjectClass::kVehicle);
inline constexpr uint8_t CLASS_BICYCLE = to_ros(ObjectClass::kBicycle);
inline constexpr uint8_t CLASS_ROAD_ANIMAL = to_ros(ObjectClass::kRoadAnimal);
inline constexpr uint8_t CLASS_ROAD_OBSTACLE = to_ros(ObjectClass::kRoadObstacle);
inline constexpr uint8_t CLASS_POTHOLE = to_ros(ObjectClass::kPothole);

inline constexpr uint8_t ZONE_CLEAR = to_ros(ZoneState::kClear);
inline constexpr uint8_t ZONE_PRESENT = to_ros(ZoneState::kPresent);
inline constexpr uint8_t ZONE_APPROACHING = to_ros(ZoneState::kApproaching);

inline constexpr uint32_t MOTION_HARD_BRAKE = to_mask(MotionFlag::kHardBrake);
inline constexpr uint32_t MOTION_LEAN = to_mask(MotionFlag::kLean);
inline constexpr uint32_t MOTION_BUMP = to_mask(MotionFlag::kBump);
inline constexpr uint32_t MOTION_FALL = to_mask(MotionFlag::kFall);

}  // namespace ev_ads_runtime_cpp
