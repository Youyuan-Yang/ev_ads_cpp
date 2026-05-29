#pragma once

#include <string>

namespace ev_ads_runtime_cpp {

struct RuntimeTopics {
  std::string camera_front_ns = "/camera/front";
  std::string camera_rear_ns = "/camera/rear";
  std::string camera_driver_ns = "/camera/driver";

  std::string sim_front_observation = "/sim/front_observation";
  std::string sim_rear_zones = "/sim/rear_zones";
  std::string sim_driver_observation = "/sim/driver_observation";

  std::string front_risk = "/perception/front_risk";
  std::string blind_spot = "/perception/blind_spot";
  std::string driver_state = "/perception/driver_state";
  std::string vehicle_motion = "/vehicle/motion";
  std::string mmwave_vital = "/sensor/mmwave/vital";

  std::string risk_state = "/decision/risk_state";
  std::string warning_cmd = "/decision/warning_cmd";
  std::string brake_cmd = "/decision/brake_cmd";

  static RuntimeTopics defaults() {
    return RuntimeTopics{};
  }

  static std::string image_topic(const std::string& camera_ns) {
    return camera_ns + "/image_raw/compressed";
  }

  static std::string health_topic(const std::string& camera_ns) {
    return camera_ns + "/health";
  }
};

}  // namespace ev_ads_runtime_cpp
