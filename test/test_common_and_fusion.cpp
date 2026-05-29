#include <cassert>
#include <cmath>
#include <iostream>

#include "ev_ads_runtime_cpp/common.hpp"
#include "ev_ads_runtime_cpp/fusion_core.hpp"

using namespace ev_ads_runtime_cpp;

namespace {

void test_enum_boundaries() {
  assert(to_ros(Health::kOk) == 0);
  assert(health_from_ros(3) == Health::kDisconnected);
  assert(health_from_ros(99) == Health::kError);
  assert(to_ros(WarningLevel::kL3) == 3);
  assert(object_class_from_ros(CLASS_POTHOLE) == ObjectClass::kPothole);
  assert(zone_from_ros(ZONE_APPROACHING) == ZoneState::kApproaching);
}

void test_common_math() {
  assert(std::isinf(estimate_ttc(10.0, 0.0)));
  assert(std::abs(estimate_ttc(10.0, 5.0) - 2.0) < 1e-9);
  assert(front_risk_score(ObjectClass::kPedestrian, 2.0, 2.0, 0.0) > 0.7);
  assert(front_risk_score(ObjectClass::kNone, 2.0, 2.0, 0.0) == 0.0);
  assert(zone_state_enum(3.0, 4.0) == ZoneState::kApproaching);
  assert(aggregate_rear_risk(ZoneState::kClear, ZoneState::kPresent, ZoneState::kApproaching) > 0.7);
  assert(fatigue_score(FaceVisibility::kNo, 1.0, 1.0, 1.0, 1.0) == 0.0);
  assert(fatigue_score(FaceVisibility::kYes, 0.9, 0.0, 0.0, 0.0) > 0.5);
}

void test_fusion_front_l3() {
  FusionConfig config;
  FusionCore core(config);
  FusionSnapshot s;
  s.front = 0.9;
  s.front_health = Health::kOk;
  s.front_ttc = 0.7;
  s.imu = 0.1;
  s.imu_health = Health::kOk;
  s.rear_health = Health::kDisconnected;
  s.driver_health = Health::kDisconnected;
  s.mmwave_health = Health::kDisconnected;

  const FusionDecision out = core.decide(s);
  assert(out.level == WarningLevel::kL3);
  assert(out.primary_reason == "front_ttc_low");
  assert(contains_action(out.allowed_actions, "brake_demand_request"));
}

void test_fusion_demotes_without_front_primary() {
  FusionConfig config;
  FusionCore core(config);
  FusionSnapshot s;
  s.driver = 1.0;
  s.driver_health = Health::kOk;
  s.imu = 1.0;
  s.imu_health = Health::kOk;
  s.front_health = Health::kOk;
  s.front_ttc = 5.0;

  const FusionDecision out = core.decide(s);
  assert(out.level != WarningLevel::kL3);
}

}  // namespace

int main() {
  test_enum_boundaries();
  test_common_math();
  test_fusion_front_l3();
  test_fusion_demotes_without_front_primary();
  std::cout << "test_common_and_fusion ok\n";
  return 0;
}
