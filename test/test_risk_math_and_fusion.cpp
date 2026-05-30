#include <cmath>

#include <gtest/gtest.h>

#include "ev_ads_runtime_cpp/risk_math.hpp"
#include "ev_ads_runtime_cpp/risk_fusion_core.hpp"

using namespace ev_ads_runtime_cpp;

namespace {

TEST(DomainTypeConversion, KeepsRosBoundaryValuesStable) {
  EXPECT_EQ(to_ros(Health::kOk), 0);
  EXPECT_EQ(health_from_ros(3), Health::kDisconnected);
  EXPECT_EQ(health_from_ros(99), Health::kError);
  EXPECT_EQ(to_ros(WarningLevel::kL3), 3);
  EXPECT_EQ(object_class_from_ros(CLASS_POTHOLE), ObjectClass::kPothole);
  EXPECT_EQ(zone_from_ros(ZONE_APPROACHING), ZoneState::kApproaching);
}

TEST(RiskMath, EstimatesTtcAndScoresRisk) {
  EXPECT_TRUE(std::isinf(estimate_ttc(10.0, 0.0)));
  EXPECT_NEAR(estimate_ttc(10.0, 5.0), 2.0, 1e-9);
  EXPECT_GT(front_risk_score(ObjectClass::kPedestrian, 2.0, 2.0, 0.0), 0.7);
  EXPECT_DOUBLE_EQ(front_risk_score(ObjectClass::kNone, 2.0, 2.0, 0.0), 0.0);
  EXPECT_EQ(zone_state_enum(3.0, 4.0), ZoneState::kApproaching);
  EXPECT_GT(aggregate_rear_risk(ZoneState::kClear, ZoneState::kPresent, ZoneState::kApproaching), 0.7);
  EXPECT_DOUBLE_EQ(fatigue_score(FaceVisibility::kNo, 1.0, 1.0, 1.0, 1.0), 0.0);
  EXPECT_GT(fatigue_score(FaceVisibility::kYes, 0.9, 0.0, 0.0, 0.0), 0.5);
}

TEST(FusionCore, EscalatesFrontLowTtcToL3) {
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
  EXPECT_EQ(out.level, WarningLevel::kL3);
  EXPECT_EQ(out.primary_reason, "front_ttc_low");
  EXPECT_TRUE(contains_action(out.allowed_actions, "brake_demand_request"));
}

TEST(FusionCore, DemotesL3WithoutFrontPrimaryRisk) {
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
  EXPECT_NE(out.level, WarningLevel::kL3);
}

}  // namespace
