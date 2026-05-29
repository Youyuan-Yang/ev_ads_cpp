#pragma once

#include <string>
#include <vector>

#include "ev_ads_runtime_cpp/topics.hpp"

namespace ev_ads_runtime_cpp {

struct ModelRuntimeConfig {
  std::string path;
  int input_width = 640;
  int input_height = 640;
  double confidence_threshold = 0.35;
  double nms_threshold = 0.45;
  bool has_objectness = false;
  std::vector<int> class_ids;
};

struct HealthGateConfig {
  bool require_camera_health = false;
  int camera_timeout_ms = 1000;
  int model_timeout_ms = 500;
};

struct RearPerceptionConfig {
  double publish_rate_hz = 10.0;
  std::string mode = "scripted";
  HealthGateConfig health;
  ModelRuntimeConfig model;
  double distance_focal_px = 700.0;
  double present_max_m = 10.0;
  double approaching_speed_mps = 1.0;
  bool fisheye_undistort = false;
  std::vector<double> fisheye_k;
  std::vector<double> fisheye_d;
  double fisheye_balance = 0.0;
  double fisheye_fov_scale = 1.0;
};

struct DriverMonitorConfig {
  double publish_rate_hz = 10.0;
  std::string mode = "scripted";
  HealthGateConfig health;
  ModelRuntimeConfig dms_model;
  std::string face_model_path;
  int face_input_width = 320;
  int face_input_height = 320;
  double face_score_threshold = 0.60;
  double face_nms_threshold = 0.30;
  int face_top_k = 5000;
  int face_absence_warning_ms = 1500;
  double face_absence_score = 0.65;
  std::vector<int> face_class_ids;
  std::vector<int> open_eye_class_ids{0};
  std::vector<int> half_eye_class_ids{1};
  std::vector<int> closed_eye_class_ids{2};
  std::vector<int> yawn_class_ids{3};
  std::vector<int> phone_class_ids{5};
  std::vector<int> distracted_class_ids{6};
  std::vector<int> fatigue_class_ids;
};

struct FusionWeights {
  double front = 0.35;
  double rear = 0.25;
  double driver = 0.20;
  double imu = 0.15;
  double mmwave = 0.05;
};

struct FusionTimeouts {
  int front_stale_ms = 500;
  int front_disc_ms = 3000;
  int rear_stale_ms = 500;
  int rear_disc_ms = 3000;
  int driver_stale_ms = 500;
  int driver_disc_ms = 3000;
  int imu_stale_ms = 200;
  int imu_disc_ms = 1000;
  int mmwave_stale_ms = 500;
  int mmwave_disc_ms = 3000;
};

struct FusionConfig {
  double publish_rate_hz = 20.0;
  bool enable_real_brake = false;
  bool zero_stale = true;
  std::string profile = "urban_day";
  FusionWeights weights;
  FusionTimeouts timeouts;
  double thr_l1 = 0.30;
  double thr_l2 = 0.55;
  double thr_l3 = 0.75;
  double hysteresis = 0.05;
  double front_ttc_l3 = 1.2;
  double stale_reliability = 0.25;
  double weighted_gain = 0.72;
  double probabilistic_gain = 0.28;
  double synergy_gain = 0.12;
  double consensus_bonus = 0.08;
  double front_emergency_ttc = 0.8;
  double front_warning_ttc = 2.5;
  double front_l3_floor = 0.78;
};

struct EventLoggerConfig {
  std::string storage_backend = "sqlite";
  std::string log_path = "/tmp/ev_ads/events.sqlite";
  bool log_all_risk = false;
  int flush_every_n = 32;
};

struct RuntimeConfig {
  RuntimeTopics topics = RuntimeTopics::defaults();
  RearPerceptionConfig rear;
  DriverMonitorConfig driver;
  FusionConfig fusion;
  EventLoggerConfig event_logger;
};

}  // namespace ev_ads_runtime_cpp
