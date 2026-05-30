// 事件记录节点：订阅关键状态并写入 SQLite/WAL 或兼容 JSONL 后端。
#include <chrono>
#include <memory>
#include <string>

#include "ev_ads_runtime_cpp/msg/blind_spot_state.hpp"
#include "ev_ads_runtime_cpp/msg/brake_command.hpp"
#include "ev_ads_runtime_cpp/msg/driver_state.hpp"
#include "ev_ads_runtime_cpp/msg/front_risk.hpp"
#include "ev_ads_runtime_cpp/msg/mm_wave_vital.hpp"
#include "ev_ads_runtime_cpp/msg/risk_state.hpp"
#include "ev_ads_runtime_cpp/msg/vehicle_motion.hpp"
#include "ev_ads_runtime_cpp/msg/warning_command.hpp"
#include "ev_ads_runtime_cpp/risk_math.hpp"
#include "ev_ads_runtime_cpp/event_store.hpp"
#include "ev_ads_runtime_cpp/runtime_config.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ev_ads_runtime_cpp {
namespace {

EventLoggerConfig read_event_logger_config(rclcpp::Node* node) {
  EventLoggerConfig cfg;
  cfg.storage_backend = node->declare_parameter<std::string>("storage_backend", cfg.storage_backend);
  cfg.log_path = node->declare_parameter<std::string>("log_path", cfg.log_path);
  cfg.log_all_risk = node->declare_parameter<bool>("log_all_risk", cfg.log_all_risk);
  cfg.flush_every_n = node->declare_parameter<int>("flush_every_n", cfg.flush_every_n);
  return cfg;
}

std::string object_payload(const std::string& body) {
  return "{" + body + "}";
}

}  // namespace

class EventRecorderNode final : public rclcpp::Node {
 public:
  EventRecorderNode()
      : Node("event_recorder_node"),
        config_(read_event_logger_config(this)) {
    EventStoreConfig store_config;
    store_config.backend = config_.storage_backend;
    store_config.path = config_.log_path;
    store_config.flush_every_n = config_.flush_every_n;

    std::string error;
    if (!store_.open(store_config, &error)) {
      RCLCPP_ERROR(get_logger(), "事件存储打开失败: %s", error.c_str());
    } else {
      RCLCPP_INFO(
          get_logger(),
          "事件存储已打开，后端=%s 路径=%s 批量=%d",
          store_config.backend.c_str(),
          store_config.path.c_str(),
          store_config.flush_every_n);
    }

    risk_sub_ = create_subscription<ev_ads_runtime_cpp::msg::RiskState>(
        topics_.risk_state, rclcpp::QoS(10),
        [this](ev_ads_runtime_cpp::msg::RiskState::SharedPtr msg) {
          on_risk(*msg);
        });
    warning_sub_ = create_subscription<ev_ads_runtime_cpp::msg::WarningCommand>(
        topics_.warning_cmd, rclcpp::QoS(10),
        [this](ev_ads_runtime_cpp::msg::WarningCommand::SharedPtr msg) {
          write_event("warning", warning_payload(*msg));
        });
    brake_sub_ = create_subscription<ev_ads_runtime_cpp::msg::BrakeCommand>(
        topics_.brake_cmd, rclcpp::QoS(1),
        [this](ev_ads_runtime_cpp::msg::BrakeCommand::SharedPtr msg) {
          write_event("brake", brake_payload(*msg));
        });
    front_sub_ = create_subscription<ev_ads_runtime_cpp::msg::FrontRisk>(
        topics_.front_risk, rclcpp::QoS(10),
        [this](ev_ads_runtime_cpp::msg::FrontRisk::SharedPtr msg) {
          if (msg->health != HEALTH_OK || msg->risk_score >= 0.60f) {
            write_event("front", front_payload(*msg));
          }
        });
    rear_sub_ = create_subscription<ev_ads_runtime_cpp::msg::BlindSpotState>(
        topics_.blind_spot, rclcpp::QoS(10),
        [this](ev_ads_runtime_cpp::msg::BlindSpotState::SharedPtr msg) {
          if (msg->health != HEALTH_OK || msg->risk_score >= 0.60f) {
            write_event("rear", rear_payload(*msg));
          }
        });
    driver_sub_ = create_subscription<ev_ads_runtime_cpp::msg::DriverState>(
        topics_.driver_state, rclcpp::QoS(10),
        [this](ev_ads_runtime_cpp::msg::DriverState::SharedPtr msg) {
          if (msg->health != HEALTH_OK || msg->fatigue_score >= 0.60f ||
              msg->distraction_ratio >= 0.60f) {
            write_event("driver", driver_payload(*msg));
          }
        });
    motion_sub_ = create_subscription<ev_ads_runtime_cpp::msg::VehicleMotion>(
        topics_.vehicle_motion, rclcpp::QoS(50),
        [this](ev_ads_runtime_cpp::msg::VehicleMotion::SharedPtr msg) {
          if (msg->health != HEALTH_OK || msg->motion_flags != 0) {
            write_event("motion", motion_payload(*msg));
          }
        });
    mmwave_sub_ = create_subscription<ev_ads_runtime_cpp::msg::MmWaveVital>(
        topics_.mmwave_vital, rclcpp::QoS(10),
        [this](ev_ads_runtime_cpp::msg::MmWaveVital::SharedPtr msg) {
          if (msg->health != HEALTH_OK || msg->confidence < 0.30f) {
            write_event("mmwave", mmwave_payload(*msg));
          }
        });

    flush_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      std::string error;
      if (!store_.flush(&error)) {
        RCLCPP_WARN(get_logger(), "事件存储 flush 失败: %s", error.c_str());
      }
    });
  }

 private:
  void write_event(const std::string& type, const std::string& payload_json) {
    EventRecord record;
    record.timestamp_s = now().seconds();
    record.type = type;
    record.payload_json = payload_json;
    std::string error;
    if (!store_.write(record, &error)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "事件写入失败: %s", error.c_str());
    }
  }

  void on_risk(const ev_ads_runtime_cpp::msg::RiskState& msg) {
    const bool changed = !has_last_risk_ ||
        msg.level != last_level_ ||
        msg.primary_reason != last_reason_;
    if (config_.log_all_risk || changed) {
      write_event("risk", risk_payload(msg));
    }
    has_last_risk_ = true;
    last_level_ = msg.level;
    last_reason_ = msg.primary_reason;
  }

  static std::string risk_payload(const ev_ads_runtime_cpp::msg::RiskState& msg) {
    return object_payload(
        "\"level\":" + std::to_string(msg.level) + "," +
        "\"score\":" + number_json(msg.score) + "," +
        "\"primary_reason\":\"" + escape_json(msg.primary_reason) + "\"," +
        "\"reasons\":" + string_array_json(msg.reasons) + "," +
        "\"actions\":" + string_array_json(msg.allowed_actions) + "," +
        "\"sensor_health\":" + std::to_string(msg.sensor_health_summary));
  }

  static std::string warning_payload(const ev_ads_runtime_cpp::msg::WarningCommand& msg) {
    return object_payload(
        "\"level\":" + std::to_string(msg.level) + "," +
        "\"voice\":\"" + escape_json(msg.voice_text) + "\"," +
        "\"icon\":\"" + escape_json(msg.screen_icon) + "\"," +
        "\"vibration\":" + std::to_string(msg.vibration_pattern) + "," +
        "\"beep_hz\":" + std::to_string(msg.beep_freq_hz) + "," +
        "\"duration_ms\":" + std::to_string(msg.duration_ms));
  }

  static std::string brake_payload(const ev_ads_runtime_cpp::msg::BrakeCommand& msg) {
    return object_payload(
        "\"enable\":" + std::string(msg.enable ? "true" : "false") + "," +
        "\"demand\":" + number_json(msg.demand) + "," +
        "\"duration_ms\":" + std::to_string(msg.max_duration_ms) + "," +
        "\"gates\":" + std::to_string(msg.safety_gates_passed) + "," +
        "\"source\":\"" + escape_json(msg.source) + "\"");
  }

  static std::string front_payload(const ev_ads_runtime_cpp::msg::FrontRisk& msg) {
    return object_payload(
        "\"risk\":" + number_json(msg.risk_score) + "," +
        "\"ttc\":" + number_json(msg.ttc) + "," +
        "\"distance\":" + number_json(msg.distance) + "," +
        "\"class\":" + std::to_string(msg.primary_class) + "," +
        "\"health\":" + std::to_string(msg.health));
  }

  static std::string rear_payload(const ev_ads_runtime_cpp::msg::BlindSpotState& msg) {
    return object_payload(
        "\"risk\":" + number_json(msg.risk_score) + "," +
        "\"left\":" + std::to_string(msg.zone_left) + "," +
        "\"center\":" + std::to_string(msg.zone_center) + "," +
        "\"right\":" + std::to_string(msg.zone_right) + "," +
        "\"health\":" + std::to_string(msg.health));
  }

  static std::string driver_payload(const ev_ads_runtime_cpp::msg::DriverState& msg) {
    return object_payload(
        "\"fatigue\":" + number_json(msg.fatigue_score) + "," +
        "\"distraction\":" + number_json(msg.distraction_ratio) + "," +
        "\"eye\":" + number_json(msg.eye_closure_ratio) + "," +
        "\"face\":" + std::to_string(msg.face_visible) + "," +
        "\"health\":" + std::to_string(msg.health));
  }

  static std::string motion_payload(const ev_ads_runtime_cpp::msg::VehicleMotion& msg) {
    return object_payload(
        "\"flags\":" + std::to_string(msg.motion_flags) + "," +
        "\"roll\":" + number_json(msg.roll) + "," +
        "\"yaw_rate\":" + number_json(msg.yaw_rate) + "," +
        "\"health\":" + std::to_string(msg.health));
  }

  static std::string mmwave_payload(const ev_ads_runtime_cpp::msg::MmWaveVital& msg) {
    return object_payload(
        "\"hr\":" + number_json(msg.heart_rate) + "," +
        "\"br\":" + number_json(msg.breath_rate) + "," +
        "\"distance_cm\":" + number_json(msg.distance) + "," +
        "\"confidence\":" + number_json(msg.confidence) + "," +
        "\"health\":" + std::to_string(msg.health));
  }

  RuntimeTopics topics_;
  EventLoggerConfig config_;
  EventStore store_;
  bool has_last_risk_{false};
  uint8_t last_level_{0};
  std::string last_reason_;
  rclcpp::Subscription<ev_ads_runtime_cpp::msg::RiskState>::SharedPtr risk_sub_;
  rclcpp::Subscription<ev_ads_runtime_cpp::msg::WarningCommand>::SharedPtr warning_sub_;
  rclcpp::Subscription<ev_ads_runtime_cpp::msg::BrakeCommand>::SharedPtr brake_sub_;
  rclcpp::Subscription<ev_ads_runtime_cpp::msg::FrontRisk>::SharedPtr front_sub_;
  rclcpp::Subscription<ev_ads_runtime_cpp::msg::BlindSpotState>::SharedPtr rear_sub_;
  rclcpp::Subscription<ev_ads_runtime_cpp::msg::DriverState>::SharedPtr driver_sub_;
  rclcpp::Subscription<ev_ads_runtime_cpp::msg::VehicleMotion>::SharedPtr motion_sub_;
  rclcpp::Subscription<ev_ads_runtime_cpp::msg::MmWaveVital>::SharedPtr mmwave_sub_;
  rclcpp::TimerBase::SharedPtr flush_timer_;
};

}  // namespace ev_ads_runtime_cpp

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ev_ads_runtime_cpp::EventRecorderNode>());
  rclcpp::shutdown();
  return 0;
}
