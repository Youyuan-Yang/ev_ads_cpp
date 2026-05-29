#include <filesystem>
#include <fstream>
#include <cmath>
#include <memory>
#include <sstream>
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
namespace {

std::string escape_json(const std::string& text) {
  std::ostringstream out;
  for (const char ch : text) {
    switch (ch) {
      case '\\':
        out << "\\\\";
        break;
      case '"':
        out << "\\\"";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << ch;
        break;
    }
  }
  return out.str();
}

std::string string_array_json(const std::vector<std::string>& items) {
  std::ostringstream out;
  out << "[";
  for (size_t i = 0; i < items.size(); ++i) {
    if (i != 0) {
      out << ",";
    }
    out << "\"" << escape_json(items[i]) << "\"";
  }
  out << "]";
  return out.str();
}

std::string number_json(double value) {
  if (!std::isfinite(value)) {
    return "null";
  }
  return std::to_string(value);
}

}  // 匿名命名空间

class EventLoggerNodeCpp final : public rclcpp::Node {
 public:
  EventLoggerNodeCpp() : Node("event_logger_node_cpp") {
    log_path_ = declare_parameter<std::string>("log_path", "/tmp/ev_ads/events.jsonl");
    log_all_risk_ = declare_parameter<bool>("log_all_risk", false);

    const std::filesystem::path path(log_path_);
    if (path.has_parent_path()) {
      std::filesystem::create_directories(path.parent_path());
    }
    file_.open(log_path_, std::ios::app);
    if (!file_.is_open()) {
      RCLCPP_ERROR(get_logger(), "事件日志打开失败: %s", log_path_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "事件日志写入: %s", log_path_.c_str());
    }

    risk_sub_ = create_subscription<ev_ads_interfaces::msg::RiskState>(
        "/decision/risk_state", rclcpp::QoS(10),
        [this](ev_ads_interfaces::msg::RiskState::SharedPtr msg) {
          on_risk(*msg);
        });
    warning_sub_ = create_subscription<ev_ads_interfaces::msg::WarningCommand>(
        "/decision/warning_cmd", rclcpp::QoS(10),
        [this](ev_ads_interfaces::msg::WarningCommand::SharedPtr msg) {
          on_warning(*msg);
        });
    brake_sub_ = create_subscription<ev_ads_interfaces::msg::BrakeCommand>(
        "/decision/brake_cmd", rclcpp::QoS(1),
        [this](ev_ads_interfaces::msg::BrakeCommand::SharedPtr msg) {
          on_brake(*msg);
        });
    front_sub_ = create_subscription<ev_ads_interfaces::msg::FrontRisk>(
        "/perception/front_risk", rclcpp::QoS(10),
        [this](ev_ads_interfaces::msg::FrontRisk::SharedPtr msg) {
          if (msg->health != HEALTH_OK || msg->risk_score >= 0.60f) {
            write_line("\"type\":\"front\","
                 "\"risk\":" + number_json(msg->risk_score) + "," +
                 "\"ttc\":" + number_json(msg->ttc) + "," +
                 "\"distance\":" + number_json(msg->distance) + "," +
                 "\"class\":" + std::to_string(msg->primary_class) + "," +
                 "\"health\":" + std::to_string(msg->health));
          }
        });
    rear_sub_ = create_subscription<ev_ads_interfaces::msg::BlindSpotState>(
        "/perception/blind_spot", rclcpp::QoS(10),
        [this](ev_ads_interfaces::msg::BlindSpotState::SharedPtr msg) {
          if (msg->health != HEALTH_OK || msg->risk_score >= 0.60f) {
            write_line("\"type\":\"rear\","
                 "\"risk\":" + number_json(msg->risk_score) + "," +
                 "\"left\":" + std::to_string(msg->zone_left) + "," +
                 "\"center\":" + std::to_string(msg->zone_center) + "," +
                 "\"right\":" + std::to_string(msg->zone_right) + "," +
                 "\"health\":" + std::to_string(msg->health));
          }
        });
    driver_sub_ = create_subscription<ev_ads_interfaces::msg::DriverState>(
        "/perception/driver_state", rclcpp::QoS(10),
        [this](ev_ads_interfaces::msg::DriverState::SharedPtr msg) {
          if (msg->health != HEALTH_OK || msg->fatigue_score >= 0.60f ||
              msg->distraction_ratio >= 0.60f) {
            write_line("\"type\":\"driver\","
                 "\"fatigue\":" + number_json(msg->fatigue_score) + "," +
                 "\"distraction\":" + number_json(msg->distraction_ratio) + "," +
                 "\"eye\":" + number_json(msg->eye_closure_ratio) + "," +
                 "\"health\":" + std::to_string(msg->health));
          }
        });
    motion_sub_ = create_subscription<ev_ads_interfaces::msg::VehicleMotion>(
        "/vehicle/motion", rclcpp::QoS(50),
        [this](ev_ads_interfaces::msg::VehicleMotion::SharedPtr msg) {
          if (msg->health != HEALTH_OK || msg->motion_flags != 0) {
            write_line("\"type\":\"motion\","
                 "\"flags\":" + std::to_string(msg->motion_flags) + "," +
                 "\"roll\":" + number_json(msg->roll) + "," +
                 "\"yaw_rate\":" + number_json(msg->yaw_rate) + "," +
                 "\"health\":" + std::to_string(msg->health));
          }
        });
    mmwave_sub_ = create_subscription<ev_ads_interfaces::msg::MmWaveVital>(
        "/sensor/mmwave/vital", rclcpp::QoS(10),
        [this](ev_ads_interfaces::msg::MmWaveVital::SharedPtr msg) {
          if (msg->health != HEALTH_OK || msg->confidence < 0.30f) {
            write_line("\"type\":\"mmwave\","
                 "\"hr\":" + number_json(msg->heart_rate) + "," +
                 "\"br\":" + number_json(msg->breath_rate) + "," +
                 "\"distance_cm\":" + number_json(msg->distance) + "," +
                 "\"confidence\":" + number_json(msg->confidence) + "," +
                 "\"health\":" + std::to_string(msg->health));
          }
        });
  }

 private:
  void write_line(const std::string& body) {
    if (!file_.is_open()) {
      return;
    }
    file_ << "{\"t\":" << now().seconds() << "," << body << "}\n";
    file_.flush();
  }

  void on_risk(const ev_ads_interfaces::msg::RiskState& msg) {
    const bool changed = !has_last_risk_ ||
        msg.level != last_level_ ||
        msg.primary_reason != last_reason_;
    if (log_all_risk_ || changed) {
      write_line("\"type\":\"risk\","
           "\"level\":" + std::to_string(msg.level) + "," +
           "\"score\":" + number_json(msg.score) + "," +
           "\"primary_reason\":\"" + escape_json(msg.primary_reason) + "\"," +
           "\"reasons\":" + string_array_json(msg.reasons) + "," +
           "\"actions\":" + string_array_json(msg.allowed_actions) + "," +
           "\"sensor_health\":" + std::to_string(msg.sensor_health_summary));
    }
    has_last_risk_ = true;
    last_level_ = msg.level;
    last_reason_ = msg.primary_reason;
  }

  void on_warning(const ev_ads_interfaces::msg::WarningCommand& msg) {
    write_line("\"type\":\"warning\","
         "\"level\":" + std::to_string(msg.level) + "," +
         "\"voice\":\"" + escape_json(msg.voice_text) + "\"," +
         "\"icon\":\"" + escape_json(msg.screen_icon) + "\"," +
         "\"vibration\":" + std::to_string(msg.vibration_pattern) + "," +
         "\"beep_hz\":" + std::to_string(msg.beep_freq_hz) + "," +
         "\"duration_ms\":" + std::to_string(msg.duration_ms));
  }

  void on_brake(const ev_ads_interfaces::msg::BrakeCommand& msg) {
    write_line("\"type\":\"brake\","
         "\"enable\":" + std::string(msg.enable ? "true" : "false") + "," +
         "\"demand\":" + number_json(msg.demand) + "," +
         "\"duration_ms\":" + std::to_string(msg.max_duration_ms) + "," +
         "\"gates\":" + std::to_string(msg.safety_gates_passed) + "," +
         "\"source\":\"" + escape_json(msg.source) + "\"");
  }

  std::string log_path_;
  bool log_all_risk_{false};
  bool has_last_risk_{false};
  uint8_t last_level_{0};
  std::string last_reason_;
  std::ofstream file_;
  rclcpp::Subscription<ev_ads_interfaces::msg::RiskState>::SharedPtr risk_sub_;
  rclcpp::Subscription<ev_ads_interfaces::msg::WarningCommand>::SharedPtr warning_sub_;
  rclcpp::Subscription<ev_ads_interfaces::msg::BrakeCommand>::SharedPtr brake_sub_;
  rclcpp::Subscription<ev_ads_interfaces::msg::FrontRisk>::SharedPtr front_sub_;
  rclcpp::Subscription<ev_ads_interfaces::msg::BlindSpotState>::SharedPtr rear_sub_;
  rclcpp::Subscription<ev_ads_interfaces::msg::DriverState>::SharedPtr driver_sub_;
  rclcpp::Subscription<ev_ads_interfaces::msg::VehicleMotion>::SharedPtr motion_sub_;
  rclcpp::Subscription<ev_ads_interfaces::msg::MmWaveVital>::SharedPtr mmwave_sub_;
};

}  // 命名空间 ev_ads_runtime_cpp

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ev_ads_runtime_cpp::EventLoggerNodeCpp>());
  rclcpp::shutdown();
  return 0;
}
