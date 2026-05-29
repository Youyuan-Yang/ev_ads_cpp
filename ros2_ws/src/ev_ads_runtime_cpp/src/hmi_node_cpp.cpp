#include <algorithm>
#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "ev_ads_interfaces/msg/blind_spot_state.hpp"
#include "ev_ads_interfaces/msg/brake_command.hpp"
#include "ev_ads_interfaces/msg/driver_state.hpp"
#include "ev_ads_interfaces/msg/front_risk.hpp"
#include "ev_ads_interfaces/msg/mm_wave_vital.hpp"
#include "ev_ads_interfaces/msg/risk_state.hpp"
#include "ev_ads_interfaces/msg/vehicle_motion.hpp"
#include "ev_ads_interfaces/msg/warning_command.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ev_ads_runtime_cpp {
namespace {

const char* health_text(uint8_t health) {
  switch (health) {
    case 0:
      return "OK";
    case 1:
      return "STALE";
    case 2:
      return "ERROR";
    case 3:
      return "DISCONNECTED";
    default:
      return "UNKNOWN";
  }
}

const char* level_color(uint8_t level) {
  switch (level) {
    case 0:
      return "\033[92m";
    case 1:
      return "\033[93m";
    case 2:
      return "\033[33m";
    case 3:
      return "\033[91m";
    default:
      return "";
  }
}

}  // 匿名命名空间

class HmiNodeCpp final : public rclcpp::Node {
 public:
  HmiNodeCpp() : Node("hmi_node_cpp") {
    const double rate = declare_parameter<double>("dashboard_rate_hz", 2.0);
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
          brake_ = *msg;
          has_brake_ = true;
          if (msg->enable) {
            RCLCPP_ERROR(
                get_logger(),
                "制动请求: demand=%.2f duration=%ums source=%s gates=%u",
                msg->demand,
                msg->max_duration_ms,
                msg->source.c_str(),
                msg->safety_gates_passed);
          }
        });
    front_sub_ = create_subscription<ev_ads_interfaces::msg::FrontRisk>(
        "/perception/front_risk", rclcpp::QoS(10),
        [this](ev_ads_interfaces::msg::FrontRisk::SharedPtr msg) {
          front_ = *msg;
          has_front_ = true;
        });
    rear_sub_ = create_subscription<ev_ads_interfaces::msg::BlindSpotState>(
        "/perception/blind_spot", rclcpp::QoS(10),
        [this](ev_ads_interfaces::msg::BlindSpotState::SharedPtr msg) {
          rear_ = *msg;
          has_rear_ = true;
        });
    driver_sub_ = create_subscription<ev_ads_interfaces::msg::DriverState>(
        "/perception/driver_state", rclcpp::QoS(10),
        [this](ev_ads_interfaces::msg::DriverState::SharedPtr msg) {
          driver_ = *msg;
          has_driver_ = true;
        });
    motion_sub_ = create_subscription<ev_ads_interfaces::msg::VehicleMotion>(
        "/vehicle/motion", rclcpp::QoS(50),
        [this](ev_ads_interfaces::msg::VehicleMotion::SharedPtr msg) {
          motion_ = *msg;
          has_motion_ = true;
        });
    mmwave_sub_ = create_subscription<ev_ads_interfaces::msg::MmWaveVital>(
        "/sensor/mmwave/vital", rclcpp::QoS(10),
        [this](ev_ads_interfaces::msg::MmWaveVital::SharedPtr msg) {
          mmwave_ = *msg;
          has_mmwave_ = true;
        });
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(0.5, rate)),
        std::bind(&HmiNodeCpp::dashboard, this));
    RCLCPP_INFO(get_logger(), "C++ 终端 HMI 已启动");
  }

 private:
  void on_risk(const ev_ads_interfaces::msg::RiskState& msg) {
    risk_ = msg;
    has_risk_ = true;
    if (last_level_ < 0 || msg.level != static_cast<uint8_t>(last_level_)) {
      RCLCPP_WARN(
          get_logger(),
          "风险等级变化: L%d -> L%u score=%.2f reason=%s",
          last_level_,
          msg.level,
          msg.score,
          msg.primary_reason.c_str());
      last_level_ = static_cast<int>(msg.level);
    }
  }

  void on_warning(const ev_ads_interfaces::msg::WarningCommand& msg) {
    RCLCPP_WARN(
        get_logger(),
        "告警 L%u: %s icon=%s beep=%uHz vib=%u duration=%ums",
        msg.level,
        msg.voice_text.empty() ? "无语音文本" : msg.voice_text.c_str(),
        msg.screen_icon.c_str(),
        msg.beep_freq_hz,
        msg.vibration_pattern,
        msg.duration_ms);
  }

  void dashboard() {
    if (!has_risk_) {
      std::printf("\r[hmi] 等待 /decision/risk_state ...");
      std::fflush(stdout);
      return;
    }

    std::ostringstream out;
    out << level_color(risk_.level) << "[L" << static_cast<int>(risk_.level)
        << "] R=" << risk_.score << " " << risk_.primary_reason << "\033[0m ";
    if (has_front_) {
      out << " 前向[" << health_text(front_.health) << " ttc=" << front_.ttc
          << "s d=" << front_.distance << "m]";
    }
    if (has_rear_) {
      out << " 后向[L" << static_cast<int>(rear_.zone_left) << " C"
          << static_cast<int>(rear_.zone_center) << " R"
          << static_cast<int>(rear_.zone_right) << "]";
    }
    if (has_driver_) {
      out << " DMS[face=" << static_cast<int>(driver_.face_visible)
          << " eye=" << driver_.eye_closure_ratio
          << " fatigue=" << driver_.fatigue_score << "]";
    }
    if (has_motion_) {
      out << " IMU[roll=" << motion_.roll << " yaw_rate=" << motion_.yaw_rate
          << "]";
    }
    if (has_mmwave_) {
      out << " 毫米波[" << health_text(mmwave_.health)
          << " hr=" << mmwave_.heart_rate
          << " br=" << mmwave_.breath_rate
          << " d=" << mmwave_.distance << "cm]";
    }
    if (has_brake_ && brake_.enable) {
      out << " \033[91mBRAKE\033[0m";
    }
    std::printf("\r%s    ", out.str().c_str());
    std::fflush(stdout);
  }

  int last_level_{-1};
  bool has_risk_{false};
  bool has_front_{false};
  bool has_rear_{false};
  bool has_driver_{false};
  bool has_motion_{false};
  bool has_mmwave_{false};
  bool has_brake_{false};
  ev_ads_interfaces::msg::RiskState risk_;
  ev_ads_interfaces::msg::FrontRisk front_;
  ev_ads_interfaces::msg::BlindSpotState rear_;
  ev_ads_interfaces::msg::DriverState driver_;
  ev_ads_interfaces::msg::VehicleMotion motion_;
  ev_ads_interfaces::msg::MmWaveVital mmwave_;
  ev_ads_interfaces::msg::BrakeCommand brake_;
  rclcpp::Subscription<ev_ads_interfaces::msg::RiskState>::SharedPtr risk_sub_;
  rclcpp::Subscription<ev_ads_interfaces::msg::WarningCommand>::SharedPtr warning_sub_;
  rclcpp::Subscription<ev_ads_interfaces::msg::BrakeCommand>::SharedPtr brake_sub_;
  rclcpp::Subscription<ev_ads_interfaces::msg::FrontRisk>::SharedPtr front_sub_;
  rclcpp::Subscription<ev_ads_interfaces::msg::BlindSpotState>::SharedPtr rear_sub_;
  rclcpp::Subscription<ev_ads_interfaces::msg::DriverState>::SharedPtr driver_sub_;
  rclcpp::Subscription<ev_ads_interfaces::msg::VehicleMotion>::SharedPtr motion_sub_;
  rclcpp::Subscription<ev_ads_interfaces::msg::MmWaveVital>::SharedPtr mmwave_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // 命名空间 ev_ads_runtime_cpp

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ev_ads_runtime_cpp::HmiNodeCpp>());
  rclcpp::shutdown();
  return 0;
}
