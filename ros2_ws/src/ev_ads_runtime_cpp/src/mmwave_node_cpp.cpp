#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <fstream>
#include <memory>
#include <regex>
#include <string>

#include "ev_ads_interfaces/msg/mm_wave_vital.hpp"
#include "ev_ads_runtime_cpp/common.hpp"
#include "ev_ads_runtime_cpp/topics.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ev_ads_runtime_cpp {
namespace {

bool read_number_field(const std::string& text, const std::string& field, double* output) {
  const std::regex pattern("\"" + field + "\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?)");
  std::smatch match;
  if (!std::regex_search(text, match, pattern)) {
    return false;
  }
  *output = std::stod(match[1].str());
  return true;
}

bool read_int_field(const std::string& text, const std::string& field, int* output) {
  double value = 0.0;
  if (!read_number_field(text, field, &value)) {
    return false;
  }
  *output = static_cast<int>(value);
  return true;
}

}  // 匿名命名空间

class MmWaveNodeCpp final : public rclcpp::Node {
 public:
  MmWaveNodeCpp() : Node("mmwave_node_cpp") {
    mode_ = declare_parameter<std::string>("mode", "fake");
    jsonl_path_ = declare_parameter<std::string>("jsonl_path", "");
    frame_id_ = declare_parameter<std::string>("frame_id", "mmwave_link");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
    t0_ = now();

    pub_ = create_publisher<ev_ads_interfaces::msg::MmWaveVital>(
        topics_.mmwave_vital, rclcpp::QoS(10));

    if (mode_ == "jsonl" && !jsonl_path_.empty()) {
      jsonl_.open(jsonl_path_);
      if (!jsonl_.is_open()) {
        RCLCPP_ERROR(get_logger(), "毫米波 JSONL 文件打开失败: %s", jsonl_path_.c_str());
      }
    }

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_)),
        std::bind(&MmWaveNodeCpp::tick, this));

    RCLCPP_INFO(get_logger(), "毫米波 C++ 节点启动，模式=%s", mode_.c_str());
    if (mode_ == "ble") {
      RCLCPP_WARN(
          get_logger(),
          "当前已移除 Python BLE。C++ BLE 后端需在 RK3588 上接 BlueZ D-Bus 后启用。");
    }
  }

 private:
  ev_ads_interfaces::msg::MmWaveVital make_base_msg(uint8_t health) {
    ev_ads_interfaces::msg::MmWaveVital msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.seq = ++seq_;
    msg.health = health;
    msg.drop_count = drop_count_;
    msg.parse_error_count = parse_error_count_;
    msg.stale_ms = health == HEALTH_OK ? 0 : 1000;
    return msg;
  }

  ev_ads_interfaces::msg::MmWaveVital fake_msg() {
    const double t = (now() - t0_).seconds();
    auto msg = make_base_msg(HEALTH_OK);
    msg.breath_rate = static_cast<float>(16.0 + 2.0 * std::sin(2.0 * M_PI * 0.10 * t));
    msg.heart_rate = static_cast<float>(75.0 + 5.0 * std::sin(2.0 * M_PI * 0.05 * t));
    msg.distance = static_cast<float>(50.0 + 5.0 * std::sin(2.0 * M_PI * 0.07 * t));
    msg.confidence = 0.85f;
    msg.status = 0x0F;
    return msg;
  }

  ev_ads_interfaces::msg::MmWaveVital json_msg() {
    std::string line;
    if (!jsonl_.is_open() || !std::getline(jsonl_, line)) {
      return make_base_msg(HEALTH_DISCONNECTED);
    }

    double breath = 0.0;
    double heart = 0.0;
    double distance = 0.0;
    int status = 0;
    bool ok = true;
    ok = read_number_field(line, "br", &breath) && ok;
    ok = read_number_field(line, "hr", &heart) && ok;
    ok = read_number_field(line, "d", &distance) && ok;
    ok = read_int_field(line, "st", &status) && ok;
    if (!ok) {
      ++parse_error_count_;
      return make_base_msg(HEALTH_ERROR);
    }

    auto msg = make_base_msg(HEALTH_OK);
    msg.breath_rate = static_cast<float>(breath);
    msg.heart_rate = static_cast<float>(heart);
    msg.distance = static_cast<float>(distance);
    msg.status = static_cast<uint8_t>(status);
    msg.confidence = 0.25f * ((status & 0x01) != 0) +
        0.25f * ((status & 0x02) != 0) +
        0.25f * ((status & 0x04) != 0) +
        0.10f * ((status & 0x08) != 0);
    msg.confidence = static_cast<float>(clamp(static_cast<double>(msg.confidence), 0.0, 1.0));
    return msg;
  }

  void tick() {
    if (mode_ == "fake") {
      pub_->publish(fake_msg());
      return;
    }
    if (mode_ == "jsonl") {
      pub_->publish(json_msg());
      return;
    }
    pub_->publish(make_base_msg(HEALTH_DISCONNECTED));
  }

  std::string mode_;
  std::string jsonl_path_;
  std::string frame_id_;
  RuntimeTopics topics_;
  double publish_rate_hz_{10.0};
  uint32_t seq_{0};
  uint32_t drop_count_{0};
  uint32_t parse_error_count_{0};
  std::ifstream jsonl_;
  rclcpp::Time t0_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<ev_ads_interfaces::msg::MmWaveVital>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // 命名空间 ev_ads_runtime_cpp

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ev_ads_runtime_cpp::MmWaveNodeCpp>());
  rclcpp::shutdown();
  return 0;
}
