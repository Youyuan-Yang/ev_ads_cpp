#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <termios.h>
#include <thread>
#include <utility>
#include <unistd.h>
#include <vector>

#ifdef __linux__
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#endif

#include "ev_ads_interfaces/msg/vehicle_motion.hpp"
#include "ev_ads_runtime_cpp/common.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace ev_ads_runtime_cpp {

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct ImuSample {
  double stamp_s = 0.0;
  Vec3 accel;
  Vec3 gyro;
};

double steady_seconds() {
  using clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

Vec3 operator+(const Vec3& a, const Vec3& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 operator-(const Vec3& a, const Vec3& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 operator*(const Vec3& a, double s) {
  return {a.x * s, a.y * s, a.z * s};
}

double norm(const Vec3& v) {
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

class LowPass3 {
 public:
  explicit LowPass3(double alpha) : alpha_(clamp(alpha, 0.0, 1.0)) {}
  Vec3 update(const Vec3& v) {
    if (!ready_) {
      value_ = v;
      ready_ = true;
      return value_;
    }
    value_ = value_ * (1.0 - alpha_) + v * alpha_;
    return value_;
  }

 private:
  double alpha_{0.2};
  bool ready_{false};
  Vec3 value_;
};

class MountingTransform {
 public:
  explicit MountingTransform(const std::vector<double>& rpy_deg) {
    const double r = deg(rpy_deg.size() > 0 ? rpy_deg[0] : 0.0);
    const double p = deg(rpy_deg.size() > 1 ? rpy_deg[1] : 0.0);
    const double y = deg(rpy_deg.size() > 2 ? rpy_deg[2] : 0.0);
    const double cr = std::cos(r), sr = std::sin(r);
    const double cp = std::cos(p), sp = std::sin(p);
    const double cy = std::cos(y), sy = std::sin(y);
    // 旋转顺序：绕 Z 轴偏航、绕 Y 轴俯仰、绕 X 轴横滚。
    m_[0][0] = cy * cp;
    m_[0][1] = cy * sp * sr - sy * cr;
    m_[0][2] = cy * sp * cr + sy * sr;
    m_[1][0] = sy * cp;
    m_[1][1] = sy * sp * sr + cy * cr;
    m_[1][2] = sy * sp * cr - cy * sr;
    m_[2][0] = -sp;
    m_[2][1] = cp * sr;
    m_[2][2] = cp * cr;
  }
  Vec3 apply(const Vec3& v) const {
    return {
        m_[0][0] * v.x + m_[0][1] * v.y + m_[0][2] * v.z,
        m_[1][0] * v.x + m_[1][1] * v.y + m_[1][2] * v.z,
        m_[2][0] * v.x + m_[2][1] * v.y + m_[2][2] * v.z};
  }

 private:
  static double deg(double v) { return v * M_PI / 180.0; }
  double m_[3][3]{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
};

class BiasEstimator {
 public:
  explicit BiasEstimator(double duration_s) : duration_s_(duration_s) {}
  void update(double t, const Vec3& accel, const Vec3& gyro) {
    if (ready_) {
      return;
    }
    if (start_s_ < 0.0) {
      start_s_ = t;
    }
    accel_sum_ = accel_sum_ + accel;
    gyro_sum_ = gyro_sum_ + gyro;
    count_++;
    if (t - start_s_ >= duration_s_ && count_ > 0) {
      accel_bias_ = accel_sum_ * (1.0 / static_cast<double>(count_));
      accel_bias_.z -= 9.80665;
      gyro_bias_ = gyro_sum_ * (1.0 / static_cast<double>(count_));
      ready_ = true;
    }
  }
  Vec3 correct_accel(const Vec3& v) const { return ready_ ? v - accel_bias_ : v; }
  Vec3 correct_gyro(const Vec3& v) const { return ready_ ? v - gyro_bias_ : v; }
  bool ready() const { return ready_; }

 private:
  double duration_s_{2.0};
  double start_s_{-1.0};
  uint64_t count_{0};
  bool ready_{false};
  Vec3 accel_sum_;
  Vec3 gyro_sum_;
  Vec3 accel_bias_;
  Vec3 gyro_bias_;
};

std::pair<double, double> estimate_roll_pitch(const Vec3& a) {
  const double roll = std::atan2(a.y, a.z);
  const double pitch = std::atan2(-a.x, std::sqrt(a.y * a.y + a.z * a.z));
  return {roll, pitch};
}

class ImuDriver {
 public:
  virtual ~ImuDriver() = default;
  virtual bool open() = 0;
  virtual std::optional<ImuSample> read_one() = 0;
  virtual void close() = 0;
};

class FakeImuDriver final : public ImuDriver {
 public:
  explicit FakeImuDriver(double rate_hz) : rate_hz_(rate_hz) {}
  bool open() override {
    t0_ = steady_seconds();
    last_ = t0_;
    return true;
  }
  std::optional<ImuSample> read_one() override {
    const double interval = 1.0 / std::max(1.0, rate_hz_);
    const double now_s = steady_seconds();
    const double sleep_s = last_ + interval - now_s;
    if (sleep_s > 0.0) {
      std::this_thread::sleep_for(std::chrono::duration<double>(sleep_s));
    }
    const double t = steady_seconds();
    last_ = t;
    const double rel = t - t0_;
    const double roll = (5.0 * M_PI / 180.0) * std::sin(2.0 * M_PI * 0.3 * rel);
    const double gyro_roll = (5.0 * M_PI / 180.0) * 2.0 * M_PI * 0.3 *
                             std::cos(2.0 * M_PI * 0.3 * rel);
    std::normal_distribution<double> noise(0.0, 0.05);
    ImuSample s;
    s.stamp_s = t;
    s.accel = {noise(rng_), 9.80665 * std::sin(roll) + noise(rng_),
               9.80665 * std::cos(roll) + noise(rng_)};
    s.gyro = {gyro_roll, 0.0, noise(rng_)};
    return s;
  }
  void close() override {}

 private:
  double rate_hz_{100.0};
  double t0_{0.0};
  double last_{0.0};
  std::mt19937 rng_{std::random_device{}()};
};

class UartJsonImuDriver final : public ImuDriver {
 public:
  UartJsonImuDriver(std::string port, int baud) : port_(std::move(port)), baud_(baud) {}
  bool open() override {
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      return false;
    }
    termios tio{};
    if (tcgetattr(fd_, &tio) == 0) {
      cfmakeraw(&tio);
      cfsetispeed(&tio, baud_to_speed(baud_));
      cfsetospeed(&tio, baud_to_speed(baud_));
      tio.c_cflag |= CLOCAL | CREAD;
      tio.c_cc[VMIN] = 0;
      tio.c_cc[VTIME] = 1;
      tcsetattr(fd_, TCSANOW, &tio);
    }
    return true;
  }
  std::optional<ImuSample> read_one() override {
    char c = 0;
    for (int i = 0; i < 256; ++i) {
      const ssize_t n = ::read(fd_, &c, 1);
      if (n == 1) {
        if (c == '\n') {
          auto out = parse_line(line_);
          line_.clear();
          return out;
        }
        line_.push_back(c);
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }
    return std::nullopt;
  }
  void close() override {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

 private:
  static speed_t baud_to_speed(int baud) {
    switch (baud) {
      case 115200:
        return B115200;
      case 460800:
        return B460800;
      case 921600:
        return B921600;
      default:
        return B115200;
    }
  }
  static bool find_number(const std::string& line, const std::string& key, double* out) {
    const std::string needle = "\"" + key + "\"";
    auto pos = line.find(needle);
    if (pos == std::string::npos) {
      return false;
    }
    pos = line.find(':', pos);
    if (pos == std::string::npos) {
      return false;
    }
    char* end = nullptr;
    *out = std::strtod(line.c_str() + pos + 1, &end);
    return end != line.c_str() + pos + 1;
  }
  static std::optional<ImuSample> parse_line(const std::string& line) {
    double ax, ay, az, gx, gy, gz;
    if (!find_number(line, "ax", &ax) || !find_number(line, "ay", &ay) ||
        !find_number(line, "az", &az) || !find_number(line, "gx", &gx) ||
        !find_number(line, "gy", &gy) || !find_number(line, "gz", &gz)) {
      return std::nullopt;
    }
    return ImuSample{steady_seconds(), {ax, ay, az}, {gx, gy, gz}};
  }

  std::string port_;
  int baud_{921600};
  int fd_{-1};
  std::string line_;
};

class I2cMpu6050LikeDriver final : public ImuDriver {
 public:
  I2cMpu6050LikeDriver(int bus, int addr) : bus_(bus), addr_(addr) {}
  bool open() override {
#ifdef __linux__
    const std::string path = "/dev/i2c-" + std::to_string(bus_);
    fd_ = ::open(path.c_str(), O_RDWR);
    if (fd_ < 0) {
      return false;
    }
    if (ioctl(fd_, I2C_SLAVE, addr_) < 0) {
      close();
      return false;
    }
    write_reg(0x6B, 0x00);
    return true;
#else
    return false;
#endif
  }
  std::optional<ImuSample> read_one() override {
#ifdef __linux__
    if (fd_ < 0) {
      return std::nullopt;
    }
    int16_t raw[7]{};
    for (int i = 0; i < 7; ++i) {
      auto v = read_i16(0x3B + i * 2);
      if (!v) {
        return std::nullopt;
      }
      raw[i] = *v;
    }
    constexpr double accel_scale = 9.80665 / 16384.0;
    constexpr double gyro_scale = (M_PI / 180.0) / 131.0;
    return ImuSample{
        steady_seconds(),
        {raw[0] * accel_scale, raw[1] * accel_scale, raw[2] * accel_scale},
        {raw[4] * gyro_scale, raw[5] * gyro_scale, raw[6] * gyro_scale}};
#else
    return std::nullopt;
#endif
  }
  void close() override {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

 private:
#ifdef __linux__
  bool write_reg(uint8_t reg, uint8_t value) {
    const uint8_t data[2] = {reg, value};
    return ::write(fd_, data, 2) == 2;
  }
  std::optional<int16_t> read_i16(uint8_t reg) {
    if (::write(fd_, &reg, 1) != 1) {
      return std::nullopt;
    }
    uint8_t buf[2]{};
    if (::read(fd_, buf, 2) != 2) {
      return std::nullopt;
    }
    const uint16_t v = (static_cast<uint16_t>(buf[0]) << 8) | buf[1];
    return static_cast<int16_t>(v);
  }
#endif
  int bus_{1};
  int addr_{0x68};
  int fd_{-1};
};

class ImuNodeCpp final : public rclcpp::Node {
 public:
  ImuNodeCpp() : Node("imu_node_cpp") {
    driver_name_ = declare_parameter<std::string>("driver", "fake");
    port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    baud_ = declare_parameter<int>("baud", 921600);
    bus_ = declare_parameter<int>("bus", 1);
    addr_ = declare_parameter<int>("addr", 0x68);
    rate_hz_ = declare_parameter<double>("rate_hz", 100.0);
    publish_imu_ = declare_parameter<bool>("publish_imu", true);
    mount_rpy_deg_ = declare_parameter<std::vector<double>>("mount_rpy_deg", {0.0, 0.0, 0.0});
    lowpass_alpha_ = declare_parameter<double>("lowpass_alpha", 0.2);
    bias_seconds_ = declare_parameter<double>("bias_seconds", 2.0);
    hard_brake_ = declare_parameter<double>("hard_brake_g", 0.6) * 9.80665;
    lean_ = declare_parameter<double>("lean_deg", 25.0) * M_PI / 180.0;
    bump_jerk_ = declare_parameter<double>("bump_jerk", 30.0);
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");

    if (driver_name_ == "uart") {
      driver_ = std::make_unique<UartJsonImuDriver>(port_, baud_);
    } else if (driver_name_ == "i2c") {
      driver_ = std::make_unique<I2cMpu6050LikeDriver>(bus_, addr_);
    } else {
      driver_ = std::make_unique<FakeImuDriver>(rate_hz_);
    }

    accel_lp_ = std::make_unique<LowPass3>(lowpass_alpha_);
    gyro_lp_ = std::make_unique<LowPass3>(lowpass_alpha_);
    bias_ = std::make_unique<BiasEstimator>(bias_seconds_);
    mount_ = std::make_unique<MountingTransform>(mount_rpy_deg_);

    pub_motion_ = create_publisher<ev_ads_interfaces::msg::VehicleMotion>(
        "/vehicle/motion", rclcpp::QoS(rclcpp::KeepLast(20)).reliable());
    if (publish_imu_) {
      pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", rclcpp::QoS(20));
    }

    worker_ = std::thread([this]() { run(); });
    RCLCPP_INFO(get_logger(), "IMU 节点启动，驱动=%s 频率=%.1f", driver_name_.c_str(), rate_hz_);
  }

  ~ImuNodeCpp() override {
    stop_.store(true);
    if (worker_.joinable()) {
      worker_.join();
    }
  }

 private:
  void run() {
    if (!driver_->open()) {
      RCLCPP_ERROR(get_logger(), "IMU 打开失败，驱动=%s", driver_name_.c_str());
      return;
    }
    while (rclcpp::ok() && !stop_.load()) {
      auto sample = driver_->read_one();
      if (!sample) {
        continue;
      }
      process(*sample);
    }
    driver_->close();
  }

  void process(const ImuSample& sample) {
    bias_->update(sample.stamp_s, sample.accel, sample.gyro);
    Vec3 a = bias_->correct_accel(sample.accel);
    Vec3 g = bias_->correct_gyro(sample.gyro);
    a = mount_->apply(a);
    g = mount_->apply(g);
    a = accel_lp_->update(a);
    g = gyro_lp_->update(g);

    const auto rp = estimate_roll_pitch(a);
    const double roll = rp.first;
    const double pitch = rp.second;
    const double accel_norm = norm(a);
    double jerk = 0.0;
    if (last_accel_norm_ >= 0.0 && last_sample_s_ >= 0.0) {
      const double dt = std::max(1e-3, sample.stamp_s - last_sample_s_);
      jerk = (accel_norm - last_accel_norm_) / dt;
    }
    last_accel_norm_ = accel_norm;
    last_sample_s_ = sample.stamp_s;

    uint32_t flags = 0;
    if (std::abs(a.x) > hard_brake_) {
      flags |= MOTION_HARD_BRAKE;
    }
    if (std::abs(roll) > lean_) {
      flags |= MOTION_LEAN;
    }
    if (std::abs(jerk) > bump_jerk_) {
      flags |= MOTION_BUMP;
    }

    ev_ads_interfaces::msg::VehicleMotion msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.roll = static_cast<float>(roll);
    msg.pitch = static_cast<float>(pitch);
    msg.yaw_rate = static_cast<float>(g.z);
    msg.accel_norm = static_cast<float>(accel_norm);
    msg.jerk = static_cast<float>(jerk);
    msg.confidence = bias_->ready() ? 1.0f : 0.3f;
    msg.motion_flags = flags;
    msg.health = bias_->ready() ? HEALTH_OK : HEALTH_STALE;
    pub_motion_->publish(msg);

    if (pub_imu_) {
      sensor_msgs::msg::Imu imu;
      imu.header = msg.header;
      imu.linear_acceleration.x = a.x;
      imu.linear_acceleration.y = a.y;
      imu.linear_acceleration.z = a.z;
      imu.angular_velocity.x = g.x;
      imu.angular_velocity.y = g.y;
      imu.angular_velocity.z = g.z;
      imu.orientation_covariance[0] = -1.0;
      pub_imu_->publish(imu);
    }
  }

  std::string driver_name_;
  std::string port_;
  int baud_{921600};
  int bus_{1};
  int addr_{0x68};
  double rate_hz_{100.0};
  bool publish_imu_{true};
  std::vector<double> mount_rpy_deg_;
  double lowpass_alpha_{0.2};
  double bias_seconds_{2.0};
  double hard_brake_{0.6 * 9.80665};
  double lean_{25.0 * M_PI / 180.0};
  double bump_jerk_{30.0};
  std::string frame_id_{"imu_link"};

  std::unique_ptr<ImuDriver> driver_;
  std::unique_ptr<LowPass3> accel_lp_;
  std::unique_ptr<LowPass3> gyro_lp_;
  std::unique_ptr<BiasEstimator> bias_;
  std::unique_ptr<MountingTransform> mount_;
  std::thread worker_;
  std::atomic<bool> stop_{false};
  double last_accel_norm_{-1.0};
  double last_sample_s_{-1.0};
  rclcpp::Publisher<ev_ads_interfaces::msg::VehicleMotion>::SharedPtr pub_motion_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
};

}  // 命名空间 ev_ads_runtime_cpp

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ev_ads_runtime_cpp::ImuNodeCpp>());
  rclcpp::shutdown();
  return 0;
}
