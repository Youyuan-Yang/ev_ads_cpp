#!/usr/bin/env bash
# EV-ADS · RK3588 一次性依赖安装
#
# 用法（首次部署时）:
#     bash install_deps.sh
#     # 之后重启
#
# 该脚本会：
#   1) apt 更新 + 装系统依赖（蓝牙/网络/I2C/视频/编译）
#   2) 装 ROS 2 Humble (ros-base)
#   3) 装 C++ OpenCV 开发库
#   4) 把当前用户加进 dialout/video/i2c/bluetooth/plugdev 组
#   5) 启用蓝牙服务
#   6) （可选）注册 systemd 开机自启服务

set -e

# ===== 颜色 =====
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[0;33m'; NC='\033[0m'
log()  { echo -e "${GREEN}[install]${NC} $*"; }
warn() { echo -e "${YELLOW}[install]${NC} $*"; }
err()  { echo -e "${RED}[install]${NC} $*" >&2; }

if [ "$(id -u)" -eq 0 ]; then
  err "请用普通用户跑（脚本里需要的 sudo 会自动提权），不要直接 root。"
  exit 1
fi

EV_ADS_DIR="$(cd "$(dirname "$0")/.." && pwd)"
log "EV-ADS 根目录: $EV_ADS_DIR"

# =====================================================================
# 1) apt 系统依赖
# =====================================================================
log "1/6 apt 更新 & 系统依赖 ..."
sudo apt update
sudo apt install -y --no-install-recommends \
  curl wget git build-essential gnupg lsb-release software-properties-common \
  bluez bluez-tools libbluetooth-dev \
  network-manager wireless-tools wpasupplicant \
  chrony ntpdate \
  i2c-tools v4l-utils usbutils \
  libopencv-dev sqlite3 libsqlite3-dev cmake

# =====================================================================
# 2) ROS 2 Humble
# =====================================================================
if ! command -v ros2 >/dev/null 2>&1; then
  log "2/6 装 ROS 2 Humble ..."
  sudo add-apt-repository -y universe
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  CODENAME=$(. /etc/os-release && echo "$UBUNTU_CODENAME")
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $CODENAME main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  sudo apt update
  sudo apt install -y \
    ros-humble-ros-base ros-humble-cv-bridge \
    ros-humble-rosbag2 ros-humble-rosbag2-transport \
    ros-humble-ament-cmake \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    python3-colcon-common-extensions python3-rosdep python3-vcstool
  sudo rosdep init 2>/dev/null || true
  rosdep update || warn "rosdep update 失败，可稍后手动 'rosdep update'"
else
  log "2/6 ROS 2 已安装：$(ros2 --version 2>&1 | head -1)"
fi

# 把 ROS source 写进 bashrc（幂等）
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc 2>/dev/null; then
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
  log "  已写入 ~/.bashrc（新终端自动 source）"
fi

# =====================================================================
# 3) 用户组
# =====================================================================
log "3/6 把 $USER 加入 dialout/video/i2c/bluetooth/plugdev 组 ..."
sudo usermod -aG dialout,video,i2c,bluetooth,plugdev "$USER"

# =====================================================================
# 4) 蓝牙服务
# =====================================================================
log "4/6 启用蓝牙服务 ..."
sudo systemctl enable --now bluetooth

# =====================================================================
# 4.5) 摄像头 udev 稳定别名
# =====================================================================
if [ -f "$EV_ADS_DIR/deploy/udev/99-ev-ads-cameras.rules" ]; then
  log "4.5/6 安装摄像头 udev 规则 ..."
  sudo cp "$EV_ADS_DIR/deploy/udev/99-ev-ads-cameras.rules" /etc/udev/rules.d/99-ev-ads-cameras.rules
  sudo udevadm control --reload-rules
  sudo udevadm trigger --subsystem-match=video4linux || true
  log "  已配置 /dev/ev_ads/front_camera、rear_fisheye、driver_face"
fi

# =====================================================================
# 5) 注册 systemd 开机自启
# =====================================================================
read -r -p "是否注册 systemd 开机自启 ev-ads.service？[y/N] " yn
if [[ "$yn" =~ ^[Yy]$ ]]; then
  log "5/6 安装 systemd 单元 ..."
  sudo cp "$EV_ADS_DIR/deploy/systemd/ev-ads.service" /etc/systemd/system/ev-ads.service
  # 替换占位符 EV_ADS_DIR、USERNAME 为实际值
  sudo sed -i "s|@EV_ADS_DIR@|$EV_ADS_DIR|g" /etc/systemd/system/ev-ads.service
  sudo sed -i "s|@USERNAME@|$USER|g" /etc/systemd/system/ev-ads.service
  sudo touch /var/log/ev_ads.log
  sudo chown "$USER:$USER" /var/log/ev_ads.log
  sudo systemctl daemon-reload
  sudo systemctl enable ev-ads.service
  log "  ev-ads.service 已 enable（下次开机自启）"
  log "  立即启动: sudo systemctl start ev-ads.service"
  log "  查看日志: tail -f /var/log/ev_ads.log"
else
  log "5/6 跳过 systemd 注册（之后想加可重跑此脚本）"
fi

# =====================================================================
# 编译 workspace（可选，依赖装完后做一次）
# =====================================================================
read -r -p "是否现在 colcon build 整个 ros2_ws？[y/N] " yn2
if [[ "$yn2" =~ ^[Yy]$ ]]; then
  log "6/6 colcon build ..."
  ( cd "$EV_ADS_DIR/ros2_ws" \
    && source /opt/ros/humble/setup.bash \
    && rosdep install --from-paths src --ignore-src -r -y || true \
    && colcon build --symlink-install )
  log "build 完成"
fi

echo
log "=========================================="
log " 全部完成！现在请：sudo reboot"
log " 重启后组权限才会生效（dialout/video/...）"
log "=========================================="
