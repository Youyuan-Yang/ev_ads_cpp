#!/usr/bin/env bash
# EV-ADS · RK3588 开机启动脚本
#
# 流程: 等网卡 → 连 WiFi → 强制时间同步 → source ROS → ros2 launch
#
# 可被 systemd 调用 (ev-ads.service)，也可手动跑:
#     bash ev_ads_boot.sh
#
# RTC 没电时 RK3588 默认时间是 2013-01-01；本脚本会强制 NTP 步进，
# 失败再用 HTTPS Date Header 兜底，确保启动后时钟正确。

set -u

# =========================================================
# ████  改这里：WiFi 配置  ████
# =========================================================
# 在下面把注释取消并填上你的 WiFi。如果板子用网线，留着注释即可。
#WIFI_SSID="MyWiFiName"
#WIFI_PASSWORD="MyWiFiPassword"
WIFI_SSID="${WIFI_SSID:-}"
WIFI_PASSWORD="${WIFI_PASSWORD:-}"

# 后备 NTP（按顺序尝试）
NTP_SERVERS=(
  "ntp.aliyun.com"
  "ntp.tencent.com"
  "cn.pool.ntp.org"
  "pool.ntp.org"
)
# 后备 HTTPS Date (NTP 全失败时用)
HTTP_DATE_URLS=(
  "https://www.aliyun.com"
  "https://www.baidu.com"
  "https://www.qq.com"
)

# EV-ADS 仓库根目录（脚本自定位）
EV_ADS_DIR="$(cd "$(dirname "$0")/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"

# =========================================================
# 日志
# =========================================================
log()  { echo "[$(date '+%H:%M:%S')][boot] $*"; }
warn() { echo "[$(date '+%H:%M:%S')][boot][WARN] $*" >&2; }
err()  { echo "[$(date '+%H:%M:%S')][boot][ERR] $*" >&2; }

# =========================================================
# Step 1 · 等系统起来 + 启网卡
# =========================================================
log "=================================================="
log " EV-ADS boot @ $(date)"
log " dir: $EV_ADS_DIR"
log "=================================================="

log "[1/4] 等待网卡就绪 ..."
sudo systemctl start NetworkManager 2>/dev/null || true
sleep 2

# =========================================================
# Step 2 · WiFi
# =========================================================
if [ -n "$WIFI_SSID" ]; then
  log "[2/4] 连接 WiFi: $WIFI_SSID ..."
  sudo nmcli radio wifi on || true
  # 等扫描列表
  for _ in 1 2 3 4 5; do
    if nmcli -t -f SSID dev wifi list --rescan yes 2>/dev/null | grep -q "^${WIFI_SSID}$"; then
      break
    fi
    sleep 2
  done
  # 已连过的同名配置直接 up；否则新建
  if nmcli -t -f NAME connection show | grep -qx "$WIFI_SSID"; then
    sudo nmcli connection up "$WIFI_SSID" || warn "WiFi up 失败，继续往下试"
  else
    if [ -n "$WIFI_PASSWORD" ]; then
      sudo nmcli device wifi connect "$WIFI_SSID" password "$WIFI_PASSWORD" \
        || warn "WiFi 连接失败，继续往下试"
    else
      sudo nmcli device wifi connect "$WIFI_SSID" \
        || warn "WiFi 连接失败 (无密码模式)"
    fi
  fi
else
  log "[2/4] 未配置 WIFI_SSID，跳过（假设走有线）"
fi

# 等网络可达
log "    等待外网可达 (最多 30s) ..."
for i in $(seq 1 15); do
  if ping -c 1 -W 2 223.5.5.5 >/dev/null 2>&1 \
     || ping -c 1 -W 2 8.8.8.8 >/dev/null 2>&1; then
    log "    外网 OK (第 ${i} 次试)"
    NETWORK_OK=1
    break
  fi
  sleep 2
done
NETWORK_OK="${NETWORK_OK:-0}"

# =========================================================
# Step 3 · 时间同步（关键：RTC 没电时系统时间是 2013）
# =========================================================
log "[3/4] 系统时间检查与强制同步 ..."
NOW_TS=$(date +%s)
THRESH=1700000000   # 2023-11-14 之前都视为不可信
log "    当前: $(date)  ts=$NOW_TS"

NEED_FORCE=0
if [ "$NOW_TS" -lt "$THRESH" ]; then
  warn "    系统时间明显早于 2023-11，强制重新同步"
  NEED_FORCE=1
fi

if [ "$NETWORK_OK" = "1" ]; then
  # 临时停 chrony，避免它拒绝大跳
  sudo systemctl stop chrony 2>/dev/null || true

  TIME_OK=0
  for srv in "${NTP_SERVERS[@]}"; do
    log "    尝试 NTP: $srv"
    if sudo chronyd -q "server $srv iburst maxsamples 1" 2>&1 | grep -q "System clock"; then
      TIME_OK=1
      break
    fi
    # chronyd 没成功就再试 ntpdate（更老但很可靠）
    if command -v ntpdate >/dev/null && sudo ntpdate -u -t 5 "$srv" 2>/dev/null; then
      TIME_OK=1
      break
    fi
  done

  if [ "$TIME_OK" = "0" ]; then
    warn "    所有 NTP 失败，回退 HTTPS Date header ..."
    for url in "${HTTP_DATE_URLS[@]}"; do
      D=$(curl -sI --max-time 5 "$url" 2>/dev/null \
            | awk -F': ' 'tolower($1)=="date" {sub(/\r$/, "", $2); print $2; exit}')
      if [ -n "$D" ]; then
        log "    HTTPS Date from $url: $D"
        sudo date -s "$D" >/dev/null && TIME_OK=1 && break
      fi
    done
  fi

  # 重启 chrony 让它稳定守时
  sudo systemctl start chrony 2>/dev/null || true

  if [ "$TIME_OK" = "1" ]; then
    log "    时间同步成功 → $(date)"
    # 写回 RTC（如果有 hwclock）
    if command -v hwclock >/dev/null; then
      sudo hwclock --systohc 2>/dev/null || true
    fi
  else
    err "    时间同步失败，但程序仍会启动（话题时间戳可能错乱）"
  fi
else
  warn "    无网络可用，跳过时间同步（RTC 时间 = $(date)）"
fi

# =========================================================
# Step 4 · 启动 ROS 链路
# =========================================================
log "[4/4] 启动 EV-ADS ROS 2 链路 ..."

if [ ! -f "$ROS_SETUP" ]; then
  err "找不到 $ROS_SETUP，是否装好 ROS 2 Humble？请先跑 install_deps.sh"
  exit 2
fi
if [ ! -f "$EV_ADS_DIR/ros2_ws/install/setup.bash" ]; then
  err "$EV_ADS_DIR/ros2_ws/install/setup.bash 不存在，请先 colcon build"
  exit 3
fi

# 让子进程能继承 ROS 环境
# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1090
source "$EV_ADS_DIR/ros2_ws/install/setup.bash"

# C++ runtime：RK3588 上优先使用 C++ 摄像头/IMU/感知/融合节点。
LAUNCH_CMD=(ros2 launch ev_ads_bringup ev_ads_cpp_runtime.launch.xml use_fakes:=false)
# 例：纯演示
# LAUNCH_CMD=(ros2 launch ev_ads_bringup ev_ads_cpp_runtime.launch.xml use_fakes:=true perception_mode:=scripted)
# 例：开启真实制动（仅低速台架）
# LAUNCH_CMD=(ros2 launch ev_ads_bringup ev_ads_cpp_runtime.launch.xml use_fakes:=false enable_real_brake:=true)

log "exec: ${LAUNCH_CMD[*]}"
exec "${LAUNCH_CMD[@]}"
