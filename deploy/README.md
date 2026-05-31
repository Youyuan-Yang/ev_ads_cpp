# EV-ADS · RK3588 部署脚本

| 脚本 | 何时跑 | 作用 |
|------|--------|------|
| `install_deps.sh`        | 首次部署时跑一次 | 装 ROS 2 Humble + C++ OpenCV 开发库 + 权限 + systemd 服务 |
| `ev_ads_boot.sh`         | 每次开机由 systemd 自动跑 | 等网 → 连 WiFi → 强制时间同步 → 启动 ROS 链路 |
| `udev/99-ev-ads-cameras.rules` | 首次部署时安装 | 把三路 UVC 摄像头固定为 `/dev/ev_ads/*` |
| `systemd/ev-ads.service` | 由 `install_deps.sh` 自动安装 | 让 `ev_ads_boot.sh` 开机自启 |

## 快速使用

```bash
cd /home/elf/Documents/ev_ads_cpp/deploy

# 1) 一次性装好所有依赖（中途会问 2 个 y/n）
bash install_deps.sh

# 2) 改 WiFi（用你常用的编辑器）
nano ev_ads_boot.sh
#    把开头这两行去掉注释并填上你的 SSID/PASSWORD：
#    WIFI_SSID="MyWiFiName"
#    WIFI_PASSWORD="MyWiFiPassword"

# 3) 重启，开机自启会生效
sudo reboot

# 4) 启动后查看日志
tail -f /var/log/ev_ads.log
sudo systemctl status ev-ads.service
```

## 摄像头固定路径

`install_deps.sh` 会安装 udev 规则，把 RK3588 上的三路摄像头固定为：

| 用途 | dmesg 名称 | VID:PID | 路径 |
|---|---|---|---|
| 前置 | `ocal4` | `1bcf:28c5` | `/dev/ev_ads/front_camera` |
| 后置鱼眼 | `A68-1600W` | `1bcf:2281` | `/dev/ev_ads/rear_fisheye` |
| 驾驶员人脸 | `WebCamera` | `32e6:9221` | `/dev/ev_ads/driver_face` |

手动刷新：

```bash
sudo cp /home/elf/Documents/ev_ads_cpp/deploy/udev/99-ev-ads-cameras.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=video4linux
ls -l /dev/ev_ads
```

## 手动调试（不走 systemd）

```bash
# 直接跑 boot 脚本（前台输出，方便看错）
bash ev_ads_boot.sh
```

## 时间同步如何工作

RK3588 RTC 没电时开机默认时间是 2013-01-01。本脚本：

1. 检测 `date +%s < 1700000000` (即早于 2023-11-14) → 强制重新同步
2. 先停 `chrony`，避免它拒绝大跨度时间跳变
3. 依次尝试 `ntp.aliyun.com / ntp.tencent.com / cn.pool.ntp.org / pool.ntp.org`
4. NTP 全失败时回退用 HTTPS `Date:` Header（aliyun/baidu/qq）
5. 时间正确后写回 RTC (`hwclock --systohc`)
6. 重启 `chrony` 持续守时

## 常见调整

| 想做的事 | 改哪 |
|----------|------|
| 改 WiFi | `ev_ads_boot.sh` 顶部 `WIFI_SSID` / `WIFI_PASSWORD` |
| 跑 fake 模式（无硬件） | `ev_ads_boot.sh` 底部 `LAUNCH_CMD=` 切到 `ev_ads_runtime.launch.xml use_fakes:=true` |
| 开真实制动 | `ev_ads_boot.sh` 底部 `LAUNCH_CMD=` 切到 `enable_real_brake:=true` |
| 换 ros launch | 改 `LAUNCH_CMD=(...)` |
| 关闭开机自启 | `sudo systemctl disable ev-ads.service` |
| 重启程序（不重启机器） | `sudo systemctl restart ev-ads.service` |
| 看实时日志 | `tail -f /var/log/ev_ads.log` 或 `journalctl -u ev-ads.service -f` |
