#!/usr/bin/env python3
import hashlib
import os
import pathlib
import subprocess
import sys
import xml.etree.ElementTree as ET


ROOT = pathlib.Path(os.environ.get("PROJECT_ROOT", pathlib.Path(__file__).resolve().parents[2])).resolve()


EXPECTED_HASHES = {
    "models/onnx/rear_yolo.onnx": "4e16b0662b3d9ceff65bc7ff79fca909f62673dc9e08aa74ee4a5e5e1511cf5d",
    "models/onnx/driver_face_yunet.onnx": "8f2383e4dd3cfbb4553ea8718107fc0423210dc964f9f4280604804ed2552fa4",
    "models/onnx/driver_dms_yolo.onnx": "cca6dffd84d0596e8ca620dd7ba91dd2624b29fde740bc76d420a5f74e908187",
}


XML_FILES = [
    "ros2_ws/src/ev_ads_runtime_cpp/launch/cpp_runtime.launch.xml",
    "ros2_ws/src/ev_ads_bringup/launch/ev_ads_cpp_runtime.launch.xml",
    "ros2_ws/src/ev_ads_bringup/launch/ev_ads_demo.launch.xml",
    "ros2_ws/src/ev_ads_runtime_cpp/package.xml",
    "ros2_ws/src/ev_ads_bringup/package.xml",
]


YAML_FILES = [
    "ros2_ws/src/ev_ads_runtime_cpp/config/cameras.yaml",
    "ros2_ws/src/ev_ads_runtime_cpp/config/driver_monitor.yaml",
    "ros2_ws/src/ev_ads_runtime_cpp/config/fusion_urban_day.yaml",
    "ros2_ws/src/ev_ads_runtime_cpp/config/fusion_night.yaml",
    "ros2_ws/src/ev_ads_runtime_cpp/config/fusion_long_ride.yaml",
    "ros2_ws/src/ev_ads_runtime_cpp/config/imu.yaml",
    "ros2_ws/src/ev_ads_runtime_cpp/config/rear_fisheye.yaml",
]


def fail(message: str) -> None:
    print(f"[FAIL] {message}", file=sys.stderr)
    raise SystemExit(1)


def read_text(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def sha256(path: pathlib.Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def check_no_runtime_python() -> None:
    allowed_prefixes = {"test"}
    ignored_parts = {"build", "install", "log", "__pycache__", ".git"}
    found = []
    for path in ROOT.rglob("*.py"):
        rel = path.relative_to(ROOT)
        parts = set(rel.parts)
        if parts & ignored_parts:
            continue
        if rel.parts and rel.parts[0] in allowed_prefixes:
            continue
        found.append(str(rel))
    if found:
        fail("发现非测试 Python 文件: " + ", ".join(found))


def check_xml() -> None:
    for rel in XML_FILES:
        ET.parse(ROOT / rel)


def check_yaml() -> None:
    missing = [rel for rel in YAML_FILES if not (ROOT / rel).exists()]
    if missing:
        fail("缺少 YAML 配置: " + ", ".join(missing))
    code = "require 'yaml'; ARGV.each { |f| YAML.load_file(f) }; puts 'yaml_ok'"
    cmd = ["ruby", "-e", code] + [str(ROOT / rel) for rel in YAML_FILES]
    result = subprocess.run(cmd, text=True, capture_output=True)
    if result.returncode != 0:
        fail("YAML 解析失败: " + result.stderr.strip())


def check_model_files() -> None:
    for rel, expected in EXPECTED_HASHES.items():
      path = ROOT / rel
      if not path.exists():
          fail(f"模型不存在: {rel}")
      if path.stat().st_size <= 1024:
          fail(f"模型文件异常过小: {rel}")
      actual = sha256(path)
      if actual != expected:
          fail(f"模型 hash 不匹配: {rel} expected={expected} actual={actual}")


def check_launch_configuration() -> None:
    runtime_launch = read_text("ros2_ws/src/ev_ads_runtime_cpp/launch/cpp_runtime.launch.xml")
    for required in [
        'name="driver_face_model_path"',
        'name="event_storage_backend" default="sqlite"',
        'name="event_log_path" default="/tmp/ev_ads/events.sqlite"',
        'name="storage_backend" value="$(var event_storage_backend)"',
        'name="face_model_path" value="$(var driver_face_model_path)"',
    ]:
        if required not in runtime_launch:
            fail(f"runtime launch 缺少配置: {required}")

    for rel in [
        "ros2_ws/src/ev_ads_bringup/launch/ev_ads_cpp_runtime.launch.xml",
        "ros2_ws/src/ev_ads_bringup/launch/ev_ads_demo.launch.xml",
    ]:
        text = read_text(rel)
        for required in ["driver_face_model_path", "event_storage_backend", "events.sqlite"]:
            if required not in text:
                fail(f"{rel} 未转发配置: {required}")


def check_runtime_sources() -> None:
    required_files = [
        "CMakeLists.txt",
        "test/CMakeLists.txt",
        "test/test_common_and_fusion.cpp",
        "test/test_event_store.cpp",
        "test/test_model_loading.cpp",
        "ros2_ws/src/ev_ads_runtime_cpp/include/ev_ads_runtime_cpp/types.hpp",
        "ros2_ws/src/ev_ads_runtime_cpp/include/ev_ads_runtime_cpp/topics.hpp",
        "ros2_ws/src/ev_ads_runtime_cpp/include/ev_ads_runtime_cpp/runtime_config.hpp",
        "ros2_ws/src/ev_ads_runtime_cpp/include/ev_ads_runtime_cpp/fusion_core.hpp",
        "ros2_ws/src/ev_ads_runtime_cpp/include/ev_ads_runtime_cpp/event_store.hpp",
        "ros2_ws/src/ev_ads_runtime_cpp/src/event_store.cpp",
    ]
    for rel in required_files:
        if not (ROOT / rel).exists():
            fail(f"缺少项目级测试/核心文件: {rel}")

    cmake = read_text("ros2_ws/src/ev_ads_runtime_cpp/CMakeLists.txt")
    for needle in ["find_package(SQLite3 REQUIRED)", "add_library(event_store", "target_link_libraries(event_logger_node_cpp"]:
        if needle not in cmake:
            fail(f"runtime CMake 未配置: {needle}")

    package_xml = read_text("ros2_ws/src/ev_ads_runtime_cpp/package.xml")
    if "<depend>sqlite3</depend>" not in package_xml:
        fail("package.xml 缺少 sqlite3 依赖")

    fusion_node = read_text("ros2_ws/src/ev_ads_runtime_cpp/src/fusion_node_cpp.cpp")
    if "FusionCore core_" not in fusion_node:
        fail("fusion_node_cpp 未使用 FusionCore")

    driver_node = read_text("ros2_ws/src/ev_ads_runtime_cpp/src/driver_monitor_node_cpp.cpp")
    if "DriverMonitorConfig config_" not in driver_node or "RuntimeTopics topics_" not in driver_node:
        fail("driver_monitor_node_cpp 未收敛到配置对象/统一话题")

    event_node = read_text("ros2_ws/src/ev_ads_runtime_cpp/src/event_logger_node_cpp.cpp")
    if "EventStore store_" not in event_node or "storage_backend" not in event_node:
        fail("event_logger_node_cpp 未使用 EventStore 或存储后端参数")


def check_docs() -> None:
    readme = read_text("README.md")
    if "cmake -S . -B build/mac" not in readme:
        fail("README 未写根目录统一测试命令")
    if "SQLite/WAL" not in readme:
        fail("README 未说明 SQLite/WAL 事件记录")

    test_plan = read_text("docs/test_plan.md")
    for needle in ["cmake -S . -B build/mac", "events.sqlite", "model_loading"]:
        if needle not in test_plan:
            fail(f"测试计划未覆盖: {needle}")


def main() -> None:
    check_no_runtime_python()
    check_xml()
    check_yaml()
    check_model_files()
    check_launch_configuration()
    check_runtime_sources()
    check_docs()
    print("project_check ok")


if __name__ == "__main__":
    main()
