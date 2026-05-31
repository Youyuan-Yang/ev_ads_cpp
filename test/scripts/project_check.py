#!/usr/bin/env python3
import hashlib
import os
import pathlib
import sys
import xml.etree.ElementTree as ET


ROOT = pathlib.Path(os.environ.get("PROJECT_ROOT", pathlib.Path(__file__).resolve().parents[2])).resolve()


EXPECTED_HASHES = {
    "models/onnx/rear_yolo.onnx": "4e16b0662b3d9ceff65bc7ff79fca909f62673dc9e08aa74ee4a5e5e1511cf5d",
    "models/onnx/driver_face_yunet.onnx": "8f2383e4dd3cfbb4553ea8718107fc0423210dc964f9f4280604804ed2552fa4",
    "models/onnx/driver_dms_yolo.onnx": "cca6dffd84d0596e8ca620dd7ba91dd2624b29fde740bc76d420a5f74e908187",
}


XML_FILES = [
    "config/ev_ads_runtime.launch.xml",
    "config/scenarios/driver_drowsy.xml",
    "config/scenarios/front_pedestrian_emergency.xml",
    "config/scenarios/front_sensor_lost.xml",
    "config/scenarios/imu_lean_in_curve.xml",
    "config/scenarios/mmwave_abnormal_low_confidence.xml",
    "config/scenarios/rear_right_blindspot_turn.xml",
    "ros2_ws/src/ev_ads_runtime_cpp/package.xml",
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


def check_no_mixed_project_config() -> None:
    ignored_parts = {"build", "install", "log", "__pycache__", ".git"}
    mixed = []
    for suffix in ("*.yaml", "*.yml", "*.toml"):
        for path in ROOT.rglob(suffix):
            rel = path.relative_to(ROOT)
            if set(rel.parts) & ignored_parts:
                continue
            mixed.append(str(rel))
    if mixed:
        fail("项目自有配置必须统一为 XML，发现混用文件: " + ", ".join(sorted(mixed)))

    misplaced_xml = []
    for path in (ROOT / "ros2_ws/src").rglob("*.xml"):
        rel = path.relative_to(ROOT).as_posix()
        if not rel.endswith("package.xml"):
            misplaced_xml.append(rel)
    if misplaced_xml:
        fail("运行/场景 XML 必须放在根目录 config/: " + ", ".join(sorted(misplaced_xml)))


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
    runtime_launch = read_text("config/ev_ads_runtime.launch.xml")
    for required in [
        'name="rear_model_path" default="$(find-pkg-share ev_ads_runtime_cpp)/models/onnx/rear_yolo.onnx"',
        'name="driver_model_path" default="$(find-pkg-share ev_ads_runtime_cpp)/models/onnx/driver_dms_yolo.onnx"',
        'name="driver_face_model_path" default="$(find-pkg-share ev_ads_runtime_cpp)/models/onnx/driver_face_yunet.onnx"',
        'name="driver_face_model_path"',
        'name="front_camera_device" default="/dev/ev_ads/front_camera"',
        'name="rear_camera_device" default="/dev/ev_ads/rear_fisheye"',
        'name="driver_camera_device" default="/dev/ev_ads/driver_face"',
        'name="event_storage_backend" default="sqlite"',
        'name="event_log_path" default="/tmp/ev_ads/events.sqlite"',
        'name="storage_backend" value="$(var event_storage_backend)"',
        'name="face_model_path" value="$(var driver_face_model_path)"',
        'name="fisheye_undistort"',
        'name="model_class_ids" value="[0, 1, 2, 3, 5, 7]"',
        'name="w_front" value="$(var w_front)"',
    ]:
        if required not in runtime_launch:
            fail(f"runtime launch 缺少配置: {required}")
    if "<param from=" in runtime_launch or "fusion_config" in runtime_launch:
        fail("runtime launch 仍依赖外部配置文件")

    if 'default=""' in runtime_launch:
        fail("根配置仍有空默认路径")
    if 'type="double"' in runtime_launch:
        fail('ROS2 Humble XML launch 不支持 type="double"，浮点参数必须用 type="float"')
    if "：" in runtime_launch:
        fail("runtime launch 不应包含全角冒号，避免 XML 失败后 fallback 解析器报干扰性 SyntaxError")


def check_runtime_sources() -> None:
    required_files = [
        "CMakeLists.txt",
        "test/CMakeLists.txt",
        "test/test_risk_math_and_fusion.cpp",
        "test/test_event_store.cpp",
        "test/test_model_loading.cpp",
        "deploy/udev/99-ev-ads-cameras.rules",
        "config/ev_ads_runtime.launch.xml",
        "config/scenarios/front_pedestrian_emergency.xml",
        "ros2_ws/src/ev_ads_runtime_cpp/include/ev_ads_runtime_cpp/domain_types.hpp",
        "ros2_ws/src/ev_ads_runtime_cpp/include/ev_ads_runtime_cpp/topics.hpp",
        "ros2_ws/src/ev_ads_runtime_cpp/include/ev_ads_runtime_cpp/runtime_config.hpp",
        "ros2_ws/src/ev_ads_runtime_cpp/include/ev_ads_runtime_cpp/risk_fusion_core.hpp",
        "ros2_ws/src/ev_ads_runtime_cpp/include/ev_ads_runtime_cpp/event_store.hpp",
        "ros2_ws/src/ev_ads_runtime_cpp/msg/FrontRisk.msg",
        "ros2_ws/src/ev_ads_runtime_cpp/msg/RiskState.msg",
        "ros2_ws/src/ev_ads_runtime_cpp/src/event_store.cpp",
    ]
    for rel in required_files:
        if not (ROOT / rel).exists():
            fail(f"缺少项目级测试/核心文件: {rel}")

    root_cmake = read_text("CMakeLists.txt")
    for needle in [
        "project(ev_ads_runtime",
        "EV_ADS_BUILD_ROS2_NATIVE",
        "EV_ADS_RUNTIME_PACKAGE_DIR",
        "add_subdirectory(\"${EV_ADS_RUNTIME_PACKAGE_DIR}\"",
        "file(GENERATE OUTPUT \"${EV_ADS_COLCON_BUILD_SCRIPT}\"",
        "${CMAKE_COMMAND} -E env",
        "/bin/bash \"${EV_ADS_COLCON_BUILD_SCRIPT}\"",
        "ros2_workspace_build",
        "run_ev_ads_fake",
        "run_ev_ads_hardware",
        "include(FetchContent)",
        "FetchContent_Declare(",
        "googletest",
        "FetchContent_MakeAvailable(googletest)",
        "include(GoogleTest)",
        "add_subdirectory(test)",
    ]:
        if needle not in root_cmake:
            fail(f"根 CMake 未配置真实项目入口或 GoogleTest: {needle}")
    if "ev_ads_project_tests" in root_cmake:
        fail("根 CMake 仍是测试工程命名，必须是 EV-ADS runtime 总入口")
    if "/bin/bash -lc" in root_cmake:
        fail("根 CMake 不允许把多行 shell 直接塞进 /bin/bash -lc，Makefile 会拆坏换行")

    test_cmake = read_text("test/CMakeLists.txt")
    for needle in [
        "GTest::gtest_main",
        "gtest_discover_tests(test_risk_math_and_fusion)",
        "gtest_discover_tests(test_event_store)",
        "gtest_discover_tests(test_model_loading)",
        "EV_ADS_PROJECT_ROOT",
    ]:
        if needle not in test_cmake:
            fail(f"测试 CMake 未按 GTest 配置: {needle}")

    for rel in [
        "test/test_risk_math_and_fusion.cpp",
        "test/test_event_store.cpp",
        "test/test_model_loading.cpp",
    ]:
        source = read_text(rel)
        if "#include <gtest/gtest.h>" not in source or "TEST(" not in source:
            fail(f"C++ 测试未改为 GoogleTest 写法: {rel}")
        if "#include <cassert>" in source or "assert(" in source:
            fail(f"C++ 测试仍包含 assert 写法: {rel}")

    cmake = read_text("ros2_ws/src/ev_ads_runtime_cpp/CMakeLists.txt")
    for needle in [
        "find_package(SQLite3 REQUIRED)",
        "rosidl_generate_interfaces",
        "add_library(event_store",
        "target_link_libraries(event_recorder_node",
        "../../../config/ev_ads_runtime.launch.xml",
        "DESTINATION share/${PROJECT_NAME}/launch",
        "../../../models/onnx/rear_yolo.onnx",
        "../../../models/onnx/driver_face_yunet.onnx",
        "../../../models/onnx/driver_dms_yolo.onnx",
        "DESTINATION share/${PROJECT_NAME}/models/onnx",
    ]:
        if needle not in cmake:
            fail(f"runtime CMake 未配置: {needle}")
    if "install(DIRECTORY config" in cmake:
        fail("runtime CMake 仍安装 YAML config 目录")
    if "find_package(ev_ads_interfaces" in cmake:
        fail("runtime CMake 仍依赖独立 interfaces 包")

    for exe in [
        "front_risk_node",
        "rear_blind_spot_node",
        "driver_attention_node",
        "risk_fusion_node",
        "imu_motion_node",
        "camera_capture_node",
        "mmwave_vital_node",
        "terminal_hmi_node",
        "event_recorder_node",
    ]:
        if f"add_executable({exe}" not in cmake or exe not in cmake.split("install(TARGETS", 1)[-1]:
            fail(f"runtime CMake 未完整编译/安装节点: {exe}")

    packages = sorted(path.relative_to(ROOT).as_posix() for path in (ROOT / "ros2_ws/src").glob("*/package.xml"))
    if packages != ["ros2_ws/src/ev_ads_runtime_cpp/package.xml"]:
        fail("ros2_ws/src 必须只保留一个 ROS2 包: " + ", ".join(packages))

    package_xml = read_text("ros2_ws/src/ev_ads_runtime_cpp/package.xml")
    if "<depend>sqlite3</depend>" not in package_xml:
        fail("package.xml 缺少 sqlite3 依赖")
    if "ev_ads_interfaces" in package_xml:
        fail("package.xml 仍依赖独立 interfaces 包")

    fusion_node = read_text("ros2_ws/src/ev_ads_runtime_cpp/src/risk_fusion_node.cpp")
    if "FusionCore core_" not in fusion_node:
        fail("risk_fusion_node 未使用 FusionCore")

    driver_node = read_text("ros2_ws/src/ev_ads_runtime_cpp/src/driver_attention_node.cpp")
    if "DriverMonitorConfig config_" not in driver_node or "RuntimeTopics topics_" not in driver_node:
        fail("driver_attention_node 未收敛到配置对象/统一话题")

    event_node = read_text("ros2_ws/src/ev_ads_runtime_cpp/src/event_recorder_node.cpp")
    if "EventStore store_" not in event_node or "storage_backend" not in event_node:
        fail("event_recorder_node 未使用 EventStore 或存储后端参数")

    udev_rules = read_text("deploy/udev/99-ev-ads-cameras.rules")
    for needle in [
        'ATTRS{idVendor}=="1bcf", ATTRS{idProduct}=="2281"',
        'SYMLINK+="ev_ads/rear_fisheye"',
        'ATTRS{idVendor}=="32e6", ATTRS{idProduct}=="9221"',
        'SYMLINK+="ev_ads/driver_face"',
        'ATTRS{idVendor}=="1bcf", ATTRS{idProduct}=="28c5"',
        'SYMLINK+="ev_ads/front_camera"',
    ]:
        if needle not in udev_rules:
            fail(f"摄像头 udev 规则缺少: {needle}")

    install_deps = read_text("deploy/install_deps.sh")
    for needle in ["99-ev-ads-cameras.rules", "udevadm control --reload-rules", "udevadm trigger"]:
        if needle not in install_deps:
            fail(f"install_deps.sh 未安装摄像头 udev 规则: {needle}")


def check_docs() -> None:
    readme = read_text("README.md")
    if "cmake -S . -B build/mac" not in readme:
        fail("README 未写根目录统一测试命令")
    if "GoogleTest" not in readme or "FetchContent" not in readme:
        fail("README 未说明根 CMake 自动下载 GoogleTest")
    for needle in ["EV_ADS_BUILD_ROS2_NATIVE", "EV_ADS_BUILD_TESTS", "ros2_workspace_build", "run_ev_ads_hardware"]:
        if needle not in readme:
            fail(f"README 未说明根 CMake 真实项目入口: {needle}")
    if "/opt/ev_ads" in readme:
        fail("README 仍使用旧 RK3588 路径 /opt/ev_ads，应改为 /home/elf/Documents/ev_ads_cpp")
    if "SQLite/WAL" not in readme:
        fail("README 未说明 SQLite/WAL 事件记录")
    if "XML/YAML" in readme or ".yaml" in readme or ".toml" in readme:
        fail("README 仍描述多配置格式")

    test_plan = read_text("docs/test_plan.md")
    for needle in ["cmake -S . -B build/mac", "ros2_workspace_build", "events.sqlite", "GoogleTest", "ModelLoading", "XML-only"]:
        if needle not in test_plan:
            fail(f"测试计划未覆盖: {needle}")

    for rel in [
        "docs/deploy_rk3588.md",
        "docs/test_plan.md",
        "docs/model_selection_and_onnx_plan.md",
        "deploy/README.md",
    ]:
        text = read_text(rel)
        if "/opt/ev_ads" in text:
            fail(f"{rel} 仍使用旧 RK3588 路径 /opt/ev_ads")
        if "/home/elf/Documents/ev_ads_cpp" not in text:
            fail(f"{rel} 未写入当前 RK3588 项目路径 /home/elf/Documents/ev_ads_cpp")


def main() -> None:
    check_no_runtime_python()
    check_xml()
    check_no_mixed_project_config()
    check_model_files()
    check_launch_configuration()
    check_runtime_sources()
    check_docs()
    print("project_check ok")


if __name__ == "__main__":
    main()
