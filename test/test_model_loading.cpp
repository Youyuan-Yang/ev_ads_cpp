#include <filesystem>
#include <string>

#include <gtest/gtest.h>
#include <opencv2/dnn.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/opencv.hpp>

#include "ev_ads_runtime_cpp/onnx_yolo_detector.hpp"

using ev_ads_runtime_cpp::YoloOnnxDetector;

namespace {

const std::filesystem::path kProjectRoot = EV_ADS_PROJECT_ROOT;

std::filesystem::path model_path(const std::string& relative) {
  const auto path = kProjectRoot / relative;
  if (!std::filesystem::exists(path)) {
    ADD_FAILURE() << "模型文件不存在: " << path;
    return path;
  }
  std::error_code ec;
  const auto size = std::filesystem::file_size(path, ec);
  if (ec) {
    ADD_FAILURE() << "读取模型大小失败: " << path << " " << ec.message();
    return path;
  }
  EXPECT_GT(size, 1024u) << path;
  return path;
}

TEST(ModelLoading, LoadsYuNetFaceDetector) {
  const auto path = model_path("models/onnx/driver_face_yunet.onnx");
  auto detector = cv::FaceDetectorYN::create(path.string(), "", cv::Size(320, 320), 0.60f, 0.30f, 5000);
  ASSERT_FALSE(detector.empty());
  cv::Mat blank(320, 320, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat faces;
  detector->detect(blank, faces);
  EXPECT_EQ(faces.rows, 0);
}

void expect_yolo_loads_and_returns_empty_on_blank(const std::string& relative) {
  const auto path = model_path(relative);
  YoloOnnxDetector::Config config;
  config.model_path = path.string();
  config.input_size = cv::Size(640, 640);
  config.confidence_threshold = 0.35f;
  config.nms_threshold = 0.45f;
  config.has_objectness = false;
  std::string error;
  YoloOnnxDetector detector;
  ASSERT_TRUE(detector.load(config, &error)) << error;
  cv::Mat blank(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  const auto detections = detector.infer(blank);
  EXPECT_TRUE(detections.empty());
}

void expect_direct_opencv_loads(const std::string& relative) {
  const auto path = model_path(relative);
  auto net = cv::dnn::readNetFromONNX(path.string());
  ASSERT_FALSE(net.empty());
  const auto names = net.getUnconnectedOutLayersNames();
  EXPECT_FALSE(names.empty());
}

TEST(ModelLoading, LoadsDriverDmsYoloThroughProjectDetector) {
  expect_yolo_loads_and_returns_empty_on_blank("models/onnx/driver_dms_yolo.onnx");
}

TEST(ModelLoading, LoadsRearYoloThroughProjectDetector) {
  expect_yolo_loads_and_returns_empty_on_blank("models/onnx/rear_yolo.onnx");
}

TEST(ModelLoading, DirectOpenCvLoadsDmsAndRearOnnx) {
  expect_direct_opencv_loads("models/onnx/driver_dms_yolo.onnx");
  expect_direct_opencv_loads("models/onnx/rear_yolo.onnx");
}

}  // namespace
