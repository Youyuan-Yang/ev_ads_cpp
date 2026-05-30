#include <cassert>
#include <filesystem>
#include <iostream>
#include <string>

#include <opencv2/dnn.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/opencv.hpp>

#include "ev_ads_runtime_cpp/yolo_onnx.hpp"

using ev_ads_runtime_cpp::YoloOnnxDetector;

namespace {

std::filesystem::path require_file(const std::filesystem::path& root, const std::string& relative) {
  const auto path = root / relative;
  assert(std::filesystem::exists(path));
  assert(std::filesystem::file_size(path) > 1024);
  return path;
}

void test_yunet(const std::filesystem::path& root) {
  const auto path = require_file(root, "models/onnx/driver_face_yunet.onnx");
  auto detector = cv::FaceDetectorYN::create(path.string(), "", cv::Size(320, 320), 0.60f, 0.30f, 5000);
  cv::Mat blank(320, 320, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat faces;
  detector->detect(blank, faces);
  assert(faces.rows == 0);
}

void test_yolo_model(const std::filesystem::path& root, const std::string& relative) {
  const auto path = require_file(root, relative);
  YoloOnnxDetector::Config config;
  config.model_path = path.string();
  config.input_size = cv::Size(640, 640);
  config.confidence_threshold = 0.35f;
  config.nms_threshold = 0.45f;
  config.has_objectness = false;
  std::string error;
  YoloOnnxDetector detector;
  assert(detector.load(config, &error));
  cv::Mat blank(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  const auto detections = detector.infer(blank);
  assert(detections.empty());
}

void test_direct_opencv_load(const std::filesystem::path& root, const std::string& relative) {
  const auto path = require_file(root, relative);
  auto net = cv::dnn::readNetFromONNX(path.string());
  assert(!net.empty());
  const auto names = net.getUnconnectedOutLayersNames();
  assert(!names.empty());
}

}  // namespace

int main(int argc, char** argv) {
  assert(argc == 2);
  const std::filesystem::path root(argv[1]);
  test_yunet(root);
  test_yolo_model(root, "models/onnx/driver_dms_yolo.onnx");
  test_yolo_model(root, "models/onnx/rear_yolo.onnx");
  test_direct_opencv_load(root, "models/onnx/driver_dms_yolo.onnx");
  test_direct_opencv_load(root, "models/onnx/rear_yolo.onnx");
  std::cout << "test_model_loading ok\n";
  return 0;
}
