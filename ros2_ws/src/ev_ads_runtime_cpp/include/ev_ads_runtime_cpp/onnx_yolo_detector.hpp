#pragma once

// YOLO ONNX 检测器接口，供后置盲区和驾驶员 DMS 复用。

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

#include <string>
#include <vector>

namespace ev_ads_runtime_cpp {

struct YoloDetection {
  int class_id{-1};
  float confidence{0.0f};
  cv::Rect2f box;
};

class YoloOnnxDetector {
 public:
  struct Config {
    std::string model_path;
    cv::Size input_size{640, 640};
    float confidence_threshold{0.35f};
    float nms_threshold{0.45f};
    bool swap_rb{true};
    bool has_objectness{false};
    std::vector<int> class_allowlist;
  };

  bool load(const Config& config, std::string* error = nullptr);
  bool ready() const;
  std::vector<YoloDetection> infer(const cv::Mat& bgr);
  const Config& config() const { return config_; }

 private:
  bool class_allowed(int class_id) const;
  std::vector<YoloDetection> parse_output(
      const cv::Mat& output,
      const cv::Size& original_size,
      float ratio,
      float pad_x,
      float pad_y) const;

  Config config_;
  cv::dnn::Net net_;
};

}  // 命名空间 ev_ads_runtime_cpp
