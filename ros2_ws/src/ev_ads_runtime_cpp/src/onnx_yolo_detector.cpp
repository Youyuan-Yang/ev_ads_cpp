// OpenCV DNN YOLO 封装：统一解析标准 YOLO 输出和端到端 ONNX 输出。
#include "ev_ads_runtime_cpp/onnx_yolo_detector.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <numeric>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace ev_ads_runtime_cpp {
namespace {

cv::Rect2f clamp_rect(const cv::Rect2f& rect, const cv::Size& size) {
  const float x1 = std::max(0.0f, std::min(rect.x, static_cast<float>(size.width - 1)));
  const float y1 = std::max(0.0f, std::min(rect.y, static_cast<float>(size.height - 1)));
  const float x2 = std::max(0.0f, std::min(rect.x + rect.width, static_cast<float>(size.width - 1)));
  const float y2 = std::max(0.0f, std::min(rect.y + rect.height, static_cast<float>(size.height - 1)));
  if (x2 <= x1 || y2 <= y1) {
    return {};
  }
  return {x1, y1, x2 - x1, y2 - y1};
}

bool looks_channels_first(int dim1, int dim2) {
  return dim1 > 0 && dim1 <= 256 && dim2 > dim1;
}

bool looks_end_to_end_output(
    int proposals,
    int features,
    bool channels_first,
    const std::function<float(int, int)>& value_at) {
  if (features != 6 || proposals <= 0 || channels_first) {
    return false;
  }
  const int samples = std::min(proposals, 50);
  int valid = 0;
  for (int i = 0; i < samples; ++i) {
    const float score = value_at(i, 4);
    const float class_value = value_at(i, 5);
    const float class_round = std::round(class_value);
    if (score >= 0.0f && score <= 1.01f &&
        class_value >= 0.0f && class_value < 1000.0f &&
        std::abs(class_value - class_round) < 0.01f) {
      ++valid;
    }
  }
  return valid >= std::max(3, samples * 4 / 5);
}

}  // 匿名命名空间

bool YoloOnnxDetector::load(const Config& config, std::string* error) {
  config_ = config;
  try {
    net_ = cv::dnn::readNetFromONNX(config_.model_path);
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    return true;
  } catch (const cv::Exception& e) {
    if (error != nullptr) {
      *error = e.what();
    }
  } catch (const std::exception& e) {
    if (error != nullptr) {
      *error = e.what();
    }
  }
  net_ = cv::dnn::Net();
  return false;
}

bool YoloOnnxDetector::ready() const {
  return !net_.empty();
}

bool YoloOnnxDetector::class_allowed(int class_id) const {
  if (config_.class_allowlist.empty()) {
    return true;
  }
  return std::find(config_.class_allowlist.begin(), config_.class_allowlist.end(), class_id) !=
      config_.class_allowlist.end();
}

std::vector<YoloDetection> YoloOnnxDetector::infer(const cv::Mat& bgr) {
  if (!ready() || bgr.empty()) {
    return {};
  }

  const double sx = static_cast<double>(config_.input_size.width) / static_cast<double>(bgr.cols);
  const double sy = static_cast<double>(config_.input_size.height) / static_cast<double>(bgr.rows);
  const float ratio = static_cast<float>(std::min(sx, sy));
  const int resized_w = std::max(1, static_cast<int>(std::round(bgr.cols * ratio)));
  const int resized_h = std::max(1, static_cast<int>(std::round(bgr.rows * ratio)));
  const float pad_x = static_cast<float>((config_.input_size.width - resized_w) / 2.0);
  const float pad_y = static_cast<float>((config_.input_size.height - resized_h) / 2.0);

  cv::Mat resized;
  cv::resize(bgr, resized, cv::Size(resized_w, resized_h));
  cv::Mat input(config_.input_size, bgr.type(), cv::Scalar(114, 114, 114));
  resized.copyTo(input(cv::Rect(
      static_cast<int>(std::round(pad_x)),
      static_cast<int>(std::round(pad_y)),
      resized_w,
      resized_h)));

  std::vector<cv::Mat> outputs;
  try {
    cv::Mat blob = cv::dnn::blobFromImage(
        input, 1.0 / 255.0, config_.input_size, cv::Scalar(), config_.swap_rb, false, CV_32F);
    net_.setInput(blob);
    net_.forward(outputs, net_.getUnconnectedOutLayersNames());
  } catch (const cv::Exception&) {
    return {};
  }
  if (outputs.empty()) {
    return {};
  }

  std::vector<YoloDetection> candidates;
  for (const auto& output : outputs) {
    const auto parsed = parse_output(output, bgr.size(), ratio, pad_x, pad_y);
    candidates.insert(candidates.end(), parsed.begin(), parsed.end());
  }
  if (candidates.empty()) {
    return {};
  }

  std::map<int, std::vector<int>> by_class;
  for (int i = 0; i < static_cast<int>(candidates.size()); ++i) {
    by_class[candidates[i].class_id].push_back(i);
  }

  std::vector<YoloDetection> kept;
  for (const auto& item : by_class) {
    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    boxes.reserve(item.second.size());
    scores.reserve(item.second.size());
    for (int index : item.second) {
      const auto& b = candidates[index].box;
      boxes.emplace_back(
          static_cast<int>(std::round(b.x)),
          static_cast<int>(std::round(b.y)),
          static_cast<int>(std::round(b.width)),
          static_cast<int>(std::round(b.height)));
      scores.push_back(candidates[index].confidence);
    }

    std::vector<int> nms_indices;
    cv::dnn::NMSBoxes(
        boxes, scores, config_.confidence_threshold, config_.nms_threshold, nms_indices);
    for (int nms_index : nms_indices) {
      kept.push_back(candidates[item.second[nms_index]]);
    }
  }

  std::sort(kept.begin(), kept.end(), [](const YoloDetection& a, const YoloDetection& b) {
    return a.confidence > b.confidence;
  });
  return kept;
}

std::vector<YoloDetection> YoloOnnxDetector::parse_output(
    const cv::Mat& output,
    const cv::Size& original_size,
    float ratio,
    float pad_x,
    float pad_y) const {
  if (output.empty() || output.depth() != CV_32F || output.dims < 2) {
    return {};
  }

  cv::Mat contiguous = output.isContinuous() ? output : output.clone();
  const float* data = reinterpret_cast<const float*>(contiguous.data);

  int proposals = 0;
  int features = 0;
  bool channels_first = false;
  if (contiguous.dims == 3) {
    const int dim1 = contiguous.size[1];
    const int dim2 = contiguous.size[2];
    channels_first = looks_channels_first(dim1, dim2);
    features = channels_first ? dim1 : dim2;
    proposals = channels_first ? dim2 : dim1;
  } else if (contiguous.dims == 2) {
    const int dim0 = contiguous.size[0];
    const int dim1 = contiguous.size[1];
    channels_first = looks_channels_first(dim0, dim1);
    features = channels_first ? dim0 : dim1;
    proposals = channels_first ? dim1 : dim0;
  } else {
    return {};
  }

  const int min_features = config_.has_objectness ? 6 : 5;
  if (features < min_features || proposals <= 0) {
    return {};
  }

  auto value_at = [&](int proposal, int feature) {
    if (channels_first) {
      return data[feature * proposals + proposal];
    }
    return data[proposal * features + feature];
  };
  const bool end_to_end_output =
      !config_.has_objectness && looks_end_to_end_output(
          proposals, features, channels_first, value_at);

  std::vector<YoloDetection> detections;
  detections.reserve(static_cast<size_t>(proposals / 20));

  for (int i = 0; i < proposals; ++i) {
    if (end_to_end_output) {
      const float x1_raw = value_at(i, 0);
      const float y1_raw = value_at(i, 1);
      const float x2_raw = value_at(i, 2);
      const float y2_raw = value_at(i, 3);
      const float score = value_at(i, 4);
      const float class_value = value_at(i, 5);
      const int class_id = static_cast<int>(std::round(class_value));
      if (std::abs(class_value - static_cast<float>(class_id)) > 0.01f ||
          score < config_.confidence_threshold ||
          !class_allowed(class_id)) {
        continue;
      }

      const float x1 = (std::min(x1_raw, x2_raw) - pad_x) / ratio;
      const float y1 = (std::min(y1_raw, y2_raw) - pad_y) / ratio;
      const float x2 = (std::max(x1_raw, x2_raw) - pad_x) / ratio;
      const float y2 = (std::max(y1_raw, y2_raw) - pad_y) / ratio;
      cv::Rect2f box(x1, y1, x2 - x1, y2 - y1);
      box = clamp_rect(box, original_size);
      if (box.width < 2.0f || box.height < 2.0f) {
        continue;
      }
      detections.push_back({class_id, score, box});
      continue;
    }

    float cx = value_at(i, 0);
    float cy = value_at(i, 1);
    float width = value_at(i, 2);
    float height = value_at(i, 3);

    int score_start = config_.has_objectness ? 5 : 4;
    float objectness = config_.has_objectness ? value_at(i, 4) : 1.0f;

    int best_class = -1;
    float best_score = 0.0f;
    for (int j = score_start; j < features; ++j) {
      const int class_id = j - score_start;
      if (!class_allowed(class_id)) {
        continue;
      }
      const float score = objectness * value_at(i, j);
      if (score > best_score) {
        best_score = score;
        best_class = class_id;
      }
    }
    if (best_class < 0 || best_score < config_.confidence_threshold) {
      continue;
    }

    cv::Rect2f box(
        (cx - width * 0.5f - pad_x) / ratio,
        (cy - height * 0.5f - pad_y) / ratio,
        width / ratio,
        height / ratio);
    box = clamp_rect(box, original_size);
    if (box.width < 2.0f || box.height < 2.0f) {
      continue;
    }
    detections.push_back({best_class, best_score, box});
  }
  return detections;
}

}  // 命名空间 ev_ads_runtime_cpp
