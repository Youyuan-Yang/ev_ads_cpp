#include "radar_payload.h"
#include "project_config.h"

// 用 snprintf 生成紧凑 JSON，避免动态库依赖。
// 字段顺序与文档 docs/ble_protocol.md 保持一致。
size_t radar_payload_to_json(const RadarSample& s, char* out, size_t out_size) {
  if (out == nullptr || out_size == 0) return 0;
  int n = snprintf(
      out, out_size,
      "{\"v\":1,\"seq\":%lu,\"t\":%lu,\"br\":%.2f,\"hr\":%.2f,\"d\":%.2f,\"st\":%u}",
      (unsigned long)s.seq,
      (unsigned long)s.timestamp_ms,
      s.breath_rate,
      s.heart_rate,
      s.distance_cm,
      (unsigned)s.status);
  if (n < 0) return 0;
  return (size_t)((size_t)n < out_size ? (size_t)n : out_size - 1);
}
