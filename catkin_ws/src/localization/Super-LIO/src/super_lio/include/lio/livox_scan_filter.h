#ifndef SUPER_LIO_LIVOX_SCAN_FILTER_H_
#define SUPER_LIO_LIVOX_SCAN_FILTER_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace LI2Sup {
namespace livox_scan_filter {

// MID-360 series protocol, section 1.2.1.3.4: 0 = good, 1 = medium,
// 2 = poor, 3 = reserved. These are confidence fields, not return numbers.
inline unsigned dustConfidence(std::uint8_t tag) {
  return (tag >> 2) & 0x03;
}

inline bool acceptsTag(std::uint8_t tag, bool filter_dust) {
  const auto other = (tag >> 4) & 0x03;
  const auto dust = dustConfidence(tag);
  return other <= 1 && !(filter_dust && (dust == 1 || dust == 2));
}

struct Statistics {
  std::size_t input_points = 0;
  // Scan candidates rejected by tag, before duplicate/range/height checks.
  std::size_t dust_rejected = 0;
  // All input points carrying reserved dust confidence, including other rejects.
  std::size_t reserved_dust = 0;
};

// Scan-only traversal. Read the immutable message rather than a partially
// populated cloud; no shared-bit flags or adjacent concurrent reads/writes.
// Retain the historical omission of index 0. Compare raw-adjacent candidates,
// but do not let an omitted/filtered predecessor erase a normal return.
template <typename Points, typename Emit>
Statistics forEachPoint(const Points& points, std::size_t point_count,
                        bool filter_dust, float blind_squared,
                        float max_range_squared, Emit emit) {
  Statistics stats;
  stats.input_points = std::min(point_count, points.size());
  for (std::size_t i = 0; i < stats.input_points; ++i) {
    const auto& point = points[i];
    const auto dust = dustConfidence(point.tag);
    if (dust == 3) ++stats.reserved_dust;
    if (i == 0 || !acceptsTag(point.tag, false)) continue;
    if (!acceptsTag(point.tag, filter_dust)) {
      ++stats.dust_rejected;
      continue;
    }
    const auto& previous = points[i - 1];
    if (i > 1 && acceptsTag(previous.tag, filter_dust) &&
        !(std::abs(point.x - previous.x) > 1e-7 ||
          std::abs(point.y - previous.y) > 1e-7 ||
          std::abs(point.z - previous.z) > 1e-7)) continue;
    const double distance_squared = point.x * point.x + point.y * point.y + point.z * point.z;
    if (distance_squared > blind_squared && distance_squared < max_range_squared) emit(point);
  }
  return stats;
}

}  // namespace livox_scan_filter
}  // namespace LI2Sup

#endif
