// Standalone C++17 test: no ROS, PCL, sensors or chassis required.
#include "lio/livox_scan_filter.h"
#include <cassert>
#include <iostream>
#include <limits>
#include <vector>

namespace scan_filter = LI2Sup::livox_scan_filter;

struct Point {
  float x = 0, y = 0, z = 0;
  std::uint8_t tag = 0, reflectivity = 0;
};

static std::vector<Point> convert(const std::vector<Point>& input, bool enabled,
                                  scan_filter::Statistics* stats = nullptr) {
  std::vector<Point> output;
  const auto result = scan_filter::forEachPoint(
      input, input.size(), enabled, 0.25f, 100.0f,
      [&](const Point& point) { output.push_back(point); });
  if (stats) *stats = result;
  return output;
}

int main() {
  // Compose every tag from its four fields. Neither glue confidence nor the
  // reserved high bits may change acceptance. Reserved dust confidence stays.
  const bool other_allowed[] = {true, true, false, false};
  const bool dust_allowed[] = {true, false, false, true};
  unsigned checked = 0;
  for (unsigned reserved = 0; reserved < 4; ++reserved)
    for (unsigned other = 0; other < 4; ++other)
      for (unsigned dust = 0; dust < 4; ++dust)
        for (unsigned glue = 0; glue < 4; ++glue) {
          const auto tag = static_cast<std::uint8_t>(reserved * 64 + other * 16 + dust * 4 + glue);
          assert(scan_filter::acceptsTag(tag, false) == other_allowed[other]);
          assert(scan_filter::acceptsTag(tag, true) == (other_allowed[other] && dust_allowed[dust]));
          ++checked;
        }
  assert(checked == 256);

  std::vector<Point> all_tags(1);  // Historical skipped first point.
  for (unsigned tag = 0; tag < 256; ++tag)
    all_tags.push_back({1.0f + tag * 0.01f, 0, 0, static_cast<std::uint8_t>(tag),
                        static_cast<std::uint8_t>(tag)});
  scan_filter::Statistics stats;
  auto filtered = convert(all_tags, true, &stats);
  assert(filtered.size() == 64 && stats.input_points == 257);
  assert(stats.dust_rejected == 64 && stats.reserved_dust == 64);
  auto unfiltered = convert(all_tags, false, &stats);
  assert(unfiltered.size() == 128 && stats.dust_rejected == 0 && stats.reserved_dust == 64);
  for (const auto& p : filtered) assert(p.reflectivity == p.tag);

  // A nearer dust return must not hide a farther normal obstacle in the same
  // angular bin; normal near obstacles and reserved dust returns also survive.
  std::vector<Point> scene = {{}, {1, 0, 0, 0x04, 5}, {3, 0, 0, 0, 20},
                             {0, 1, 0, 0, 30}, {0, 2, 0, 0x08, 10},
                             {-2, 0, 0, 0x0c, 40}};
  filtered = convert(scene, true, &stats);
  assert(filtered.size() == 3 && stats.dust_rejected == 2 && stats.reserved_dust == 1);
  assert(filtered[0].x == 3 && filtered[1].y == 1 && filtered[2].x == -2);
  assert(convert(scene, false).size() == 5);
  assert(convert({{1, 0, 0}, {1, 0, 0}}, true).size() == 1);
  // A normal return at the exact same coordinates as filtered dust must stay.
  assert(convert({{}, {1, 0, 0, 0x04}, {1, 0, 0}}, true).size() == 1);
  assert(convert({{}, {1, 0, 0, 0x20}, {1, 0, 0}}, true).size() == 1);

  // Raw-adjacent duplicates are deterministic, even if an earlier point was
  // excluded; exact blind/max-range boundaries remain excluded.
  const std::vector<Point> boundaries = {{}, {1, 0, 0}, {1, 0, 0},
                                         {0.5f, 0, 0}, {10, 0, 0},
                                         {0.5001f, 0, 0}, {9.999f, 0, 0},
                                         {std::numeric_limits<float>::infinity(), 0, 0},
                                         {std::numeric_limits<float>::quiet_NaN(), 0, 0}};
  filtered = convert(boundaries, true);
  assert(filtered.size() == 3);
  assert(filtered[0].x == 1 && filtered[1].x == 0.5001f && filtered[2].x == 9.999f);
  assert(convert({{}, {1, 0, 0, 0x04}, {2, 0, 0, 0x08}}, true).empty());
  assert(convert({}, true).empty() && convert({{}}, true).empty());
  unsigned emitted = 0;
  const auto bounded = scan_filter::forEachPoint(scene, scene.size() + 10, true, 0.25f, 100.0f,
                                               [&](const Point&) { ++emitted; });
  assert(bounded.input_points == scene.size() && emitted == 3);
  std::cout << "Livox scan filter: all 256 tags, conversion and boundaries passed\n";
}
