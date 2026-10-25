#pragma once
#include <cmath>
#include <vector>
#include <cstdint>
#include <algorithm>

namespace regulated_pure_pursuit_controller {
struct ScanXY { double x, y; };
// Only call for a valid, transformed, fresh scan. Repeated scans never count twice.
inline bool advanceClearScan(bool& latched, std::uint64_t& last, int& confirm, int& clear,
    std::uint64_t sequence, std::size_t hits, int minimum_hits, int required) {
  if (!latched || sequence == last) return false;
  last=sequence; confirm=0;
  if (hits >= static_cast<std::size_t>(std::max(1,minimum_hits))) { clear=0; return false; }
  if (++clear < std::max(1,required)) return false;
  latched=false; clear=0; return true;
}

inline bool freshScanAge(double receive_age, double sample_age, double timeout) {
  return std::isfinite(receive_age) && std::isfinite(sample_age) &&
    receive_age >= 0 && receive_age <= timeout && sample_age >= -timeout && sample_age <= timeout;
}

inline bool scanCollisionEnabled(bool fixed, bool footprint) { return fixed || footprint; }
enum class ScanZone { outside, front, side, body, self };

// Boundary points are retained: a filter must not hide a touching obstacle.
inline bool strictlyInside(const std::vector<ScanXY>& polygon, double x, double y) {
  if (polygon.size() < 3) return false;
  bool inside = false;
  for (std::size_t i=0, j=polygon.size()-1; i<polygon.size(); j=i++) {
    const auto& a=polygon[i]; const auto& b=polygon[j];
    const double cross=(x-a.x)*(b.y-a.y)-(y-a.y)*(b.x-a.x);
    if (std::abs(cross)<1e-9 && x>=std::fmin(a.x,b.x) && x<=std::fmax(a.x,b.x) &&
        y>=std::fmin(a.y,b.y) && y<=std::fmax(a.y,b.y)) return false;
    if ((a.y>y)!=(b.y>y) && x<(b.x-a.x)*(y-a.y)/(b.y-a.y)+a.x) inside=!inside;
  }
  return inside;
}

inline ScanZone classifyScan(double x, double y, double min_x, double max_x,
    double min_y, double max_y, double front, double side,
    const std::vector<ScanXY>& self_polygon) {
  if (!std::isfinite(x) || !std::isfinite(y)) return ScanZone::outside;
  if (strictlyInside(self_polygon,x,y)) return ScanZone::self;
  if (x<min_x-side || x>max_x+front || y<min_y-side || y>max_y+side) return ScanZone::outside;
  if (x>max_x) return ScanZone::front;
  if (x>=min_x && y>=min_y && y<=max_y) return ScanZone::body;
  return ScanZone::side;
}
}
