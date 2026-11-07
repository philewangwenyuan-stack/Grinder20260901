// Geometry and stop evidence shared by the opt-in controller and offline tests.
#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace regulated_pure_pursuit_controller {
namespace line_corner {
struct XY { double x=0, y=0; };
inline XY operator-(XY a, XY b) { return {a.x-b.x,a.y-b.y}; }
inline XY operator+(XY a, XY b) { return {a.x+b.x,a.y+b.y}; }
inline XY operator*(XY a, double s) { return {a.x*s,a.y*s}; }
inline double dot(XY a, XY b) { return a.x*b.x+a.y*b.y; }
inline double norm(XY a) { return std::hypot(a.x,a.y); }
inline double angle(XY a) { return std::atan2(a.y,a.x); }
inline double wrap(double a) { return std::atan2(std::sin(a),std::cos(a)); }
struct Corner { XY point, incoming, outgoing; std::size_t vertex=0; };
enum class Phase { Approach, Brake, Rotate };

// Time and message identity both matter: polling a 10 Hz sample at 20 Hz must
// not count as two independent confirmations. A bad/moving sample resets it.
class StopEvidence {
 public:
  void reset() { first_=last_=0; count_=0; }
  bool update(double now, double stamp, bool stopped, double duration) {
    if (!stopped || !std::isfinite(now) || stamp<=0 || stamp>now) {
      reset(); return false;
    }
    if (count_ && stamp<last_) reset();
    if (!count_) { first_=now; last_=stamp; count_=1; }
    else if (stamp>last_) { last_=stamp; ++count_; }
    return count_>=2 && now-first_+1e-9>=duration;
  }
 private:
  double first_=0,last_=0;
  unsigned count_=0;
};
inline bool fresh(double now,double stamp,double timeout,double vx,double vy,double wz) {
  return std::isfinite(now) && std::isfinite(stamp) && stamp>0 && stamp<=now &&
    now-stamp<=timeout && std::isfinite(vx) && std::isfinite(vy) && std::isfinite(wz);
}
class Route {
 public:
  void set(const std::vector<XY>& source,double min_angle) {
    points_.clear(); corners_.clear(); next_=0; acquired_=false; phase=Phase::Approach;
    for (auto p:source) {
      if (!std::isfinite(p.x)||!std::isfinite(p.y)) { points_.clear(); return; }
      if (points_.empty() || norm(p-points_.back())>1e-6) points_.push_back(p);
    }
    for (std::size_t i=1;i+1<points_.size();++i) {
      auto a=points_[i]-points_[i-1], b=points_[i+1]-points_[i];
      a=a*(1/norm(a)); b=b*(1/norm(b));
      if (std::abs(wrap(angle(b)-angle(a)))>min_angle)
        corners_.push_back({points_[i],a,b,i});
    }
  }
  void reacquire() { acquired_=false; phase=Phase::Approach; }
  const Corner* current(XY robot) {
    if (!acquired_) {
      double best=std::numeric_limits<double>::infinity(); std::size_t segment=0;
      for (std::size_t i=0;i+1<points_.size();++i) {
        auto d=points_[i+1]-points_[i];
        auto q=points_[i]+d*std::clamp(dot(robot-points_[i],d)/dot(d,d),0.0,1.0);
        const auto distance=norm(robot-q);
        if (distance<best) { best=distance; segment=i; }
      }
      next_=0;
      while (next_<corners_.size() && corners_[next_].vertex<=segment) ++next_;
      acquired_=true;
    }
    return next_<corners_.size()? &corners_[next_]:nullptr;
  }
  void complete() { if (next_<corners_.size()) ++next_; phase=Phase::Approach; }
  double remaining(const Corner& c,XY p) const { return dot(c.point-p,c.incoming); }
  XY carrot(const Corner& c,XY p,double lookahead) const {
    const auto projection=c.point+c.incoming*dot(p-c.point,c.incoming);
    return projection+c.incoming*std::max(0.01,lookahead);
  }
  Phase phase=Phase::Approach;
 private:
  std::vector<XY> points_;
  std::vector<Corner> corners_;
  std::size_t next_=0;
  bool acquired_=false;
};
inline double approachSpeed(double desired,double remaining,double distance,double floor) {
  return std::min(desired,std::max(floor,desired*std::clamp(remaining/distance,0.0,1.0)));
}
} // namespace line_corner
} // namespace regulated_pure_pursuit_controller
