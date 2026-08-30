#pragma once

#include <stair_perception/StairObservation.h>
#include <stair_perception/StairTrack.h>

#include <algorithm>
#include <cmath>

namespace stair_perception {

struct ExtentProposal {
  bool valid{false};
  StairObservation observation;
  int hits{0};
};

inline void fuseObservation(double alpha, const StairObservation& o, StairTrack* t) {
  auto mix = [alpha](double old_value, double new_value) {
    return (1.0 - alpha) * old_value + alpha * new_value;
  };

  // Flight endpoints are fused separately by considerExtent(); ordinary partial views may only
  // refine orientation/appearance and can never shrink an already observed flight.
  t->heading.x = mix(t->heading.x, o.heading.x);
  t->heading.y = mix(t->heading.y, o.heading.y);
  const double norm = std::hypot(t->heading.x, t->heading.y);
  if (norm > 1e-6) {
    t->heading.x /= norm;
    t->heading.y /= norm;
  }
  t->slope = mix(t->slope, o.slope);
  t->width = mix(t->width, o.width);
  t->confidence = mix(t->confidence, o.confidence);
}

inline double planarDistance(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

inline bool considerExtent(const StairObservation& o, StairTrack* t, ExtentProposal* proposal) {
  constexpr float kMinConfidence = 0.45F;
  constexpr float kMinExtension = 0.05F;
  if (o.confidence < kMinConfidence || o.rise <= t->rise + kMinExtension) return false;

  const bool matches = proposal->valid &&
      planarDistance(o.entry_pose.position, proposal->observation.entry_pose.position) < 0.8 &&
      planarDistance(o.exit_pose.position, proposal->observation.exit_pose.position) < 0.8 &&
      std::abs(o.rise - proposal->observation.rise) < 0.5F;
  if (!matches) {
    proposal->valid = true;
    proposal->observation = o;
    proposal->hits = 1;
    return false;
  }

  ++proposal->hits;
  if (proposal->hits < 2) return false;
  const StairObservation& best = o.rise >= proposal->observation.rise ? o : proposal->observation;
  t->entry_pose = best.entry_pose;
  t->exit_pose = best.exit_pose;
  t->rise = best.rise;
  *proposal = ExtentProposal();
  return true;
}

}  // namespace stair_perception
