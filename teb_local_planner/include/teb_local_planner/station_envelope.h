#ifndef TEB_LOCAL_PLANNER_STATION_ENVELOPE_H_
#define TEB_LOCAL_PLANNER_STATION_ENVELOPE_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

namespace teb_local_planner {

// Station tracking reference and reporting allowance. Hull collision checking
// remains independent of the desired position tolerance.
struct StationEnvelope {
  bool enabled = false;
  std::string request_id;
  std::string route_digest;
  std::string frame;
  std::uint64_t generation = 0;
  double center_x = 0.0;
  double center_y = 0.0;
  double radius = 0.0;
  double surge_limit = 0.04;
  double command_horizon = 0.75;

  bool valid() const {
    return !enabled || (!request_id.empty() && !route_digest.empty() &&
        !frame.empty() && generation > 0 && std::isfinite(center_x) &&
        std::isfinite(center_y) && std::isfinite(radius) && radius > 0.0 &&
        std::isfinite(surge_limit) && surge_limit >= 0.01 &&
        std::isfinite(command_horizon) && command_horizon > 0.0);
  }

  bool operator==(const StationEnvelope& other) const {
    return enabled == other.enabled && request_id == other.request_id &&
        route_digest == other.route_digest && frame == other.frame &&
        generation == other.generation && center_x == other.center_x &&
        center_y == other.center_y && radius == other.radius &&
        surge_limit == other.surge_limit && command_horizon == other.command_horizon;
  }


};
}  // namespace teb_local_planner
#endif
