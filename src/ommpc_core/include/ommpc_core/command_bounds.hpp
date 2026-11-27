#pragma once
#include <algorithm>
#include <cmath>
#include <ommpc_core/types.hpp>

namespace ommpc_core {
// OSQP constraints have numerical tolerances. Project only small violations;
// a materially infeasible or non-finite command is still a controller failure.
inline bool enforceCommandBounds(ControlCommand &command, const MPCParams &limits) {
  constexpr double rate_tolerance = .005;  // rad/s, includes the recorded .00133 overshoot
  constexpr double acceleration_tolerance = .01;  // m/s^2
  if (!command.bodyrates.allFinite() || !std::isfinite(command.collective_acceleration) ||
      !std::isfinite(limits.max_bodyrate_xy) || !std::isfinite(limits.max_bodyrate_z) ||
      !std::isfinite(limits.min_thrust) || !std::isfinite(limits.max_thrust) ||
      limits.max_bodyrate_xy <= 0. || limits.max_bodyrate_z <= 0. ||
      limits.min_thrust <= 0. || limits.max_thrust < limits.min_thrust) return false;
  const double rates[3] = {limits.max_bodyrate_xy, limits.max_bodyrate_xy, limits.max_bodyrate_z};
  for (int i=0; i<3; ++i)
    if (std::abs(command.bodyrates[i]) > rates[i]+rate_tolerance) return false;
  if (command.collective_acceleration < limits.min_thrust-acceleration_tolerance ||
      command.collective_acceleration > limits.max_thrust+acceleration_tolerance) return false;
  for (int i=0; i<3; ++i) command.bodyrates[i]=std::clamp(command.bodyrates[i],-rates[i],rates[i]);
  command.collective_acceleration=std::clamp(command.collective_acceleration,limits.min_thrust,limits.max_thrust);
  return true;
}
}
