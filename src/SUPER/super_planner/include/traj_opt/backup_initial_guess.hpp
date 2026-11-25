#pragma once
#include <algorithm>
#include <cmath>
#include <limits>

namespace traj_opt {
// A jerk-aware stopping-time estimate for initialization only. The optimized
// trajectory still passes the original corridor and dynamic constraint checks.
inline double backupInitialDuration(double original, double speed, double acceleration,
                                    double distance, double max_acceleration,
                                    double max_jerk, double max_velocity) {
    for (double value : {original, speed, acceleration, distance, max_acceleration, max_jerk, max_velocity})
        if (!std::isfinite(value) || value < 0.) return std::numeric_limits<double>::infinity();
    if (max_acceleration <= 0. || max_jerk <= 0. || max_velocity <= 0.)
        return std::numeric_limits<double>::infinity();
    const double threshold = max_acceleration * max_acceleration / max_jerk;
    const double stopping = speed <= threshold ? 2. * std::sqrt(speed / max_jerk)
                                               : speed / max_acceleration + max_acceleration / max_jerk;
    return std::max({original, stopping, acceleration / max_jerk, distance / max_velocity, 0.1});
}

// Prefer the latest join that already has enough stopping time and remains in
// the SAME safe corridor. Moving the seed does not change admissible geometry.
template<class Traj, class Corridor, class Point>
double backupInitialJoin(const Traj& trajectory, const Corridor& corridor,
                         const Point& endpoint, double lower, double upper,
                         double original, double max_acceleration,
                         double max_jerk, double max_velocity) {
    for (int i=0; i<=48; ++i) {
        const double candidate = original - (original-lower) * i / 48.;
        const auto state = trajectory.getState(candidate);
        if (!state.allFinite() || !corridor.PointIsInside(state.col(0), 0.)) continue;
        const double needed = backupInitialDuration(0., state.col(1).norm(), state.col(2).norm(),
            (endpoint-state.col(0)).norm(), max_acceleration, max_jerk, max_velocity);
        if (upper-candidate >= needed) return candidate;
    }
    return original;
}
}
