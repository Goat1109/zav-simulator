#pragma once
#include <cmath>

namespace super_planner {
// Compare the polynomial epoch, not the newly generated message header stamp.
inline bool initialPlanFresh(double now, double start, double maximum_age) {
    return std::isfinite(now) && std::isfinite(start) && std::isfinite(maximum_age) &&
           maximum_age > 0. && now >= start && now - start <= maximum_age;
}
}
