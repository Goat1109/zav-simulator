#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace traj_opt {
using OptimizationClock = std::chrono::steady_clock;
using OptimizationDeadline = OptimizationClock::time_point;

inline OptimizationDeadline deadlineAfter(double seconds) {
    return OptimizationClock::now() +
        std::chrono::duration_cast<OptimizationClock::duration>(std::chrono::duration<double>(seconds));
}

// A cycle deadline includes frontend work. A call budget also works when an
// optimizer is used alone. Neither deadline depends on ROS or wall-clock jumps.
class OptimizationBudget {
    OptimizationDeadline cycle_{OptimizationDeadline::max()};
    OptimizationDeadline active_{OptimizationDeadline::max()};
    mutable bool timed_out_{false};
public:
    void setCycleDeadline(OptimizationDeadline deadline) { cycle_ = deadline; }
    void begin(double seconds) { active_ = std::min(cycle_, deadlineAfter(seconds)); timed_out_ = false; }
    OptimizationDeadline deadline() const { return active_; }
    bool expired() const {
        if (OptimizationClock::now() >= active_) timed_out_ = true;
        return timed_out_;
    }
    bool timedOut() const { return timed_out_; }
};
}
