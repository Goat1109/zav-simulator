#pragma once
#include <Eigen/Core>
#include <cmath>

namespace fsm {
// Waiting for the next goal still executes the previous stopping reference.
// Heartbeat here certifies a live planner and fresh localization, not arrival.
template<class Trajectory>
bool terminalHoldHeartbeatAllowed(const Trajectory& trajectory, double elapsed,
                                  const Eigen::Vector3d& position, double odom_age,
                                  bool endpoint_occupied) {
    if (trajectory.getPieceNum()==0 || !std::isfinite(elapsed) ||
        !std::isfinite(odom_age) || odom_age<0. || odom_age>.1 ||
        !position.allFinite() || endpoint_occupied) return false;
    const double end=trajectory.getTotalDuration();
    if (!std::isfinite(end) || end<=0. || elapsed<end) return false;
    const auto p=trajectory.getPos(end),v=trajectory.getVel(end);
    const auto a=trajectory.getAcc(end),j=trajectory.getJer(end);
    return p.allFinite() && v.allFinite() && a.allFinite() && j.allFinite() &&
        v.norm()<1e-3 && a.norm()<1e-3 && j.norm()<1e-3 &&
        (position-p).norm()<=.3;
}
}
