#pragma once
#include <Eigen/Core>
#include <cmath>

namespace fsm {
enum class TerminalAction { Wait, Arrived, Replan, Unsafe };

// Applies only after a committed reference expires. Waiting preserves the
// existing zero-PVAJ endpoint; it does not extend a moving/backup reference.
template<class Trajectory>
TerminalAction terminalSettlingAction(const Trajectory& trajectory, double elapsed,
                                     const Eigen::Vector3d& position,
                                     const Eigen::Vector3d& velocity,
                                     const Eigen::Vector3d& goal, double odom_age,
                                     bool endpoint_occupied, double wait_s,
                                     double position_tolerance, double speed_tolerance) {
    if (!std::isfinite(elapsed) || !position.allFinite() || !velocity.allFinite() ||
        !goal.allFinite() || !std::isfinite(odom_age) || odom_age<0. || odom_age>.1 ||
        endpoint_occupied) return TerminalAction::Unsafe;
    if (trajectory.getPieceNum()==0) return TerminalAction::Replan;
    const double end=trajectory.getTotalDuration();
    if (!std::isfinite(end) || end<=0. || elapsed<end) return TerminalAction::Replan;
    const auto p=trajectory.getPos(end),v=trajectory.getVel(end);
    const auto a=trajectory.getAcc(end),j=trajectory.getJer(end);
    if (!p.allFinite() || !v.allFinite() || !a.allFinite() || !j.allFinite())
        return TerminalAction::Unsafe;
    if (v.norm()>=1e-3 || a.norm()>=1e-3 || j.norm()>=1e-3 ||
        (p-goal).norm()>=position_tolerance || (position-p).norm()>.3)
        return TerminalAction::Replan;
    if ((position-goal).norm()<position_tolerance && velocity.norm()<speed_tolerance)
        return TerminalAction::Arrived;
    return elapsed-end<wait_s ? TerminalAction::Wait : TerminalAction::Replan;
}

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
