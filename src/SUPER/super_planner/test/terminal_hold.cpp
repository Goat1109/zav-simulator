#include <fsm/terminal_hold.hpp>
#include <traj_opt/yaw_traj_opt.h>
#include <stdexcept>
#include <limits>

int main() {
    using geometry_utils::Trajectory;
    const Eigen::Vector3d position(1.,0.,0.);
    Eigen::MatrixXd c=Eigen::MatrixXd::Zero(3,8);c(0,7)=1.;
    Trajectory stopped;stopped.emplace_back(1.,c);
    auto allowed=[&](const Trajectory& t,double elapsed,double age,bool occupied=false) {
        return fsm::terminalHoldHeartbeatAllowed(t,elapsed,position,age,occupied);
    };
    if (!allowed(stopped,1.1,.02)) throw std::runtime_error("Live completed stopping reference lost heartbeat");
    if (allowed(stopped,.9,.02) || allowed(stopped,1.1,.11) ||
        allowed(stopped,1.1,-.01) || allowed(stopped,1.1,.02,true) ||
        allowed(Trajectory{},1.1,.02)) throw std::runtime_error("Invalid terminal hold was certified");
    if (fsm::terminalHoldHeartbeatAllowed(stopped,1.1,Eigen::Vector3d(2.,0.,0.),.02,false) ||
        fsm::terminalHoldHeartbeatAllowed(stopped,1.1,Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()),.02,false))
        throw std::runtime_error("Distant or invalid localization was certified");
    const auto goal=position;
    auto action=[&](const Trajectory& t,double elapsed,double offset,double speed,
                    double age=.02,bool occupied=false,Eigen::Vector3d target=Eigen::Vector3d(1.,0.,0.)) {
        return fsm::terminalSettlingAction(t,elapsed,goal+Eigen::Vector3d(offset,0.,0.),
            Eigen::Vector3d(speed,0.,0.),target,age,occupied,3.,.1,.1);
    };
    using A=fsm::TerminalAction;
    // Regression: 15cm residual and braking velocity must preserve the old
    // reference instead of restarting a short trajectory at every expiration.
    if (action(stopped,1.01,.15,.2)!=A::Wait ||
        action(stopped,3.99,.15,.02)!=A::Wait ||
        action(stopped,4.01,.15,.02)!=A::Replan)
        throw std::runtime_error("Terminal convergence wait/deadline is incorrect");
    if (action(stopped,1.1,.08,.12)!=A::Wait || action(stopped,1.1,.08,.08)!=A::Arrived ||
        action(stopped,1.1,.15,.0)!=A::Wait)
        throw std::runtime_error("10cm/0.1m/s arrival criteria are not enforced");
    // A backup stop away from the actual goal, or localization outside the
    // pre-existing 30cm envelope, must not be treated as terminal arrival.
    if (action(stopped,1.1,.0,.0,.02,false,Eigen::Vector3d(2.,0.,0.))!=A::Replan ||
        action(stopped,1.1,.31,.0)!=A::Replan)
        throw std::runtime_error("Backup/distant endpoint was accepted as the goal");
    if (action(stopped,1.1,.15,.0,.11)!=A::Unsafe ||
        action(stopped,1.1,.15,.0,.02,true)!=A::Unsafe ||
        action(stopped,1.1,.15,std::numeric_limits<double>::quiet_NaN())!=A::Unsafe)
        throw std::runtime_error("Unsafe terminal settling was allowed");
    for (int derivative=1;derivative<=3;++derivative) {
        c.setZero();
        if (derivative==1) c(0,6)=1.;
        if (derivative==2) {c(0,5)=1.;c(0,6)=-2.;c(0,7)=2.;}
        if (derivative==3) {c(0,4)=1.;c(0,5)=-3.;c(0,6)=3.;}
        Trajectory moving;moving.emplace_back(1.,c);
        if (allowed(moving,1.1,.02)) throw std::runtime_error("Non-stopping reference was certified");
        if (action(moving,1.1,.0,.0)!=A::Replan)
            throw std::runtime_error("A moving endpoint was held as a stop");
    }
}
