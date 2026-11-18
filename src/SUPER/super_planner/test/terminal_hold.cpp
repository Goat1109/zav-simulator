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
    for (int derivative=1;derivative<=3;++derivative) {
        c.setZero();
        if (derivative==1) c(0,6)=1.;
        if (derivative==2) {c(0,5)=1.;c(0,6)=-2.;c(0,7)=2.;}
        if (derivative==3) {c(0,4)=1.;c(0,5)=-3.;c(0,6)=3.;}
        Trajectory moving;moving.emplace_back(1.,c);
        if (allowed(moving,1.1,.02)) throw std::runtime_error("Non-stopping reference was certified");
    }
}
