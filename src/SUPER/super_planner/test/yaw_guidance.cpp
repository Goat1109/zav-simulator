#include <traj_opt/yaw_traj_opt.h>
#include <cmath>
#include <iostream>
#include <stdexcept>

int main() {
    traj_opt::YawTrajOpt optimizer(.8);
    for (double velocity : {-1.,1.}) {
        geometry_utils::Trajectory position,yaw;
        Eigen::MatrixXd coefficients=Eigen::MatrixXd::Zero(3,8);
        coefficients(0,6)=velocity;
        position.emplace_back(12.,coefficients);
        super_utils::Vec4f initial(.7,0.,0.,0.),goal=initial;
        if(!optimizer.optimize(initial,goal,position,yaw,3,false,false))
            throw std::runtime_error("Explicit exploration yaw optimization failed");
        for(double t=0.;t<=12.;t+=.05)
            if(std::abs(yaw.getPos(t)[0]-.7)>1e-7 || std::abs(yaw.getVel(t)[0])>1e-7)
                throw std::runtime_error("Explicit yaw was replaced by direction of motion");
        initial[0]=velocity>0.?0.:M_PI;
        if(!optimizer.optimize(initial,goal,position,yaw,3,false,true))
            throw std::runtime_error("Travel-facing yaw optimization failed");
        for(double t=0.;t<=12.;t+=.05)
            if(std::abs(std::remainder(yaw.getPos(t)[0]-initial[0],2.*M_PI))>1e-7)
                throw std::runtime_error("Unspecified yaw did not face direction of motion");
    }
    geometry_utils::Trajectory position,yaw;
    Eigen::MatrixXd coefficients=Eigen::MatrixXd::Zero(3,8);coefficients(0,6)=1.;
    position.emplace_back(12.,coefficients);
    super_utils::Vec4f initial(3.12,0.,0.,0.),goal(-3.12,0.,0.,0.);
    if(!optimizer.optimize(initial,goal,position,yaw,3,false,false))
        throw std::runtime_error("Wrapped yaw optimization failed");
    for(double t=0.;t<=12.;t+=.05)
        if(std::abs(yaw.getPos(t)[0]-initial[0])>.05 || std::abs(yaw.getVel(t)[0])>.8)
            throw std::runtime_error("Yaw crossed the long arc around pi");
    std::cout<<"PASS: explicit exploration yaw, unspecified travel-facing yaw in both directions, and shortest pi wrap\n";
}
