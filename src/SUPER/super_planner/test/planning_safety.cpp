#include <super_core/planning_envelope.hpp>
#include <data_structure/base/curve_certificate.hpp>
#include <data_structure/base/trajectory.h>
#include <cassert>
#include <iostream>
using namespace super_planner;
using geometry_utils::Trajectory;
int main(){
 PlanningEnvelope e;e.enabled=e.ready=true;e.world_floor=-.37;e.world_ceiling=1.37;
 e.profile_floor=1.2;e.profile_ceiling=1.6;
 Eigen::Quaterniond q(.06477022639804543,0,0,-.9979002043152141);
 e.rotation=q.toRotationMatrix();e.translation={-3.3085676191682145,-.0859663940202335,.34432277840586933};
 e.lower={{-11.2,-11.2,-.3},{-9.6,-2.,-.3}};e.upper={{-6.5,47.2,2.4},{-2.6,1.65,2.4}};
 Eigen::Vector3d requested(4.533537999554331,-25.97212448524284,1.0009707434752342);
 Eigen::Vector3d old(4.575,-26.025,.975),corrected;
 assert(e.contains(requested) && !e.contains(old)); // recorded 9.393mm violation
 const Eigen::Vector3i centre=(requested/.15).array().floor().cast<int>();
 auto pos=[](const Eigen::Vector3i& i)->Eigen::Vector3d{return (i.cast<double>().array()+.5)*.15;};
 assert(boundedGridGoal(requested,centre,.15,.3,pos,[&](const auto& p){return e.contains(p);},corrected));
 assert(e.contains(corrected) && (corrected-requested).norm()<=.3);
 assert(!boundedGridGoal(requested,centre,.15,.3,pos,[](const auto&){return false;},corrected));
 Eigen::Matrix<double,Eigen::Dynamic,4> planes(0,4);
 assert(e.clip(planes,corrected,corrected));
 assert((planes.leftCols(3)*corrected+planes.col(3)).maxCoeff()<1e-8);
 assert((planes.leftCols(3)*old+planes.col(3)).maxCoeff()>0.);
 e.ready=false;assert(!e.contains(requested));e.ready=true;
 auto make=[](double speed){Trajectory t;Eigen::Matrix<double,3,8> c=Eigen::Matrix<double,3,8>::Zero();c(0,6)=speed;t.emplace_back(2.,c);return t;};
 assert(super_utils::certifyKinematics(make(.95),1.,1.5,3.));
 assert(!super_utils::certifyKinematics(make(1.5),1.,1.5,3.));
 // Quadratic velocity peaks between t=0 and t=1; endpoints alone pass.
 Trajectory peak;Eigen::Matrix<double,3,8> c=Eigen::Matrix<double,3,8>::Zero();c(0,4)=-2.;c(0,5)=3.;peak.emplace_back(1.,c);
 assert(peak.getVel(0.).norm()<1. && peak.getVel(1.).norm()<1.);
 assert(!super_utils::certifyKinematics(peak,1.,10.,20.));
 std::cout<<"PASS recorded boundary snap, mission planes, fail-closed and continuous overspeed\n";
}
