#pragma once
#include <Eigen/Core>
#include <cmath>
#include <vector>
#include <stdexcept>

namespace argo_super {
constexpr double pi=3.14159265358979323846;
inline double angle(double x){return std::atan2(std::sin(x),std::cos(x));}
struct Waypoint {Eigen::Vector3d position;double yaw;};
// Coordinates are frozen at acceptance. World is ENU/FLU: right is -body Y.
inline std::vector<Waypoint> patrol(const Eigen::Vector3d& origin,double yaw,double side=7.){
  if(!origin.allFinite() || !std::isfinite(yaw) || !std::isfinite(side) || side<=0.)
    throw std::invalid_argument("Invalid patrol origin");
  const Eigen::Vector3d f(std::cos(yaw),std::sin(yaw),0.),r(std::sin(yaw),-std::cos(yaw),0.);
  return {{origin+side*f,angle(yaw+pi/2)}, {origin+side*(f+r),angle(yaw)},
          {origin+side*r,angle(yaw-pi/2)}, {origin,angle(yaw+pi)}};
}
inline std::vector<Waypoint> return_home(const Eigen::Vector3d& home,const Eigen::Vector3d& current,double yaw,double clearance=.4){
  if(!home.allFinite() || !current.allFinite() || !std::isfinite(yaw) || clearance<.3 || clearance>2.)
    throw std::invalid_argument("Invalid home/clearance");
  auto approach=home;approach.z()+=clearance;
  // One 3D destination: the planner must find a connected collision-free route.
  // A current-altitude Home-XY waypoint would be inappropriate across floors.
  return {{approach,yaw}};
}
inline bool settled(const Eigen::Vector3d& position,const Eigen::Vector3d& velocity,const Waypoint& target,double yaw){
  return (position-target.position).norm()<.1 && velocity.norm()<.1 && std::abs(angle(yaw-target.yaw))<.0873;
}
}
