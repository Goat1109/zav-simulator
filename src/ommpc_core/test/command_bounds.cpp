#include <ommpc_core/command_bounds.hpp>
#include <cassert>
#include <limits>
#include <iostream>
int main() {
  ommpc_core::MPCParams p;p.max_bodyrate_xy=1.5;p.max_bodyrate_z=.8;p.min_thrust=1.;p.max_thrust=30.;
  ommpc_core::ControlCommand u;u.collective_acceleration=9.81;
  u.bodyrates<<1.5000000000000002,0.,-.80132749;
  assert(ommpc_core::enforceCommandBounds(u,p));
  assert(u.bodyrates.x()==1.5 && u.bodyrates.z()==-.8);
  u.bodyrates.z()=-.81;assert(!ommpc_core::enforceCommandBounds(u,p));
  u.bodyrates.z()=std::numeric_limits<double>::quiet_NaN();assert(!ommpc_core::enforceCommandBounds(u,p));
  u.bodyrates.setZero();u.collective_acceleration=30.005;
  assert(ommpc_core::enforceCommandBounds(u,p) && u.collective_acceleration==30.);
  u.collective_acceleration=31.;assert(!ommpc_core::enforceCommandBounds(u,p));
  std::cout<<"PASS: recorded yaw overshoot projected, gross/nonfinite outputs rejected\n";
}
