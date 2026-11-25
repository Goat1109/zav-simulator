#include <zav_super/trajectory.hpp>
#include <ommpc_core/controller_core.hpp>
#include <cassert>
int main(){
  mars_quadrotor_msgs::msg::PolynomialTrajectory m;
  m.order_pos=7;m.piece_num_pos=1;m.time_pos={2.};
  m.coef_pos_x={0,0,0,0,0,0,0,7};m.coef_pos_y=std::vector<double>(8,0);m.coef_pos_z={0,0,0,0,0,0,0,2};
  auto t=argo_super::decode(m);
  assert((t.getPos(1.)-Eigen::Vector3d(7,0,2)).norm()<1e-9);
  assert(argo_super::terminal_stop_reached(t,2.1,{7,0,2},{0,0,0}));
  assert(!argo_super::terminal_stop_reached(t,1.,{7,0,2},{0,0,0}));
  assert(!argo_super::terminal_stop_reached(t,3.,{5,0,2},{0,0,0}));
  m.coef_pos_x.pop_back();bool caught=false;try{argo_super::decode(m);}catch(const std::invalid_argument&){caught=true;}assert(caught);
  ommpc_core::ParameterSet p;p.mpc.step_T=.02;p.mpc.Q_pos_xy=600;p.mpc.Q_pos_z=800;p.mpc.Q_velocity=60;
  p.mpc.Q_attitude_rp=p.mpc.Q_attitude_yaw=80;p.mpc.R_thrust=.5;p.mpc.R_pitchroll=16;p.mpc.R_yaw=3;
  p.mpc.state_cost_exponential=p.mpc.input_cost_exponential=.5;p.mpc.max_bodyrate_xy=1.5;p.mpc.max_bodyrate_z=.8;p.mpc.min_thrust=1;p.mpc.max_thrust=30;
  ommpc_core::OMMPCControllerCore core;core.init(p);
  ommpc_core::ControllerState state;state.p={0,0,2};state.timestamp_s=1;
  assert(core.setHoverReference({0,0,2,0}));ommpc_core::ControlCommand u;
  assert(core.execMPC(state,1,u));assert(u.bodyrates.norm()<.01);assert(std::abs(u.collective_acceleration-9.81)<.1);
  return 0;
}
