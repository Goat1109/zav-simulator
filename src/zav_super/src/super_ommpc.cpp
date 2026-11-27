// Hardware wrapper around the unchanged argo_v2 OMMPC core. No FC transport,
// arm, takeoff, mode-change or land commands. The EV bridge owns the UART.
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/bool.hpp>
#include <zav_super/epic_gate.hpp>
#include <super_core/entry_envelope.hpp>
#include <zav_super/path_follower.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <zav_super/srv/go_forward.hpp>
#include <ommpc_core/controller_core.hpp>
#include <zav_super/trajectory.hpp>
#include <data_structure/base/curve_certificate.hpp>
#include <zav_super/missions.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <sys/file.h>
#include <unistd.h>
#include <fcntl.h>
using Steady=std::chrono::steady_clock;
using Poly=mars_quadrotor_msgs::msg::PolynomialTrajectory;
class HardwareController : public rclcpp::Node {
  ommpc_core::OMMPCControllerCore mpc_;
  ommpc_core::ControllerState state_;
  Trajectory trajectory_,yaw_;
  Eigen::Vector4d hold_=Eigen::Vector4d::Zero();
  Eigen::Vector3d goal_=Eigen::Vector3d::Zero();
  Eigen::Vector3d requested_goal_=Eigen::Vector3d::Zero();
  int64_t goal_request_stamp_ns_=0,committed_request_stamp_ns_=0;
  double replacement_timeout_s_=5.,epic_guide_wait_timeout_s_=20.;
  double terminal_settle_time_=3., retry_outside_since_=-1.;
  double goal_position_tolerance_=.1,goal_speed_tolerance_=.1,goal_snap_max_distance_=.3;
  double active_r_pitchroll_=0.;
  double tracking_prediction_limit_s_=0.;
  bool execute_=false,odom_ok_=false,active_=false,mission_=false,fault_=false;
  bool bridge_active_=false,bridge_healthy_=false;
  double last_stamp_=0.,step_=.02,period_=.02,distance_=10.,start_=0.,yaw_start_=0.,mission_epoch_=0.;
  double cloud_stamp_=0.;
  int lock_=-1,failures_=0;
  unsigned long sequence_=0;
  std::string session_,phase_="WAITING";
  Eigen::Vector3d home_=Eigen::Vector3d::Zero();
  double home_yaw_=0., home_sample_at_=0., turn_yaw_=0., settled_at_=0., takeoff_height_=.4,turn_started_=0.;
  bool home_valid_=false,seen_disarmed_=false,armed_=false,landed_=false,home_locked_=false;
  bool route_turning_=false,mission_complete_=false,pending_plan_=false,landing_=false;
  double rc_ground_seed_=0.;
  bool px4_tracking_=false;
  bool px4_hover_handoff_=false;
  bool auto_takeoff_=false,local_takeoff_=false,takeoff_started_=false,land_disarm_enabled_=false;
  std::string mission_kind_="none",intent_="none";
  std::vector<argo_super::Waypoint> route_;
  size_t leg_=0;
  argo_super::PathFollower epic_follower_;
  double epic_lookahead_=4.,epic_lookahead_max_=15.,epic_chord_=.15,epic_path_acc_=6.,epic_path_jerk_=10.;
  double last_goal_yaw_=0.,trajectory_max_vel_=1.,trajectory_max_acc_=6.,trajectory_max_jerk_=10.;
  bool entering_=false;
  super_planner::PlanningEnvelope entry_corridor_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr entry_goal_;
  bool epic_guide_arrived_=false;
  bool epic_target_final_=false,epic_sent_final_=false,epic_route_dirty_=false;
  unsigned long epic_revision_=0;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planner_cancel_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr trajectory_feedback_;
  void cancelPlanner(const std::string& reason){
    std_msgs::msg::String m; m.data="{\"stamp_ns\":"+std::to_string(now().nanoseconds())+",\"reason\":\""+reason+"\"}";
    planner_cancel_->publish(m);
  }

  bool epic_frame_required_=false,epic_frame_received_=false,epic_frame_changed_=false;
  double epic_frame_c_=1.,epic_frame_s_=0.;
  Eigen::Vector3d epic_frame_translation_=Eigen::Vector3d::Zero();
  Steady::time_point epic_frame_rx_{};
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr epic_frame_sub_;
  bool epic_hover_ready_=false,epic_enabled_=false,epic_airborne_=false,epic_battery_ok_=false,epic_rc_fresh_=false;
  int epic_rc7_=0;
  argo_super::EpicSwitch epic_switch_;
  argo_super::EpicBounds epic_bounds_;
  double epic_world_floor_=-.4,epic_world_ceiling_=9.5;
  std::string epic_profile_,epic_fsm_="UNKNOWN",epic_reason_="WAIT_READY";
  Steady::time_point epic_state_rx_{},epic_wait_rx_{},epic_status_tx_{};
  int64_t epic_epoch_ns_=0,epic_guide_stamp_ns_=0;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr epic_trigger_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr epic_manual_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr epic_status_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr epic_path_sub_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr epic_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr epic_done_sub_;
  double epic_flight_min_=0.,epic_flight_max_=9.5,epic_rc_age_=999.;
  Steady::time_point created_=Steady::now();
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> mission_services_;
  Steady::time_point odom_rx_{},cloud_rx_{},bridge_rx_{},heartbeat_rx_{},goal_rx_{};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr commands_,status_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goals_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr bridge_sub_;
  rclcpp::Subscription<Poly>::SharedPtr traj_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planner_status_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr go_;
  rclcpp::Service<zav_super::srv::GoForward>::SharedPtr go_distance_;

  argo_super::EpicPoint point(const Eigen::Vector3d& p) const {
    return {epic_frame_c_*p.x()-epic_frame_s_*p.y()+epic_frame_translation_.x(),
            epic_frame_s_*p.x()+epic_frame_c_*p.y()+epic_frame_translation_.y(),p.z()+epic_frame_translation_.z()};
  }
  bool insideEpic(const Eigen::Vector3d& p,double margin=0.)const{
    return epic_bounds_.contains(point(p),margin);
  }
  bool pilot_ready_=false,pilot_moving_=false,braking_for_epic_=false;
  Eigen::Vector4d pilot_axes_=Eigen::Vector4d::Zero();
  Steady::time_point pilot_rx_{},pilot_tick_{};
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr pilot_sub_;

  void stopPilot(){
    if(pilot_moving_ && odom_ok_){hold_.head<3>()=state_.p;hold_[3]=heading();}
    pilot_ready_=pilot_moving_=false;
    pilot_rx_={};
  }
  void updatePilot(){
    const double dt=std::clamp(age(pilot_tick_),0.,.03);
    pilot_tick_=Steady::now();
    if(!epic_enabled_ || !bridge_active_ || !active_ || !armed_ || !epic_airborne_ ||
       !epic_rc_fresh_ || epic_rc_age_+age(bridge_rx_)>=.5 || epic_switch_.running ||
       mission_ || mission_kind_!="none" || age(pilot_rx_)>=.2){
      stopPilot();return;
    }
    const bool centered=pilot_axes_.isZero();
    if(!pilot_ready_){
      pilot_ready_=centered && epic_hover_ready_ && state_.v.norm()<.15;
      return;
    }
    if(centered){
      if(pilot_moving_){hold_.head<3>()=state_.p;hold_[3]=heading();}
      pilot_moving_=false;return;
    }
    pilot_moving_=true;phase_="PILOT_STICKS";
    Eigen::Vector2d xy=pilot_axes_.head<2>();
    xy/=std::max(1.,xy.norm());
    const double yaw=heading(),speed=std::min(.5,trajectory_max_vel_);
    Eigen::Vector3d velocity(speed*(cos(yaw)*xy.x()-sin(yaw)*xy.y()),
                                   speed*(sin(yaw)*xy.x()+cos(yaw)*xy.y()),
                                   std::min(.3,trajectory_max_vel_)*pilot_axes_.z());
    if(velocity.norm()>trajectory_max_vel_)velocity=velocity.normalized()*trajectory_max_vel_;
    // Limit the lead so holding a stick against a tracking error cannot wind up the reference.
    const Eigen::Vector3d offset=hold_.head<3>()+dt*velocity-state_.p;
    hold_.head<3>()=state_.p+offset/std::max(1.,offset.norm()/.25);
    hold_.z()=std::clamp(hold_.z(),epic_world_floor_+.03,epic_world_ceiling_-.03);
    hold_[3]=argo_super::angle(yaw+std::clamp(argo_super::angle(hold_[3]-yaw)+.5*dt*pilot_axes_[3],-.25,.25));
  }
  void stopEpic(const std::string& reason){
    braking_for_epic_=false;stopPilot();entering_=false;epic_switch_.stop();epic_reason_=reason;
    if(mission_kind_=="epic"){cancelPlanner(reason);freeze();if(!fault_)phase_="HOVER";}
    std_msgs::msg::Bool manual;manual.data=true;epic_manual_->publish(manual);
  }
  void epicGuide(const nav_msgs::msg::Path& path){
    if(braking_for_epic_ || entering_ || !epic_switch_.running || !admission(true).empty())return;
    if(count_publishers("/mission/exploration_path")!=1){stopEpic("MULTIPLE_EPIC_PUBLISHERS_HOLD");return;}
    const auto stamp=rclcpp::Time(path.header.stamp).nanoseconds();
    const double dt=(now().nanoseconds()-stamp)/1e9;
    if(stamp<epic_epoch_ns_ || stamp<=epic_guide_stamp_ns_ || dt<-.02 || dt>1.)return;
    auto reject=[this](){stopEpic("INVALID_GUIDE_HOLD");};
    if(path.header.frame_id!="world" || path.poses.size()<2 || path.poses.size()>1000){reject();return;}
    std::vector<argo_super::Waypoint> route;
    double length=0.;
    Eigen::Vector3d previous=state_.p;
    for(size_t i=0;i<path.poses.size();++i){
      const auto& msg=path.poses[i];const auto& v=msg.pose.position;const auto& q=msg.pose.orientation;
      Eigen::Vector3d p(v.x,v.y,v.z);
      const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
      if(msg.header.frame_id!="world" || !p.allFinite() || !std::isfinite(norm) || std::abs(norm-1.)>.05 || !insideEpic(p)){reject();return;}
      if(i>0 && !epic_bounds_.segment(point(previous),point(p))){reject();return;}
      if(i>0)length+=(p-previous).norm();
      if(length>250.){reject();return;}
      const double yaw=std::atan2(2.*(q.w*q.z+q.x*q.y),1.-2.*(q.y*q.y+q.z*q.z));
      route.push_back({p,yaw});previous=p;
    }
    epic_guide_stamp_ns_=stamp;
    // Repeated previews must not rewind progress, including after their first
    // vertex is behind the moving vehicle.
    if(!epic_follower_.empty() && epic_follower_.same(route))return;
    if((route.front().position-state_.p).norm()>1. ||
       !epic_bounds_.segment(point(state_.p),point(route.front().position))){reject();return;}
    try {
      argo_super::PathFollower revised(route,epic_chord_);
      revised.inheritProgress(epic_follower_,state_.p);
      epic_follower_=std::move(revised);
    }
    catch(const std::exception&){reject();return;}
    const bool same_final=!route_.empty() && (route.back().position-route_.back().position).norm()<.1 &&
      std::abs(argo_super::angle(route.back().yaw-route_.back().yaw))<.1;
    route_=std::move(route);leg_=0;mission_kind_="epic";mission_complete_=false;
    epic_guide_arrived_=false;
    ++epic_revision_;epic_route_dirty_=true;epic_sent_final_=same_final && epic_sent_final_;settled_at_=0.;epic_reason_="FOLLOW_GUIDE";
    updateEpicGoal();
  }
  void updateEpicGoal(){
    if(!epic_switch_.running || epic_follower_.empty() || mission_kind_!="epic")return;
    epic_follower_.lookahead=argo_super::brakingLookahead(epic_lookahead_,state_.v.norm(),epic_path_acc_,epic_path_jerk_,epic_lookahead_max_);
    // Cap each forward preview increment to at most half a second of travel.
    const double advance=trajectory_max_vel_*std::clamp(age(goal_rx_),0.,.5);
    const auto target=epic_follower_.update(state_.p,mission_ ? &requested_goal_ : nullptr,advance);
    leg_=epic_follower_.segment();epic_target_final_=epic_follower_.targetArc()>=epic_follower_.length()-1e-6;
    // Exactly one outstanding request: continuously replace only the desired
    // target, never the request identity/deadline or the committed reference.
    if(pending_plan_)return;
    const double target_change=(target.position-requested_goal_).norm();
    const bool changed=!mission_ || (epic_route_dirty_ && target_change>.1) || target_change>.3 ||
      std::abs(argo_super::angle(target.yaw-last_goal_yaw_))>.1 || (epic_target_final_ && !epic_sent_final_);
    // Low speed also occurs during the valid final braking phase. Reissuing
    // the identical goal there can replace an almost-finished request with a
    // cached/expired response. Retry only a completed stop short of the goal;
    // normal rolling target/path changes above still update while moving.
    const bool stopped=trajectory_.getPieceNum()>0 && argo_super::terminal_stop_reached(
      trajectory_,std::max(0.,state_.timestamp_s-start_),state_.p,state_.v);
    // goal_ is the map-adjusted goal from the committed response. Give the
    // stopped reference time to converge. A brief tolerance crossing must
    // not reset the planner deadline by issuing another goal.
    const double t=state_.timestamp_s;
    // Hysteresis on retry only. The arrival tolerance stays unchanged.
    const double retry_distance=goal_position_tolerance_*(retry_outside_since_<0.?1.2:1.);
    const bool outside=stopped && !mission_complete_ &&
      (state_.p-goal_).norm()>=retry_distance;
    if(!outside) retry_outside_since_=-1.;
    else if(retry_outside_since_<0.) retry_outside_since_=t;
    // SUPER initiates its own recovery at the settling deadline. Give it one
    // response interval before a supervisor retry supersedes that recovery.
    const bool retry=outside && t-retry_outside_since_>=terminal_settle_time_+replacement_timeout_s_;
    if((changed || retry) && (!mission_ || age(goal_rx_)>=.2)){
      if(!epic_bounds_.segment(point(state_.p),point(target.position),250.)){
        stopEpic("PREVIEW_BOUNDARY_HOLD");return;
      }
      epic_route_dirty_=false;epic_sent_final_=epic_target_final_;last_goal_yaw_=target.yaw;
      publishGoal(target,trajectory_.getPieceNum()>0);
    }
  }
  bool entryTarget(Eigen::Vector3d& world,bool& inside) const {
    const auto p=point(state_.p);
    double best=std::numeric_limits<double>::infinity();
    inside=false;
    for(size_t box=0;box<epic_bounds_.lower.size();++box){
      auto lo=epic_bounds_.lower[box],hi=epic_bounds_.upper[box];
      lo[2]=std::max({lo[2],epic_flight_min_,epic_bounds_.floor});
      hi[2]=std::min({hi[2],epic_flight_max_,epic_bounds_.ceiling});
      Eigen::Vector3d q;
      bool valid=true,within=true;
      for(int k=0;k<3;++k){
        if(hi[k]-lo[k]<=.06){valid=false;break;}
        within &= p[k]>=lo[k]+.03 && p[k]<=hi[k]-.03;
        // Leave room for grid snapping and endpoint tolerance when entering.
        const double inset=std::min(.2,(hi[k]-lo[k])*.49);
        q[k]=std::clamp(p[k],lo[k]+inset,hi[k]-inset);
      }
      if(!valid)continue;
      inside|=within;
      q-=epic_frame_translation_;
      Eigen::Vector3d target(epic_frame_c_*q.x()+epic_frame_s_*q.y(),
                             -epic_frame_s_*q.x()+epic_frame_c_*q.y(),q.z());
      const double distance=(target-state_.p).squaredNorm();
      if(distance<best){world=target;best=distance;}
    }
    return std::isfinite(best);
  }
  void triggerExploration(){
    epic_epoch_ns_=now().nanoseconds();epic_guide_stamp_ns_=0;epic_follower_=argo_super::PathFollower();
    epic_wait_rx_=Steady::now();epic_reason_="WAIT_GUIDE";
    nav_msgs::msg::Path trigger;trigger.header.frame_id="world";trigger.header.stamp=now();
    geometry_msgs::msg::PoseStamped pose;pose.header=trigger.header;
    pose.pose.position.z=std::max(0.,state_.p.z());pose.pose.orientation.w=1.;
    trigger.poses.push_back(pose);epic_trigger_->publish(trigger);
  }
  void epicTick(){
    if(!epic_enabled_)return;
    const bool live=age(bridge_rx_)<.6 && epic_rc_fresh_ && epic_rc_age_+age(bridge_rx_)<.5;
    const bool node=age(epic_state_rx_)<1.5;
    const bool bounds=odom_ok_ && insideEpic(state_.p,.2);
    const bool frame=!epic_frame_required_ || (epic_frame_received_ && !epic_frame_changed_ && age(epic_frame_rx_)<.6);
    Eigen::Vector3d entry_target;
    bool inside=false;
    const bool entry_possible=frame && odom_ok_ && state_.p.z()>=epic_world_floor_+.03 &&
      state_.p.z()<=epic_world_ceiling_-.03 && entryTarget(entry_target,inside);
    const bool location=braking_for_epic_?entry_possible:entering_?entry_corridor_.contains(state_.p):
      (epic_switch_.running?bounds:entry_possible);
    const bool safe=node && live && frame && epic_airborne_ && epic_battery_ok_ && location && admission(true).empty() &&
      (mission_kind_=="none" || mission_kind_=="epic" || mission_complete_);
    const bool ready=safe && epic_hover_ready_ && epic_fsm_=="WAIT_TRIGGER" && admission().empty() &&
      (phase_=="HOVER" || phase_=="ENDPOINT_HOLD");
    const bool can_brake=safe && pilot_ready_ && age(pilot_rx_)<.2 && mission_kind_=="none" && epic_fsm_=="WAIT_TRIGGER";
    const auto event=epic_switch_.update(std::chrono::duration<double>(Steady::now().time_since_epoch()).count(),live,epic_rc7_,ready || can_brake,safe);
    if(event==argo_super::EpicSwitch::STOP){
      stopEpic(!live?"RC7_LOST_HOLD":!node?"EPIC_TIMEOUT_HOLD":!epic_battery_ok_?"BATTERY_HOLD":
        !location?"BOUNDARY_HOLD":!frame?"PROFILE_FRAME_LOST_HOLD":!safe?"CONTROL_NOT_READY_HOLD":"RC7_OUTSIDE_MIDDLE_HOLD");
    }else if(event==argo_super::EpicSwitch::START){
      stopPilot();
      braking_for_epic_=true;epic_wait_rx_=Steady::now();epic_reason_="BRAKING_FOR_EPIC";
    }
    if(braking_for_epic_){
      if(epic_fsm_!="WAIT_TRIGGER" || age(epic_wait_rx_)>epic_guide_wait_timeout_s_){stopEpic("BRAKE_TIMEOUT_HOLD");}
      else if(epic_hover_ready_ && state_.v.norm()<.15 && (phase_=="HOVER" || phase_=="ENDPOINT_HOLD")){
        braking_for_epic_=false;
        if(inside)triggerExploration();
        else{
          entering_=true;epic_reason_="ENTERING_BOUNDS";
          entry_corridor_=super_planner::entryEnvelope(state_.p,entry_target,epic_world_floor_+.03,epic_world_ceiling_-.03);
          mission_kind_="epic";last_goal_yaw_=heading();
          publishGoal({entry_target,last_goal_yaw_});
        }
      }
    }
    if(epic_switch_.running && entering_){
      if(fault_ || epic_fsm_!="WAIT_TRIGGER" || (pending_plan_ && age(goal_rx_)>replacement_timeout_s_) ||
         age(goal_rx_)>120.)stopEpic("ENTRY_PLAN_FAILED");
      else if(phase_=="ENDPOINT_HOLD" && !pending_plan_ && inside && (state_.p-goal_).norm()<goal_position_tolerance_ &&
              state_.v.norm()<goal_speed_tolerance_){
        cancelPlanner("ENTRY_COMPLETE");freeze();entering_=false;triggerExploration();
      }
    }else if(epic_switch_.running && !braking_for_epic_){
      if(epic_fsm_=="LAND" || epic_fsm_=="CAUTION"){stopEpic("EPIC_UNSAFE_HOLD");}
      else if(epic_fsm_=="FINISH"){stopEpic("COMPLETE_HOLD");}
      else if(mission_kind_!="epic" || epic_guide_arrived_){
        if(age(epic_wait_rx_)>epic_guide_wait_timeout_s_)stopEpic("NO_GUIDE_HOLD");
      }else {epic_wait_rx_=Steady::now();updateEpicGoal();}
    }
    if(age(epic_status_tx_)<.1)return;epic_status_tx_=Steady::now();
    if(!epic_switch_.running){std_msgs::msg::Bool manual;manual.data=true;epic_manual_->publish(manual);}
    const std::string blocked=!frame?(epic_reason_=="HEIGHT_BAND_CONFLICT"?"HEIGHT_BAND_CONFLICT":"GROUND_ANCHOR_REQUIRED"):!node?"EPIC_STATUS_STALE":!live?"RC7_STALE":!epic_airborne_?"ARMED_AIRBORNE_REQUIRED":
      !epic_battery_ok_?"BATTERY_NOT_READY":!epic_hover_ready_?"OFFBOARD_HOVER_NOT_READY":!location?"OUTSIDE_ENTRY_ENVELOPE":!admission().empty()?admission():
      epic_fsm_!="WAIT_TRIGGER"?"WAIT_EPIC_READY":"";
    std::ostringstream j;j<<"{\"stamp_ns\":"<<now().nanoseconds()<<",\"enabled\":true,\"profile\":\""<<epic_profile_<<"\",\"active\":"<<(epic_switch_.running?"true":"false")
      <<",\"fsm\":\""<<epic_fsm_<<"\",\"fsm_fresh\":"<<(node?"true":"false")
      <<",\"state\":\""<<(epic_switch_.running?"EXPLORING":ready?"READY_RC7":"HOLD_OR_WAIT")
      <<"\",\"reason\":\""<<epic_reason_<<"\",\"blocked_by\":\""<<blocked<<"\",\"rc7_pwm\":"<<epic_rc7_
      <<",\"rc7_fresh\":"<<(live?"true":"false")<<",\"edge_armed\":"<<(epic_switch_.armed?"true":"false")
      <<",\"world_height_bounds\":["<<epic_world_floor_<<","<<epic_world_ceiling_<<"]"
      <<",\"profile_height_bounds\":["<<epic_bounds_.floor<<","<<epic_bounds_.ceiling<<"]"
      <<",\"aligned_flight_band\":["<<std::max(epic_flight_min_,epic_bounds_.floor+.03)<<","<<std::min(epic_flight_max_,epic_bounds_.ceiling-.03)<<"]"
      <<",\"revision\":"<<epic_revision_<<",\"progress_m\":"<<epic_follower_.progress()<<",\"target_arc_m\":"<<epic_follower_.targetArc()
      <<",\"guide_arrived\":"<<(epic_guide_arrived_?"true":"false")<<",\"path_length_m\":"<<epic_follower_.length()<<",\"pending_plan\":"<<(pending_plan_?"true":"false")
      <<",\"committed_request_stamp_ns\":"<<committed_request_stamp_ns_
      <<",\"goal_request_stamp_ns\":"<<goal_request_stamp_ns_<<",\"request_age_s\":"<<(mission_?age(goal_rx_):0.)
      <<",\"leg\":"<<(mission_kind_=="epic"?leg_+1:0)<<",\"legs\":"<<(mission_kind_=="epic"?route_.size():0)<<"}";
    std_msgs::msg::String msg;msg.data=j.str();epic_status_->publish(msg);
  }
  void setupEpic(){
    epic_enabled_=declare_parameter<bool>("epic_enabled",false);
    epic_profile_=declare_parameter<std::string>("epic_profile","gym");
    if(!epic_enabled_)return;
    pilot_sub_=create_subscription<sensor_msgs::msg::Joy>("/zav_real/pilot_sticks",rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::Joy::SharedPtr msg){
        const double lag=now().seconds()-rclcpp::Time(msg->header.stamp).seconds();
        if(msg->axes.size()!=4 || lag<-.02 || lag>.1 || count_publishers("/zav_real/pilot_sticks")!=1)return;
        Eigen::Vector4d axes;
        for(int i=0;i<4;++i){
          const double v=msg->axes[i];
          if(!std::isfinite(v) || std::abs(v)>1.)return;
          axes[i]=std::copysign(std::max(0.,std::abs(v)-.1)/.9,v);
        }
        pilot_axes_=axes;pilot_rx_=Steady::now();
      });
    epic_lookahead_=declare_parameter<double>("epic_path_lookahead_m",4.);
    epic_lookahead_max_=declare_parameter<double>("epic_path_lookahead_max_m",15.);
    epic_chord_=declare_parameter<double>("epic_path_chord_error_m",.15);
    epic_path_acc_=declare_parameter<double>("epic_path_max_acc",6.);
    epic_path_jerk_=declare_parameter<double>("epic_path_max_jerk",10.);
    if(!std::isfinite(epic_path_acc_) || epic_path_acc_<=0. || !std::isfinite(epic_path_jerk_) || epic_path_jerk_<=0. || !std::isfinite(epic_lookahead_) || !std::isfinite(epic_lookahead_max_) || !std::isfinite(epic_chord_) ||
       epic_lookahead_<=0. || epic_lookahead_max_<epic_lookahead_ || epic_lookahead_max_>30. || epic_chord_<=0. || epic_chord_>.3)
      throw std::invalid_argument("Invalid EPIC path following parameters");
    epic_frame_required_=declare_parameter<bool>("epic_frame_required",false);
    if(epic_profile_!="gym" && epic_profile_!="frontier_whole" && epic_profile_!="frontier_single_floor" && epic_profile_!="forest")throw std::invalid_argument("Invalid EPIC profile");
    auto low=declare_parameter<std::vector<double>>("epic_bounds_lower",std::vector<double>{});
    auto high=declare_parameter<std::vector<double>>("epic_bounds_upper",std::vector<double>{});
    epic_bounds_.floor=declare_parameter<double>("epic_floor",-.4);epic_bounds_.ceiling=declare_parameter<double>("epic_ceiling",9.5);
    epic_world_floor_=epic_bounds_.floor;epic_world_ceiling_=epic_bounds_.ceiling;
    epic_flight_min_=declare_parameter<double>("epic_flight_min",.3);epic_flight_max_=declare_parameter<double>("epic_flight_max",9.5);
    if(low.empty() || low.size()!=high.size() || low.size()%3 || low.size()>12 ||
       !std::isfinite(epic_bounds_.floor) || !std::isfinite(epic_bounds_.ceiling) || epic_bounds_.floor>=epic_bounds_.ceiling ||
       !std::isfinite(epic_flight_min_) ||
       !std::isfinite(epic_flight_max_) || epic_flight_min_>=epic_flight_max_)throw std::invalid_argument("Real EPIC bounds required");
    for(size_t i=0;i<low.size();i+=3){
      argo_super::EpicPoint a,b;
      for(size_t k=0;k<3;++k){a[k]=low[i+k];b[k]=high[i+k];if(!std::isfinite(a[k]) || !std::isfinite(b[k]) || a[k]>=b[k])throw std::invalid_argument("Invalid EPIC bounds");}
      epic_bounds_.lower.push_back(a);epic_bounds_.upper.push_back(b);
    }
    if(epic_frame_required_)epic_frame_sub_=create_subscription<geometry_msgs::msg::PoseStamped>("/zav/epic/profile_transform",rclcpp::QoS(1).reliable().transient_local(),[this](geometry_msgs::msg::PoseStamped::SharedPtr m){
      const double dt=now().seconds()-rclcpp::Time(m->header.stamp).seconds();const auto& p=m->pose.position;const auto& q=m->pose.orientation;
      Eigen::Vector3d translation(p.x,p.y,p.z);
      const double norm=q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z;
      if(m->header.frame_id!="epic_profile" || dt<-.02 || dt>.25 || !translation.allFinite() ||
         !std::isfinite(norm) || std::abs(norm-1.)>.01 || std::abs(q.x)+std::abs(q.y)>1.e-6 || count_publishers("/zav/epic/profile_transform")!=1)return;
      const double c=1.-2.*q.z*q.z,s=2.*q.w*q.z;
      if(epic_frame_received_ && ((translation-epic_frame_translation_).norm()>1.e-6 || std::abs(c-epic_frame_c_)+std::abs(s-epic_frame_s_)>1.e-6)){
        epic_frame_changed_=true;if(epic_switch_.running)stopEpic("PROFILE_FRAME_CHANGED_HOLD");return;
      }
      if(!epic_frame_received_ && (armed_ || !landed_ || age(bridge_rx_)>.6))return;
      epic_frame_translation_=translation;epic_frame_c_=c;epic_frame_s_=s;epic_frame_received_=true;epic_frame_rx_=Steady::now();
      // EpicBounds evaluates PROFILE points. Convert the independent WORLD
      // safety planes once; never compare profile Z directly to world Z.
      epic_bounds_.floor=epic_world_floor_+translation.z();
      epic_bounds_.ceiling=epic_world_ceiling_+translation.z();
      if(std::max(epic_flight_min_,epic_bounds_.floor+.03)>=std::min(epic_flight_max_,epic_bounds_.ceiling-.03)){
        epic_frame_changed_=true;epic_reason_="HEIGHT_BAND_CONFLICT";
        if(epic_switch_.running)stopEpic("HEIGHT_BAND_CONFLICT");
      }
    });
    entry_goal_=create_publisher<nav_msgs::msg::Path>("/zav/super/entry_goal",rclcpp::QoS(1).reliable());
    epic_trigger_=create_publisher<nav_msgs::msg::Path>("/epic/trigger",rclcpp::QoS(1).reliable());
    epic_manual_=create_publisher<std_msgs::msg::Bool>("/mission/st10_manual",rclcpp::QoS(1).reliable());
    epic_status_=create_publisher<std_msgs::msg::String>("/zav/epic/status",rclcpp::QoS(1).reliable().transient_local());
    epic_path_sub_=create_subscription<nav_msgs::msg::Path>("/mission/exploration_path",rclcpp::QoS(1).reliable(),[this](nav_msgs::msg::Path::SharedPtr m){epicGuide(*m);});
    epic_state_sub_=create_subscription<visualization_msgs::msg::Marker>("/epic/state",rclcpp::QoS(10).reliable(),[this](visualization_msgs::msg::Marker::SharedPtr m){
      const double dt=now().seconds()-rclcpp::Time(m->header.stamp).seconds();
      const std::vector<std::string> states={"INIT","WAIT_TRIGGER","PLAN_TRAJ","CAUTION","EXEC_TRAJ","FINISH","LAND","WAIT_MAP","RETURN_HOME"};
      if(dt<-.02 || dt>.5 || std::find(states.begin(),states.end(),m->text)==states.end())return;
      epic_fsm_=m->text;epic_state_rx_=Steady::now();
    });
    epic_done_sub_=create_subscription<std_msgs::msg::Bool>("/mission/exploration_complete",rclcpp::QoS(1).reliable(),[this](std_msgs::msg::Bool::SharedPtr m){if(m->data && epic_switch_.running && !entering_)stopEpic("COMPLETE_HOLD");});
  }

  static double age(Steady::time_point t){return std::chrono::duration<double>(Steady::now()-t).count();}
  bool feedbackFresh(){
    const double odom_age=now().seconds()-state_.timestamp_s,cloud_age=now().seconds()-cloud_stamp_;
    return odom_ok_ && odom_age>=-.02 && odom_age<.25 && cloud_age>=-.02 && cloud_age<.5
      && age(odom_rx_)<.25 && age(cloud_rx_)<.5 && age(bridge_rx_)<.6;
  }
  double heading() const {auto r=state_.q.toRotationMatrix();return std::atan2(r(1,0),r(0,0));}
  void freeze(){hold_.head<3>()=state_.p;hold_[3]=heading();trajectory_.clear();yaw_.clear();mission_=false;
    epic_follower_=argo_super::PathFollower();epic_guide_arrived_=epic_sent_final_=epic_target_final_=epic_route_dirty_=false;
    route_.clear();route_turning_=mission_complete_=pending_plan_=false;mission_kind_="none";intent_="none";}
  void fail(const std::string& reason){if(!fault_)cancelPlanner(reason);fault_=true;phase_=reason;RCLCPP_ERROR(get_logger(),"%s",reason.c_str());}
  std::string admission(bool preempt=false){
    if(!execute_)return "execute_disabled";
    if(!armed_ || landed_)return "armed_airborne_required";
    if(fault_)return "controller_fault:"+phase_;
    if(landing_)return "landing_in_progress";
    if(!active_ || !bridge_active_ || !bridge_healthy_)return "healthy_SUPER_Offboard_required";
    if(!feedbackFresh())return "stale_feedback";
    if(!preempt && epic_switch_.running)return "epic_in_progress";
    if(!preempt && state_.v.norm()>=.15)return "speed_not_stopped";
    if(!preempt && mission_ && !mission_complete_)return "mission_in_progress";
    return "";
  }
  void publishGoal(const argo_super::Waypoint& waypoint,bool keep_reference=false){
    retry_outside_since_=-1.;
    requested_goal_=goal_=waypoint.position;
    geometry_msgs::msg::PoseStamped goal;goal.header.frame_id="world";goal.header.stamp=now();
    goal.pose.position.x=goal_.x();goal.pose.position.y=goal_.y();goal.pose.position.z=goal_.z();
    goal.pose.orientation.w=std::cos(waypoint.yaw/2);goal.pose.orientation.z=std::sin(waypoint.yaw/2);
    goal_request_stamp_ns_=rclcpp::Time(goal.header.stamp).nanoseconds();
    mission_=true;mission_complete_=false;mission_epoch_=rclcpp::Time(goal.header.stamp).seconds();goal_rx_=Steady::now();
    heartbeat_rx_=Steady::now();pending_plan_=true;
    if(!keep_reference){trajectory_.clear();yaw_.clear();}
    settled_at_=0.;
    if(mission_kind_!="epic" && (state_.p-goal_).norm()<goal_position_tolerance_ && state_.v.norm()<goal_speed_tolerance_){
      Eigen::MatrixXd c=Eigen::MatrixXd::Zero(3,8);c.col(7)=goal_;trajectory_.clear();trajectory_.emplace_back(.2,c);
      yaw_=fixedYaw(waypoint.yaw);start_=yaw_start_=now().seconds();pending_plan_=false;
    }else if(entering_){
      nav_msgs::msg::Path entry;entry.header=goal.header;
      auto start=goal;start.pose.position.x=state_.p.x();start.pose.position.y=state_.p.y();start.pose.position.z=state_.p.z();
      entry.poses={start,goal};entry_goal_->publish(entry);
    }else goals_->publish(goal);
    RCLCPP_INFO(get_logger(),"MISSION %s leg %zu/%zu world [%.3f %.3f %.3f], yaw %.3f",mission_kind_.c_str(),leg_+1,route_.size(),goal_.x(),goal_.y(),goal_.z(),waypoint.yaw);
  }
  Trajectory fixedYaw(double value){Eigen::MatrixXd c=Eigen::MatrixXd::Zero(3,8);c(0,7)=value;Trajectory t;t.emplace_back(3600.,c);return t;}
  void patrol(std_srvs::srv::Trigger::Response& response){
    auto reason=admission();response.success=reason.empty();response.message=reason;
    if(!response.success)return;
    freeze();mission_=true;mission_kind_="patrol";leg_=0;
    route_=argo_super::patrol(state_.p,heading());route_turning_=true;turn_yaw_=heading();settled_at_=0.;turn_started_=now().seconds();
    response.message="7m patrol accepted: forward/right/back/left, outward yaw, stop and turn at corners";
  }
  void returnHome(std_srvs::srv::Trigger::Response& response){
    auto reason=admission(true);
    if(reason.empty() && !home_valid_)reason="home_unavailable_start_SUPER_before_takeoff";
    if(reason.empty() && !land_disarm_enabled_)reason="COM_DISARM_LAND_readback_must_be_positive";
    response.success=reason.empty();response.message=reason;if(!response.success)return;
    if(epic_switch_.running)stopEpic("RTH_PREEMPTED");
    const bool keep=trajectory_.getPieceNum()>0 && !mission_complete_ && !route_turning_;
    local_takeoff_=false;route_turning_=false;mission_complete_=false;mission_kind_="rth";leg_=0;settled_at_=0.;
    route_=argo_super::return_home(home_,state_.p,home_yaw_);
    hold_.head<3>()=state_.p;hold_[3]=heading();
    publishGoal(route_[0],keep);
    response.message="RTH accepted: replace current goal with single 3D home +0.4m target, SUPER route, PX4 land";
  }
  template<class Response> void goForward(double distance,Response& response){
    response.success=false;
    if(!std::isfinite(distance) || distance<=0. || distance>20.){
      response.message="distance_m must be in (0,20]";return;
    }
    response.message=admission();response.success=response.message.empty();
    if(!response.success)return;
    const double yaw=heading();
    freeze();mission_kind_="forward";leg_=0;
    route_={{state_.p+distance*Eigen::Vector3d(std::cos(yaw),std::sin(yaw),0.),yaw}};
    publishGoal(route_[0]);
    std::ostringstream accepted;accepted<<distance<<"m SUPER goal accepted";response.message=accepted.str();
    RCLCPP_INFO(get_logger(),"%s: world [%.3f, %.3f, %.3f]",response.message.c_str(),goal_.x(),goal_.y(),goal_.z());
  }
  void odometry(const nav_msgs::msg::Odometry& m){
    if(m.header.frame_id!="world" || m.child_frame_id!="argo_body") return;
    const auto &p=m.pose.pose.position;const auto&q=m.pose.pose.orientation;const auto&v=m.twist.twist.linear;const auto&w=m.twist.twist.angular;
    ommpc_core::ControllerState x;x.p={p.x,p.y,p.z};x.q={q.w,q.x,q.y,q.z};x.v={v.x,v.y,v.z};x.w={w.x,w.y,w.z};x.timestamp_s=rclcpp::Time(m.header.stamp).seconds();
    const double stamp_age=now().seconds()-x.timestamp_s;
    if(odom_ok_ && x.timestamp_s+1.<last_stamp_){home_valid_=false;fail("ODOMETRY_CLOCK_RESET");return;}
    if(!x.p.allFinite() || !x.v.allFinite() || !x.w.allFinite() || !x.q.coeffs().allFinite() || x.q.norm()<.9 || x.q.norm()>1.1 || stamp_age<-.02 || stamp_age>.25 || x.timestamp_s<=last_stamp_)return;
    if(odom_ok_ && (active_ || home_locked_) && (x.p-state_.p).norm()>.5+20.*(x.timestamp_s-last_stamp_)){home_valid_=false;fail("ODOMETRY_JUMP");return;}
    x.q.normalize();state_=x;last_stamp_=x.timestamp_s;odom_ok_=true;odom_rx_=Steady::now();
  }
  void bridge(const std_msgs::msg::String& m){
    try{
      const auto b=YAML::Load(m.data);const auto off=b["offboard"];
      if(epic_enabled_){
        epic_hover_ready_=off["hover_ready"].as<bool>(false);
        const auto t=b["telemetry"];const auto rc=t["rc7"];
        epic_rc7_=rc["pwm"].as<int>(0);
        const double rc_age=rc["age_s"].as<double>(999.);epic_rc_age_=rc_age;
        epic_rc_fresh_=rc["fresh"].as<bool>(false) && rc_age>=0. && rc_age<.5;
        epic_airborne_=b["landed_fresh"].as<bool>(false) && !b["landed"].as<bool>(true);
        const auto bat=t["battery"];
        const int pct=bat["remaining_pct"].as<int>(-1);
        epic_battery_ok_=t["battery_fresh"].as<bool>(false) && bat["charge_state"].as<int>(0)==1 && pct>20 && pct<=100;
      }
      const bool next_armed=b["armed"].as<bool>(true);
      landed_=b["landed_fresh"].as<bool>(false) && b["landed"].as<bool>(false);
      land_disarm_enabled_=off["auto_land_disarm_s"].as<double>(0.)>0.;
      if(!next_armed && landed_ && odom_ok_ && now().seconds()-state_.timestamp_s<.25){
        seen_disarmed_=true;
        if(!home_locked_){home_=state_.p;home_yaw_=heading();home_sample_at_=now().seconds();}
      }
      if(next_armed && !armed_){home_valid_=seen_disarmed_ && now().seconds()-home_sample_at_<1. && odom_ok_ && now().seconds()-state_.timestamp_s<.25;home_locked_=home_valid_;}
      if(!next_armed && armed_ && !landing_)home_locked_=false;
      armed_=next_armed;
      const bool mode=off["fc_mode"].IsSequence() && off["fc_mode"].size()==2 && off["fc_mode"][0].as<int>()==6;
      bridge_healthy_=b["ready"].as<bool>() && !b["latched"].as<bool>() && off["source"].as<std::string>("")=="super" && !off["stopped"].as<bool>();
      const std::string ground=off["rc_ground_stage"].as<std::string>("IDLE");
      // Track measured hover in PRESTREAM while PX4 owns the local position target.
      px4_hover_handoff_=ground=="HANDOFF";
      const bool ommpc_owner=ground=="IDLE" || ground=="HANDOFF" || ground=="COMPLETE";
      const bool running=mode && bridge_healthy_ && ommpc_owner;
      if(!auto_takeoff_ && !next_armed && landed_){takeoff_started_=false;}
      if(!auto_takeoff_ && local_takeoff_ && (!running || !next_armed)){local_takeoff_=false;freeze();}
      if(bridge_active_ && !running){cancelPlanner("OFFBOARD_EXIT");active_=false;if(odom_ok_ && !landing_)freeze();}
      bridge_active_=running;bridge_rx_=Steady::now();
    }catch(const YAML::Exception&){bridge_healthy_=false;}
  }
  void arrived(const std_msgs::msg::String& msg){
    if(!pending_plan_ || !active_ || !mission_ || fault_ || landing_ || !feedbackFresh())return;
    try{
      const auto event=YAML::Load(msg.data);
      if(event["event"].as<std::string>()!="GOAL_ARRIVED" ||
         event["goal_request_stamp_ns"].as<int64_t>()!=goal_request_stamp_ns_)return;
      const auto g=event["accepted_goal"];
      if(!g.IsSequence() || g.size()!=3)return;
      const Eigen::Vector3d accepted(g[0].as<double>(),g[1].as<double>(),g[2].as<double>());
      if(!accepted.allFinite() || (accepted-requested_goal_).norm()>goal_snap_max_distance_+1e-6 ||
         (state_.p-accepted).norm()>=goal_position_tolerance_ || state_.v.norm()>=goal_speed_tolerance_ ||
         (mission_kind_=="epic" && !insideEpic(accepted)))return;
      Eigen::MatrixXd hold=Eigen::MatrixXd::Zero(3,8);hold.col(7)=accepted;
      goal_=accepted;trajectory_.clear();trajectory_.emplace_back(.2,hold);
      yaw_=fixedYaw(mission_kind_=="epic"?last_goal_yaw_:route_.at(leg_).yaw);
      start_=yaw_start_=now().seconds();pending_plan_=false;heartbeat_rx_=Steady::now();
    }catch(const YAML::Exception&){}
  }
  void trajectory(const Poly& m){
    if(!active_ || !mission_ || mission_complete_ || local_takeoff_ || route_turning_ || landing_ || fault_ || m.header.frame_id!="world")return;
    const double generated=rclcpp::Time(m.header.stamp).seconds();
    if(generated<mission_epoch_ || generated>now().seconds()+.1 || now().seconds()-generated>.5)return;
    if(m.type & Poly::EMER_STOP){fail("SUPER_EMERGENCY");return;}
    if(!(m.type & Poly::POSITION_TRAJ)){
      const bool current_hold=!epic_guide_arrived_ || m.start_wt_pos==start_;
      if((m.type & Poly::HEART_BEAT) && current_hold)heartbeat_rx_=Steady::now();
      return;
    }
    try{
      auto p=argo_super::decode(m);Trajectory y;
      if(!super_utils::certifyKinematics(p,trajectory_max_vel_,trajectory_max_acc_,trajectory_max_jerk_))
        throw std::invalid_argument("continuous trajectory speed/acceleration/jerk limit");
      if(mission_kind_=="epic"){
        const double T=p.getTotalDuration();
        if(T>120.)throw std::invalid_argument("EPIC local trajectory too long");
        auto bounds=epic_bounds_;
        bounds.floor=std::max(bounds.floor,epic_flight_min_);
        bounds.ceiling=std::min(bounds.ceiling,epic_flight_max_);
        int budget=10000;
        auto choose=[](int n,int k){double value=1.;for(int j=1;j<=k;++j)value=value*(n-j+1)/j;return value;};
        for(int piece=0;piece<p.getPieceNum();++piece){
          const auto c=p[piece].getCoeffMat();const double duration=p[piece].getDuration();
          std::array<argo_super::EpicPoint,8> controls;
          for(int k=0;k<8;++k){
            Eigen::Vector3d v=Eigen::Vector3d::Zero();
            for(int i=0;i<=k;++i)v+=c.col(7-i)*std::pow(duration,i)*choose(k,i)/choose(7,i);
            if(entering_ && !entry_corridor_.contains(v))
              throw std::invalid_argument("EPIC entry corridor exceeded");
            controls[k]=point(v);
          }
          if(!entering_ && !bounds.bezierInside(controls,budget))throw std::invalid_argument("EPIC polynomial boundary cannot be certified");
        }
      }
      if(m.type & Poly::YAW_TRAJ)y=argo_super::decode(m,true);
      // A retransmission may be in the ROS queue as its executed reference
      // expires. It cannot extend that reference or invalidate its settlement.
      if(!pending_plan_ && committed_request_stamp_ns_==goal_request_stamp_ns_ &&
         m.start_wt_pos==start_ && m.start_wt_pos+p.getTotalDuration()<=now().seconds() &&
         YAML::Load(m.debug_info)["goal_request_stamp_ns"].as<int64_t>()==committed_request_stamp_ns_)return;
      if(!std::isfinite(m.start_wt_pos) || m.start_wt_pos<mission_epoch_-.1 || m.start_wt_pos>now().seconds()+1. || m.start_wt_pos+p.getTotalDuration()<now().seconds())throw std::invalid_argument("stale/future plan");
      const auto metadata=YAML::Load(m.debug_info);
      if (!metadata.IsMap() || metadata["goal_request_stamp_ns"].as<int64_t>()!=goal_request_stamp_ns_)
        throw std::invalid_argument("missing/mismatched planner goal request");
      const auto g=metadata["accepted_goal"];
      if (!g.IsSequence() || g.size()!=3) throw std::invalid_argument("invalid accepted planner goal");
      const Eigen::Vector3d accepted(g[0].as<double>(),g[1].as<double>(),g[2].as<double>());
      if (!accepted.allFinite() || (accepted-requested_goal_).norm()>goal_snap_max_distance_+1e-6)
        throw std::invalid_argument("accepted planner goal outside snap limit");
      if(entering_ && (!insideEpic(accepted) || point(accepted)[2]<epic_flight_min_+.03-1e-6 ||
                       point(accepted)[2]>epic_flight_max_-.03+1e-6))
        throw std::invalid_argument("EPIC entry endpoint outside flight band");
      if ((goal_-accepted).norm()>1e-6) settled_at_=0.;
      goal_=accepted;
      committed_request_stamp_ns_=goal_request_stamp_ns_;trajectory_=p;yaw_=mission_kind_=="patrol"?fixedYaw(route_[leg_].yaw):y;
      start_=m.start_wt_pos;yaw_start_=m.start_wt_yaw;pending_plan_=false;heartbeat_rx_=Steady::now();
    }catch(const std::exception&e){
      RCLCPP_WARN(get_logger(),"Rejected SUPER plan: %s",e.what());
      // Correlate feedback with the exact outstanding request and packet.
      // Neither this message nor a replacement guide resets its timeout.
      try{
        const auto metadata=YAML::Load(m.debug_info);
        const auto request=metadata["goal_request_stamp_ns"].as<int64_t>();
        if(request==goal_request_stamp_ns_ && std::isfinite(m.start_wt_pos)){
          std::ostringstream out;out<<std::setprecision(17)<<"{\"event\":\"REJECTED\",\"stamp_ns\":"<<now().nanoseconds()
            <<",\"goal_request_stamp_ns\":"<<request<<",\"start_wt_pos\":"<<m.start_wt_pos<<"}";
          std_msgs::msg::String feedback;feedback.data=out.str();trajectory_feedback_->publish(feedback);
        }
      }catch(const std::exception&){}
    }
  }
  void tick(){
    if(!feedbackFresh() || mission_ || landing_ || fault_ || !bridge_active_)stopPilot();
    epicTick();
    ommpc_core::ControlCommand out;bool valid=false;
    Eigen::Vector4d px4_target=hold_;
    Eigen::Vector3d px4_velocity=Eigen::Vector3d::Zero();
    const bool fresh=feedbackFresh();
    if(landing_){
      phase_=!armed_ && landed_?"RTH_LANDED":"RTH_LANDING";
      intent_=armed_?"land":"none";
      if(bridge_active_ && fresh && !fault_){
        Eigen::Vector4d target;target.head<3>()=goal_;target[3]=home_yaw_;
        if(px4_tracking_){px4_target=target;valid=target.allFinite();}
        else if(mpc_.setHoverReference(target) && mpc_.execMPC(state_,state_.timestamp_s,out)
            && out.bodyrates.allFinite() && std::isfinite(out.collective_acceleration))valid=true;
      }
    }else if(!fault_ && fresh){
      if(bridge_active_ && !active_){
        // Preserve the last Position/prestream hold. The FC mode feedback is
        // delayed; recapturing here would accept a height lost during transfer.
        active_=true;
        RCLCPP_INFO(get_logger(),"Offboard hold retained: %.3f %.3f %.3f m",hold_[0],hold_[1],hold_[2]);
      }
      if(auto_takeoff_ && !takeoff_started_){
        phase_="TAKEOFF_WAIT";mission_kind_="takeoff";hold_.head<3>()=state_.p;hold_[3]=heading();
        if(auto_takeoff_ && age(created_)>45.)fail("TAKEOFF_PREPARATION_TIMEOUT");
        if(armed_ && bridge_active_ && home_valid_){
          takeoff_started_=local_takeoff_=mission_=true;mission_complete_=false;settled_at_=0.;
          goal_=home_+Eigen::Vector3d(0.,0.,takeoff_height_);hold_.head<3>()=home_;hold_[3]=home_yaw_;
          Eigen::MatrixXd c=Eigen::MatrixXd::Zero(3,8);c.col(7)=home_;const double T=3.;
          c(2,3)=35*takeoff_height_/std::pow(T,4);c(2,2)=-84*takeoff_height_/std::pow(T,5);
          c(2,1)=70*takeoff_height_/std::pow(T,6);c(2,0)=-20*takeoff_height_/std::pow(T,7);
          trajectory_.clear();trajectory_.emplace_back(T,c);yaw_=fixedYaw(home_yaw_);start_=now().seconds()+1.;yaw_start_=start_;
        }
      }else if(!bridge_active_) {freeze();phase_="PRESTREAM";}
      else phase_=mission_?"PLANNING":"HOVER";
      const double t=state_.timestamp_s;
      auto tracking_state=state_;
      bool reference=false;
      if(route_turning_){
        phase_="PATROL_TURN";
        if(now().seconds()-turn_started_>30.)fail("PATROL_YAW_TIMEOUT");
        const double error=argo_super::angle(route_[leg_].yaw-turn_yaw_);
        turn_yaw_+=std::clamp(error,-.4*period_,.4*period_);hold_[3]=turn_yaw_;
        px4_target=hold_;
        reference=px4_tracking_ || mpc_.setHoverReference(hold_);
        if(std::abs(argo_super::angle(heading()-route_[leg_].yaw))<.0873 && state_.v.norm()<goal_speed_tolerance_){
          if(settled_at_==0.)settled_at_=now().seconds();
          if(now().seconds()-settled_at_>1.){route_turning_=false;settled_at_=0.;publishGoal(route_[leg_]);}
        }else settled_at_=0.;
      }else if(mission_ && trajectory_.getPieceNum()>0){
        if(local_takeoff_ && ((state_.p.head<2>()-home_.head<2>()).norm()>.4 || state_.p.z()>goal_.z()+.3))fail("TAKEOFF_POSITION_ENVELOPE");
        if(local_takeoff_ && now().seconds()-start_>15.)fail("TAKEOFF_HEIGHT_TIMEOUT");
        const double elapsed=std::max(0.,t-start_);
        const bool stopped=argo_super::terminal_stop_reached(trajectory_,elapsed,state_.p,state_.v);
        // A slow replacement keeps the same request and the committed reference.
        // Only a genuine stopping endpoint may be held after its own expiry.
        if(!local_takeoff_ && pending_plan_ &&
           !argo_super::can_wait_for_replacement(trajectory_,elapsed,state_.p,state_.v))
          fail("REPLACEMENT_REFERENCE_EXPIRED");
        if(!local_takeoff_ && pending_plan_ && age(goal_rx_)>replacement_timeout_s_)fail("REPLACEMENT_PLAN_TIMEOUT");
        if(!local_takeoff_ && !pending_plan_ && age(heartbeat_rx_)>1. && !stopped)fail("PLANNER_TIMEOUT");
        if(!fault_ && px4_tracking_){
          const double time=now().seconds();
          const double sample=std::clamp(time-start_,0.,trajectory_.getTotalDuration());
          px4_target.head<3>()=trajectory_.getPos(sample);
          if(time>=start_ && time<start_+trajectory_.getTotalDuration())px4_velocity=trajectory_.getVel(sample);
          px4_target[3]=hold_[3];
          if(yaw_.getPieceNum()>0){
            const double yaw_elapsed=mission_kind_=="patrol"?sample:time-yaw_start_;
            px4_target[3]=yaw_.getPos(std::clamp(yaw_elapsed,0.,yaw_.getTotalDuration()))[0];
          }
          reference=px4_target.allFinite() && px4_velocity.allFinite();
        }else if(!fault_){
          // Align only the MPC tracking input and reference. Mission guards use the measurement.
          double preview=0.;
          if(tracking_prediction_limit_s_>0. && !local_takeoff_ && !stopped && t>=start_){
            const double remaining=std::max(0.,trajectory_.getTotalDuration()-elapsed);
            preview=std::clamp(now().seconds()-t,0.,std::min(remaining,tracking_prediction_limit_s_));
            tracking_state.p+=state_.v*preview;
            const double rate=state_.w.norm();
            if(rate>1e-9)
              tracking_state.q=(state_.q*Eigen::AngleAxisd(rate*preview,state_.w/rate)).normalized();
            tracking_state.timestamp_s+=preview;
          }
          reference=mpc_.setTrajectoryReference(trajectory_,elapsed+preview,hold_[3],yaw_,tracking_state,mission_kind_=="patrol"?0.:start_-yaw_start_);
        }
        phase_=fault_?phase_:local_takeoff_?"TAKEOFF_CLIMB":pending_plan_?"REPLANNING":"TRACKING";
        if(!fault_ && stopped){
          const bool at_goal=(state_.p-goal_).norm()<(local_takeoff_?.2:goal_position_tolerance_) &&
              state_.v.norm()<(local_takeoff_?.15:goal_speed_tolerance_) && !pending_plan_;
          const auto endpoint=trajectory_.getPos(trajectory_.getTotalDuration());
          phase_=at_goal?"ENDPOINT_HOLD":(endpoint-goal_).norm()<goal_position_tolerance_?"ENDPOINT_SETTLING":"BACKUP_STOP";
          if(at_goal){
            if(settled_at_==0.)settled_at_=now().seconds();
            if((mission_kind_=="epic" || now().seconds()-settled_at_>1.) && !mission_complete_){
              if(local_takeoff_){local_takeoff_=mission_=false;mission_complete_=true;if(!auto_takeoff_)mission_kind_="none";hold_.head<3>()=goal_;hold_[3]=home_yaw_;trajectory_.clear();phase_="HOVER";}
              else if(mission_kind_=="epic"){if(epic_sent_final_ && !epic_route_dirty_)epic_guide_arrived_=true;}
              else if(leg_+1<route_.size()){
                ++leg_;settled_at_=0.;
                if(mission_kind_=="patrol"){
                  route_turning_=true;turn_yaw_=heading();hold_.head<3>()=state_.p;hold_[3]=turn_yaw_;trajectory_.clear();turn_started_=now().seconds();
                }else {hold_.head<3>()=state_.p;hold_[3]=heading();publishGoal(route_[leg_]);}
              }else if(mission_kind_=="rth") {landing_=true;intent_="land";phase_="RTH_LANDING";}
              else mission_complete_=true;
            }
          }else settled_at_=0.;
        }else settled_at_=0.;
      }else{
        updatePilot();
        px4_target=hold_;
        reference=px4_tracking_ || mpc_.setHoverReference(hold_);
        if(mission_ && age(goal_rx_)>replacement_timeout_s_)fail("NO_PLAN");
      }
      if(mission_complete_ && mission_kind_=="takeoff")phase_="HOVER";
      if(mission_ && !local_takeoff_ && !route_turning_ && age(goal_rx_)>120. && !mission_complete_)fail("MISSION_SEGMENT_TIMEOUT");
      if(!fault_ && reference && (px4_tracking_ || (mpc_.execMPC(tracking_state,tracking_state.timestamp_s,out) && out.bodyrates.allFinite() && std::isfinite(out.collective_acceleration)))){
        valid=true;failures_=0;
      }else if(!fault_ && ++failures_>=3)fail("SOLVER_FAILURE");
    }else if(!fault_)phase_="FEEDBACK_WAIT";
    std::ostringstream json;json<<std::setprecision(17)<<"{\"session\":\""<<session_<<"\",\"sequence\":"<<++sequence_
      <<",\"stamp_ns\":"<<now().nanoseconds()<<",\"valid\":"<<(valid&&!fault_?"true":"false")
      <<",\"rates_flu\":["<<out.bodyrates.x()<<","<<out.bodyrates.y()<<","<<out.bodyrates.z()<<"]"
      <<",\"acceleration\":"<<out.collective_acceleration<<",\"phase\":\""<<phase_<<"\",\"solver_ms\":"<<(px4_tracking_?0.:mpc_.getTimingMs())
      <<",\"position\":["<<state_.p.x()<<","<<state_.p.y()<<","<<state_.p.z()<<"]"
      <<",\"goal\":["<<goal_.x()<<","<<goal_.y()<<","<<goal_.z()<<"]}";
    auto text=json.str();text.pop_back();
    if(px4_tracking_){
      std::ostringstream ref;ref<<std::setprecision(17)<<",\"tracking_backend\":\"px4\",\"px4_reference\":["
        <<px4_target.x()<<","<<px4_target.y()<<","<<px4_target.z()<<","<<px4_target[3]<<","
        <<px4_velocity.x()<<","<<px4_velocity.y()<<","<<px4_velocity.z()<<"],\"measured_yaw\":"<<(odom_ok_?heading():0.);
      text+=ref.str();
    }
    std::ostringstream meta;meta<<std::setprecision(17)<<",\"intent\":\""<<intent_<<"\",\"mission\":{\"kind\":\""<<mission_kind_<<"\",\"leg\":"<<(route_.empty()?0:leg_+1)
      <<",\"legs\":"<<route_.size()<<",\"complete\":"<<(mission_complete_?"true":"false")<<",\"home_valid\":"<<(home_valid_?"true":"false")
      <<",\"px4_hover_acquired\":"<<((px4_hover_handoff_ && active_ && phase_=="HOVER" && valid)?"true":"false")
      <<",\"rc_ground_seed\":"<<((!auto_takeoff_ && !armed_ && landed_ && seen_disarmed_ && feedbackFresh() &&
          (!epic_frame_required_ || (epic_frame_received_ && !epic_frame_changed_ && age(epic_frame_rx_)<.6)) &&
          home_.z()+takeoff_height_>=epic_world_floor_+.03 && home_.z()+takeoff_height_<=epic_world_ceiling_-.03)?rc_ground_seed_:0.)
      <<",\"requested_goal\":["<<requested_goal_.x()<<","<<requested_goal_.y()<<","<<requested_goal_.z()<<"]"
      <<",\"home\":["<<home_.x()<<","<<home_.y()<<","<<home_.z()<<"],\"home_yaw\":"<<home_yaw_<<"}}";
    std_msgs::msg::String msg;msg.data=text+meta.str();if(execute_)commands_->publish(msg);
    msg.data.pop_back();
    msg.data+=",\"odom_stamp_ns\":"+std::to_string(static_cast<int64_t>(state_.timestamp_s*1e9))
      +",\"pilot_ready\":"+std::string(pilot_ready_?"true":"false")+",\"hold_target\":["+std::to_string(hold_[0])+","+std::to_string(hold_[1])+","+std::to_string(hold_[2])+","+std::to_string(hold_[3])+"]"
      +",\"R_pitchroll\":"+std::to_string(active_r_pitchroll_)+"}";
    status_->publish(msg);
  }
public:
  HardwareController():Node("zav_super_ommpc"){
    if(get_parameter("use_sim_time").as_bool())throw std::invalid_argument("Hardware requires use_sim_time=false");
    execute_=declare_parameter<bool>("execute",false);distance_=declare_parameter<double>("distance_m",10.);
    const auto backend=declare_parameter<std::string>("tracking_backend","ommpc");
    if(backend!="px4" && backend!="ommpc")throw std::invalid_argument("invalid tracking_backend");
    px4_tracking_=backend=="px4";
    rc_ground_seed_=declare_parameter<double>("rc_ground_hover_thrust",0.);
    auto_takeoff_=declare_parameter<bool>("auto_takeoff",false);takeoff_height_=declare_parameter<double>("takeoff_height",.4);
    if(!std::isfinite(takeoff_height_) || takeoff_height_<.3 || takeoff_height_>2.)throw std::invalid_argument("Takeoff height must be 0.3..2m");
    if(!std::isfinite(distance_) || distance_<=0. || distance_>20.)throw std::invalid_argument("distance_m must be in (0,20]");
    const auto lock_path="/tmp/zav-super-controller-"+std::to_string(getuid())+".lock";
    lock_=open(lock_path.c_str(),O_CREAT|O_RDWR,0600);if(lock_<0 || flock(lock_,LOCK_EX|LOCK_NB))throw std::runtime_error("SUPER controller already running");
    session_="super-"+std::to_string(getpid())+"-"+std::to_string(now().nanoseconds());
    ommpc_core::ParameterSet p;
    auto par=[this](const std::string& n,double v){return declare_parameter<double>(n,v);};
    trajectory_max_vel_=par("trajectory_max_vel",1.);
    trajectory_max_acc_=par("trajectory_max_acc",6.);
    trajectory_max_jerk_=par("trajectory_max_jerk",10.);
    if(!std::isfinite(trajectory_max_vel_) || !std::isfinite(trajectory_max_acc_) || !std::isfinite(trajectory_max_jerk_) ||
       trajectory_max_vel_<=0. || trajectory_max_acc_<=0. || trajectory_max_jerk_<=0.)throw std::invalid_argument("Invalid trajectory limits");
    replacement_timeout_s_=par("planner_response_timeout_s",5.);
    epic_guide_wait_timeout_s_=par("epic_guide_wait_timeout_s",20.);
    if(!std::isfinite(replacement_timeout_s_) || replacement_timeout_s_<=0. || replacement_timeout_s_>60. ||
       !std::isfinite(epic_guide_wait_timeout_s_) || epic_guide_wait_timeout_s_<=replacement_timeout_s_ || epic_guide_wait_timeout_s_>120.)
      throw std::invalid_argument("Invalid planner/guide wait timeouts");
    goal_position_tolerance_=par("goal_position_tolerance",.1);
    terminal_settle_time_=par("terminal_settle_time_s",3.);
    goal_speed_tolerance_=par("goal_speed_tolerance",.1);
    goal_snap_max_distance_=par("goal_snap_max_distance",.3);
    if (!std::isfinite(terminal_settle_time_) || terminal_settle_time_<0. || !std::isfinite(goal_position_tolerance_) || goal_position_tolerance_<=0. || goal_position_tolerance_>.3 ||
        !std::isfinite(goal_speed_tolerance_) || goal_speed_tolerance_<=0. ||
        !std::isfinite(goal_snap_max_distance_) || goal_snap_max_distance_<0.)
      throw std::invalid_argument("Invalid planner goal tolerances");
    tracking_prediction_limit_s_=par("tracking_prediction_limit_s",0.);
    if(!std::isfinite(tracking_prediction_limit_s_) || tracking_prediction_limit_s_<0. || tracking_prediction_limit_s_>.04)
      throw std::invalid_argument("tracking_prediction_limit_s must be in [0, 0.04]");
    period_=par("control_period",.02);step_=p.mpc.step_T=par("step_T",.02);
    if(!std::isfinite(period_) || period_<.01 || !std::isfinite(step_) || step_<=0.)throw std::invalid_argument("Invalid OMMPC timing");
    p.mpc.Q_pos_xy=par("Q_pos_xy",500.);p.mpc.Q_pos_z=par("Q_pos_z",800.);p.mpc.Q_velocity=par("Q_velocity",40.);
    p.mpc.Q_attitude_rp=par("Q_attitude_rp",80.);p.mpc.Q_attitude_yaw=par("Q_attitude_yaw",80.);
    p.mpc.R_thrust=par("R_thrust",.5);p.mpc.R_pitchroll=par("R_pitchroll",4.);p.mpc.R_yaw=par("R_yaw",2.);
    p.mpc.state_cost_exponential=par("state_cost_exponential",.5);p.mpc.input_cost_exponential=par("input_cost_exponential",.5);
    p.mpc.max_bodyrate_xy=par("max_bodyrate_xy",2.);p.mpc.max_bodyrate_z=par("max_bodyrate_z",1.);
    p.mpc.min_thrust=par("min_thrust",1.);p.mpc.max_thrust=par("max_thrust",30.);
    p.controller.use_fix_yaw=false;p.controller.velocity_frame=ommpc_core::VelocityFrame::kWorld;
    p.controller.thrust_normalization.hover_percentage=.18; // Only collective_acceleration is transmitted; bridge normalizes with FC readback.
    if(!px4_tracking_)mpc_.init(p);
    active_r_pitchroll_=p.mpc.R_pitchroll;
    commands_=create_publisher<std_msgs::msg::String>("/zav/super/rates",rclcpp::QoS(1).reliable());
    status_=create_publisher<std_msgs::msg::String>("/zav/super/controller_status",10);
    planner_cancel_=create_publisher<std_msgs::msg::String>("/zav/super/planner_cancel",rclcpp::QoS(1).reliable().transient_local());
    trajectory_feedback_=create_publisher<std_msgs::msg::String>("/zav/super/trajectory_feedback",rclcpp::QoS(10).reliable());
    goals_=create_publisher<geometry_msgs::msg::PoseStamped>("/zav/super/goal",rclcpp::QoS(1).reliable());
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/argo/super/odom",rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::SharedPtr m){odometry(*m);});
    cloud_sub_=create_subscription<sensor_msgs::msg::PointCloud2>("/argo/super/cloud_world",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::PointCloud2::SharedPtr m){
      const double stamp=rclcpp::Time(m->header.stamp).seconds(),dt=now().seconds()-stamp;
      if(m->header.frame_id=="world" && m->width*m->height>0 && dt>=-.02 && dt<.5){cloud_stamp_=stamp;cloud_rx_=Steady::now();}
    });
    bridge_sub_=create_subscription<std_msgs::msg::String>("/zav_real/status",rclcpp::QoS(1).reliable().transient_local(),[this](std_msgs::msg::String::SharedPtr m){bridge(*m);});
    planner_status_sub_=create_subscription<std_msgs::msg::String>("/zav/super/planner_status",10,[this](std_msgs::msg::String::SharedPtr m){arrived(*m);});
    traj_sub_=create_subscription<Poly>("/planning_cmd/poly_traj",rclcpp::SensorDataQoS(),[this](Poly::SharedPtr m){trajectory(*m);});
    go_=create_service<std_srvs::srv::Trigger>("/zav/super/go_front",[this](std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr response){
      goForward(distance_,*response);
    });
    go_distance_=create_service<zav_super::srv::GoForward>("/zav/super/go_forward",[this](zav_super::srv::GoForward::Request::SharedPtr request,zav_super::srv::GoForward::Response::SharedPtr response){
      goForward(request->distance_m,*response);
    });
    mission_services_.push_back(create_service<std_srvs::srv::Trigger>("/zav/super/patrol",[this](std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr r){patrol(*r);}));
    mission_services_.push_back(create_service<std_srvs::srv::Trigger>("/zav/super/return_home",[this](std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr r){returnHome(*r);}));
    setupEpic();
    timer_=create_wall_timer(std::chrono::duration<double>(period_),[this]{tick();});
    RCLCPP_INFO(get_logger(),"SUPER OMMPC: execute=%s, goal=%.1fm. No FC arm/mode/takeoff APIs.",execute_?"true":"false",distance_);
  }
  ~HardwareController(){if(lock_>=0)close(lock_);}
};
int main(int argc,char**argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<HardwareController>());}catch(const std::exception&e){std::cerr<<e.what()<<std::endl;rclcpp::shutdown();return 1;}rclcpp::shutdown();}
