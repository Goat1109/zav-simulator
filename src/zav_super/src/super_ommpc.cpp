// Hardware wrapper around the unchanged argo_v2 OMMPC core. No FC transport,
// arm, takeoff, mode-change or land commands. The EV bridge owns the UART.
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/bool.hpp>
#include <zav_super/epic_gate.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <zav_super/srv/go_forward.hpp>
#include <ommpc_core/controller_core.hpp>
#include <zav_super/trajectory.hpp>
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
  int64_t goal_request_stamp_ns_=0;
  double goal_position_tolerance_=.1,goal_speed_tolerance_=.1,goal_snap_max_distance_=.3;
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
  bool auto_takeoff_=false,local_takeoff_=false,takeoff_started_=false,land_disarm_enabled_=false;
  std::string mission_kind_="none",intent_="none";
  std::vector<argo_super::Waypoint> route_;
  size_t leg_=0;

  bool epic_frame_required_=false,epic_frame_received_=false,epic_frame_changed_=false;
  double epic_frame_c_=1.,epic_frame_s_=0.;
  Eigen::Vector3d epic_frame_translation_=Eigen::Vector3d::Zero();
  Steady::time_point epic_frame_rx_{};
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr epic_frame_sub_;
  bool epic_hover_ready_=false,epic_enabled_=false,epic_airborne_=false,epic_battery_ok_=false,epic_rc_fresh_=false;
  int epic_rc7_=0;
  argo_super::EpicSwitch epic_switch_;
  argo_super::EpicBounds epic_bounds_;
  std::string epic_profile_,epic_fsm_="UNKNOWN",epic_reason_="WAIT_READY";
  Steady::time_point epic_state_rx_{},epic_wait_rx_{},epic_status_tx_{};
  int64_t epic_epoch_ns_=0,epic_guide_stamp_ns_=0;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr epic_trigger_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr epic_manual_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr epic_status_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr epic_path_sub_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr epic_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr epic_done_sub_;
  std::vector<argo_super::Waypoint> epic_pending_;
  Steady::time_point epic_pending_rx_{};
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
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr go_;
  rclcpp::Service<zav_super::srv::GoForward>::SharedPtr go_distance_;

  argo_super::EpicPoint point(const Eigen::Vector3d& p) const {
    return {epic_frame_c_*p.x()-epic_frame_s_*p.y()+epic_frame_translation_.x(),
            epic_frame_s_*p.x()+epic_frame_c_*p.y()+epic_frame_translation_.y(),p.z()+epic_frame_translation_.z()};
  }
  bool insideEpic(const Eigen::Vector3d& p,double margin=0.)const{
    return p.z()>=epic_bounds_.floor-margin && p.z()<=epic_bounds_.ceiling+margin && epic_bounds_.contains(point(p),margin);
  }
  void stopEpic(const std::string& reason){
    epic_switch_.stop();epic_reason_=reason;epic_pending_.clear();
    if(mission_kind_=="epic"){freeze();phase_="HOVER";}
    std_msgs::msg::Bool manual;manual.data=true;epic_manual_->publish(manual);
  }
  void epicGuide(const nav_msgs::msg::Path& path){
    if(!epic_switch_.running || !admission(true).empty())return;
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
      if(i==0 && (p-state_.p).norm()>1.){reject();return;}
      if(!epic_bounds_.segment(point(previous),point(p))){reject();return;}
      length+=(p-previous).norm();if(length>250.){reject();return;}
      const double yaw=std::atan2(2.*(q.w*q.z+q.x*q.y),1.-2.*(q.y*q.y+q.z*q.z));
      route.push_back({p,yaw});previous=p;
    }
    epic_guide_stamp_ns_=stamp;
    epic_pending_=std::move(route);epic_pending_rx_=Steady::now();
    // Updating EPIC's preview must not reset an executing local trajectory.
    if(mission_kind_!="epic" || mission_complete_)acceptEpicPending();
  }
  bool acceptEpicPending(){
    if(epic_pending_.empty() || age(epic_pending_rx_)>1.){epic_pending_.clear();return false;}
    size_t first=0;
    while(first<epic_pending_.size() && (epic_pending_[first].position-state_.p).norm()<.2)++first;
    if(first==epic_pending_.size()){epic_pending_.clear();return false;}
    if(!epic_bounds_.segment(point(state_.p),point(epic_pending_[first].position))){epic_pending_.clear();return false;}
    route_.assign(epic_pending_.begin()+first,epic_pending_.end());epic_pending_.clear();
    mission_kind_="epic";leg_=0;settled_at_=0.;epic_reason_="FOLLOW_GUIDE";
    publishGoal(route_[0]);return true;
  }
  void epicTick(){
    if(!epic_enabled_)return;
    const bool live=age(bridge_rx_)<.6 && epic_rc_fresh_ && epic_rc_age_+age(bridge_rx_)<.5;
    const bool node=age(epic_state_rx_)<1.5;
    const bool bounds=odom_ok_ && insideEpic(state_.p,.2);
    const bool frame=!epic_frame_required_ || (epic_frame_received_ && !epic_frame_changed_ && age(epic_frame_rx_)<.6);
    const bool safe=node && live && frame && epic_airborne_ && epic_battery_ok_ && bounds && admission(true).empty() &&
      (mission_kind_=="none" || mission_kind_=="epic" || mission_complete_);
    const bool ready=safe && epic_hover_ready_ && epic_fsm_=="WAIT_TRIGGER" && admission().empty() &&
      (phase_=="HOVER" || phase_=="ENDPOINT_HOLD");
    const auto event=epic_switch_.update(std::chrono::duration<double>(Steady::now().time_since_epoch()).count(),live,epic_rc7_,ready,safe);
    if(event==argo_super::EpicSwitch::STOP){
      stopEpic(!live?"RC7_LOST_HOLD":!node?"EPIC_TIMEOUT_HOLD":!epic_battery_ok_?"BATTERY_HOLD":
        !bounds?"BOUNDARY_HOLD":!frame?"PROFILE_FRAME_LOST_HOLD":!safe?"CONTROL_NOT_READY_HOLD":"RC7_OUTSIDE_MIDDLE_HOLD");
    }else if(event==argo_super::EpicSwitch::START){
      epic_epoch_ns_=now().nanoseconds();epic_guide_stamp_ns_=0;epic_pending_.clear();
      epic_wait_rx_=Steady::now();epic_reason_="WAIT_GUIDE";
      nav_msgs::msg::Path trigger;trigger.header.frame_id="world";trigger.header.stamp=now();
      geometry_msgs::msg::PoseStamped pose;pose.header=trigger.header;
      pose.pose.position.z=std::max(0.,state_.p.z());pose.pose.orientation.w=1.;trigger.poses.push_back(pose);epic_trigger_->publish(trigger);
    }
    if(epic_switch_.running){
      if(epic_fsm_=="LAND" || epic_fsm_=="CAUTION"){stopEpic("EPIC_UNSAFE_HOLD");}
      else if(epic_fsm_=="FINISH"){stopEpic("COMPLETE_HOLD");}
      else if(mission_kind_!="epic" || mission_complete_){
        if(!acceptEpicPending() && age(epic_wait_rx_)>5.)stopEpic("NO_GUIDE_HOLD");
      }else epic_wait_rx_=Steady::now();
    }
    if(age(epic_status_tx_)<.1)return;epic_status_tx_=Steady::now();
    if(!epic_switch_.running){std_msgs::msg::Bool manual;manual.data=true;epic_manual_->publish(manual);}
    const std::string blocked=!frame?"GROUND_ANCHOR_REQUIRED":!node?"EPIC_STATUS_STALE":!live?"RC7_STALE":!epic_airborne_?"ARMED_AIRBORNE_REQUIRED":
      !epic_battery_ok_?"BATTERY_NOT_READY":!epic_hover_ready_?"OFFBOARD_HOVER_NOT_READY":!bounds?"OUTSIDE_REAL_BOUNDS":!admission().empty()?admission():
      epic_fsm_!="WAIT_TRIGGER"?"WAIT_EPIC_READY":"";
    std::ostringstream j;j<<"{\"stamp_ns\":"<<now().nanoseconds()<<",\"enabled\":true,\"profile\":\""<<epic_profile_<<"\",\"active\":"<<(epic_switch_.running?"true":"false")
      <<",\"fsm\":\""<<epic_fsm_<<"\",\"fsm_fresh\":"<<(node?"true":"false")
      <<",\"state\":\""<<(epic_switch_.running?"EXPLORING":ready?"READY_RC7":"HOLD_OR_WAIT")
      <<"\",\"reason\":\""<<epic_reason_<<"\",\"blocked_by\":\""<<blocked<<"\",\"rc7_pwm\":"<<epic_rc7_
      <<",\"rc7_fresh\":"<<(live?"true":"false")<<",\"edge_armed\":"<<(epic_switch_.armed?"true":"false")
      <<",\"leg\":"<<(mission_kind_=="epic"?leg_+1:0)<<",\"legs\":"<<(mission_kind_=="epic"?route_.size():0)<<"}";
    std_msgs::msg::String msg;msg.data=j.str();epic_status_->publish(msg);
  }
  void setupEpic(){
    epic_enabled_=declare_parameter<bool>("epic_enabled",false);
    epic_profile_=declare_parameter<std::string>("epic_profile","gym");
    if(!epic_enabled_)return;
    epic_frame_required_=declare_parameter<bool>("epic_frame_required",false);
    if(epic_profile_!="gym" && epic_profile_!="frontier_whole" && epic_profile_!="frontier_single_floor")throw std::invalid_argument("Invalid EPIC profile");
    auto low=declare_parameter<std::vector<double>>("epic_bounds_lower",std::vector<double>{});
    auto high=declare_parameter<std::vector<double>>("epic_bounds_upper",std::vector<double>{});
    epic_bounds_.floor=declare_parameter<double>("epic_floor",-.4);epic_bounds_.ceiling=declare_parameter<double>("epic_ceiling",9.5);
    epic_flight_min_=declare_parameter<double>("epic_flight_min",.3);epic_flight_max_=declare_parameter<double>("epic_flight_max",9.5);
    if(low.empty() || low.size()!=high.size() || low.size()%3 || low.size()>12 ||
       !std::isfinite(epic_bounds_.floor) || !std::isfinite(epic_bounds_.ceiling) || epic_bounds_.floor>=epic_bounds_.ceiling ||
       epic_flight_min_<epic_bounds_.floor || epic_flight_max_>epic_bounds_.ceiling || !std::isfinite(epic_flight_min_) ||
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
    });
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
    epic_done_sub_=create_subscription<std_msgs::msg::Bool>("/mission/exploration_complete",rclcpp::QoS(1).reliable(),[this](std_msgs::msg::Bool::SharedPtr m){if(m->data && epic_switch_.running)stopEpic("COMPLETE_HOLD");});
  }

  static double age(Steady::time_point t){return std::chrono::duration<double>(Steady::now()-t).count();}
  bool feedbackFresh(){
    const double odom_age=now().seconds()-state_.timestamp_s,cloud_age=now().seconds()-cloud_stamp_;
    return odom_ok_ && odom_age>=-.02 && odom_age<.25 && cloud_age>=-.02 && cloud_age<.5
      && age(odom_rx_)<.25 && age(cloud_rx_)<.5 && age(bridge_rx_)<.6;
  }
  double heading() const {auto r=state_.q.toRotationMatrix();return std::atan2(r(1,0),r(0,0));}
  void freeze(){hold_.head<3>()=state_.p;hold_[3]=heading();trajectory_.clear();yaw_.clear();mission_=false;
    route_.clear();route_turning_=mission_complete_=pending_plan_=false;mission_kind_="none";intent_="none";}
  void fail(const std::string& reason){fault_=true;phase_=reason;RCLCPP_ERROR(get_logger(),"%s",reason.c_str());}
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
    requested_goal_=goal_=waypoint.position;
    geometry_msgs::msg::PoseStamped goal;goal.header.frame_id="world";goal.header.stamp=now();
    goal.pose.position.x=goal_.x();goal.pose.position.y=goal_.y();goal.pose.position.z=goal_.z();
    goal.pose.orientation.w=std::cos(waypoint.yaw/2);goal.pose.orientation.z=std::sin(waypoint.yaw/2);
    goal_request_stamp_ns_=rclcpp::Time(goal.header.stamp).nanoseconds();
    mission_=true;mission_complete_=false;mission_epoch_=rclcpp::Time(goal.header.stamp).seconds();goal_rx_=Steady::now();
    heartbeat_rx_=Steady::now();pending_plan_=true;
    if(!keep_reference){trajectory_.clear();yaw_.clear();}
    if((state_.p-goal_).norm()<goal_position_tolerance_ && state_.v.norm()<goal_speed_tolerance_){
      Eigen::MatrixXd c=Eigen::MatrixXd::Zero(3,8);c.col(7)=goal_;trajectory_.clear();trajectory_.emplace_back(.2,c);
      yaw_=fixedYaw(waypoint.yaw);start_=yaw_start_=now().seconds();pending_plan_=false;
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
      const bool running=mode && bridge_healthy_;
      if(bridge_active_ && !running){active_=false;if(odom_ok_ && !landing_)freeze();}
      bridge_active_=running;bridge_rx_=Steady::now();
    }catch(const YAML::Exception&){bridge_healthy_=false;}
  }
  void trajectory(const Poly& m){
    if(!active_ || !mission_ || mission_complete_ || local_takeoff_ || route_turning_ || landing_ || fault_ || m.header.frame_id!="world")return;
    const double generated=rclcpp::Time(m.header.stamp).seconds();
    if(generated<mission_epoch_ || generated>now().seconds()+.1 || now().seconds()-generated>.5)return;
    if(m.type & Poly::EMER_STOP){fail("SUPER_EMERGENCY");return;}
    if(m.type & Poly::HEART_BEAT)heartbeat_rx_=Steady::now();
    if(!(m.type & Poly::POSITION_TRAJ))return;
    try{
      auto p=argo_super::decode(m);Trajectory y;
      if(mission_kind_=="epic"){
        const double T=p.getTotalDuration();
        if(T>120.)throw std::invalid_argument("EPIC local trajectory too long");
        for(double t=0.;t<=T+.049;t+=.05)
          if(!insideEpic(p.getPos(std::min(t,T))))
            throw std::invalid_argument("EPIC local trajectory crosses exploration boundary");
      }
      if(m.type & Poly::YAW_TRAJ)y=argo_super::decode(m,true);
      if(!std::isfinite(m.start_wt_pos) || m.start_wt_pos<mission_epoch_-.1 || m.start_wt_pos>now().seconds()+1. || m.start_wt_pos+p.getTotalDuration()<now().seconds())throw std::invalid_argument("stale/future plan");
      const auto metadata=YAML::Load(m.debug_info);
      if (!metadata.IsMap() || metadata["goal_request_stamp_ns"].as<int64_t>()!=goal_request_stamp_ns_)
        throw std::invalid_argument("missing/mismatched planner goal request");
      const auto g=metadata["accepted_goal"];
      if (!g.IsSequence() || g.size()!=3) throw std::invalid_argument("invalid accepted planner goal");
      const Eigen::Vector3d accepted(g[0].as<double>(),g[1].as<double>(),g[2].as<double>());
      if (!accepted.allFinite() || (accepted-requested_goal_).norm()>goal_snap_max_distance_+1e-6)
        throw std::invalid_argument("accepted planner goal outside snap limit");
      if ((goal_-accepted).norm()>1e-6) settled_at_=0.;
      goal_=accepted;
      trajectory_=p;yaw_=mission_kind_=="patrol"?fixedYaw(route_[leg_].yaw):y;
      start_=m.start_wt_pos;yaw_start_=m.start_wt_yaw;pending_plan_=false;
    }catch(const std::exception&e){RCLCPP_WARN(get_logger(),"Rejected SUPER plan: %s",e.what());}
  }
  void tick(){
    epicTick();
    ommpc_core::ControlCommand out;bool valid=false;
    const bool fresh=feedbackFresh();
    if(landing_){
      phase_=!armed_ && landed_?"RTH_LANDED":"RTH_LANDING";
      intent_=armed_?"land":"none";
      if(bridge_active_ && fresh && !fault_){
        Eigen::Vector4d target;target.head<3>()=goal_;target[3]=home_yaw_;
        if(mpc_.setHoverReference(target) && mpc_.execMPC(state_,state_.timestamp_s,out)
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
        if(age(created_)>45.)fail("TAKEOFF_PREPARATION_TIMEOUT");
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
      bool reference=false;
      if(route_turning_){
        phase_="PATROL_TURN";
        if(now().seconds()-turn_started_>30.)fail("PATROL_YAW_TIMEOUT");
        const double error=argo_super::angle(route_[leg_].yaw-turn_yaw_);
        turn_yaw_+=std::clamp(error,-.4*period_,.4*period_);hold_[3]=turn_yaw_;
        reference=mpc_.setHoverReference(hold_);
        if(std::abs(argo_super::angle(heading()-route_[leg_].yaw))<.0873 && state_.v.norm()<goal_speed_tolerance_){
          if(settled_at_==0.)settled_at_=now().seconds();
          if(now().seconds()-settled_at_>1.){route_turning_=false;settled_at_=0.;publishGoal(route_[leg_]);}
        }else settled_at_=0.;
      }else if(mission_ && trajectory_.getPieceNum()>0){
        if(local_takeoff_ && ((state_.p.head<2>()-home_.head<2>()).norm()>.4 || state_.p.z()>goal_.z()+.3))fail("TAKEOFF_POSITION_ENVELOPE");
        if(local_takeoff_ && now().seconds()-start_>15.)fail("TAKEOFF_HEIGHT_TIMEOUT");
        const double elapsed=std::max(0.,t-start_);
        const bool stopped=argo_super::terminal_stop_reached(trajectory_,elapsed,state_.p,state_.v);
        if(!local_takeoff_ && pending_plan_ && age(goal_rx_)>1.)fail("REPLACEMENT_PLAN_TIMEOUT");
        if(!local_takeoff_ && !pending_plan_ && age(heartbeat_rx_)>1. && !stopped)fail("PLANNER_TIMEOUT");
        if(!fault_){
          reference=mpc_.setTrajectoryReference(trajectory_,elapsed,hold_[3],yaw_,state_,mission_kind_=="patrol"?0.:start_-yaw_start_);
        }
        phase_=fault_?phase_:local_takeoff_?"TAKEOFF_CLIMB":pending_plan_?"RTH_REPLANNING":"TRACKING";
        if(!fault_ && stopped){
          const bool at_goal=(state_.p-goal_).norm()<(local_takeoff_?.2:goal_position_tolerance_) &&
              state_.v.norm()<(local_takeoff_?.15:goal_speed_tolerance_) && !pending_plan_;
          const auto endpoint=trajectory_.getPos(trajectory_.getTotalDuration());
          phase_=at_goal?"ENDPOINT_HOLD":(endpoint-goal_).norm()<goal_position_tolerance_?"ENDPOINT_SETTLING":"BACKUP_STOP";
          if(at_goal){
            if(settled_at_==0.)settled_at_=now().seconds();
            if(now().seconds()-settled_at_>1. && !mission_complete_){
              if(local_takeoff_){local_takeoff_=mission_=false;mission_complete_=true;hold_.head<3>()=goal_;hold_[3]=home_yaw_;trajectory_.clear();phase_="HOVER";}
              else if(mission_kind_=="epic" && acceptEpicPending()){}
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
        reference=mpc_.setHoverReference(hold_);
        if(mission_ && age(goal_rx_)>5.)fail("NO_PLAN");
      }
      if(mission_complete_ && mission_kind_=="takeoff")phase_="HOVER";
      if(mission_ && !local_takeoff_ && !route_turning_ && age(goal_rx_)>120. && !mission_complete_)fail("MISSION_SEGMENT_TIMEOUT");
      if(!fault_ && reference && mpc_.execMPC(state_,t,out) && out.bodyrates.allFinite() && std::isfinite(out.collective_acceleration)){
        valid=true;failures_=0;
      }else if(!fault_ && ++failures_>=3)fail("SOLVER_FAILURE");
    }else if(!fault_)phase_="FEEDBACK_WAIT";
    std::ostringstream json;json<<std::setprecision(17)<<"{\"session\":\""<<session_<<"\",\"sequence\":"<<++sequence_
      <<",\"stamp_ns\":"<<now().nanoseconds()<<",\"valid\":"<<(valid&&!fault_?"true":"false")
      <<",\"rates_flu\":["<<out.bodyrates.x()<<","<<out.bodyrates.y()<<","<<out.bodyrates.z()<<"]"
      <<",\"acceleration\":"<<out.collective_acceleration<<",\"phase\":\""<<phase_<<"\",\"solver_ms\":"<<mpc_.getTimingMs()
      <<",\"position\":["<<state_.p.x()<<","<<state_.p.y()<<","<<state_.p.z()<<"]"
      <<",\"goal\":["<<goal_.x()<<","<<goal_.y()<<","<<goal_.z()<<"]}";
    auto text=json.str();text.pop_back();
    std::ostringstream meta;meta<<std::setprecision(17)<<",\"intent\":\""<<intent_<<"\",\"mission\":{\"kind\":\""<<mission_kind_<<"\",\"leg\":"<<(route_.empty()?0:leg_+1)
      <<",\"legs\":"<<route_.size()<<",\"complete\":"<<(mission_complete_?"true":"false")<<",\"home_valid\":"<<(home_valid_?"true":"false")
      <<",\"requested_goal\":["<<requested_goal_.x()<<","<<requested_goal_.y()<<","<<requested_goal_.z()<<"]"
      <<",\"home\":["<<home_.x()<<","<<home_.y()<<","<<home_.z()<<"],\"home_yaw\":"<<home_yaw_<<"}}";
    std_msgs::msg::String msg;msg.data=text+meta.str();status_->publish(msg);
    if(execute_)commands_->publish(msg);
  }
public:
  HardwareController():Node("zav_super_ommpc"){
    if(get_parameter("use_sim_time").as_bool())throw std::invalid_argument("Hardware requires use_sim_time=false");
    execute_=declare_parameter<bool>("execute",false);distance_=declare_parameter<double>("distance_m",10.);
    auto_takeoff_=declare_parameter<bool>("auto_takeoff",false);takeoff_height_=declare_parameter<double>("takeoff_height",.4);
    if(!std::isfinite(takeoff_height_) || takeoff_height_<.3 || takeoff_height_>2.)throw std::invalid_argument("Takeoff height must be 0.3..2m");
    if(!std::isfinite(distance_) || distance_<=0. || distance_>20.)throw std::invalid_argument("distance_m must be in (0,20]");
    const auto lock_path="/tmp/zav-super-controller-"+std::to_string(getuid())+".lock";
    lock_=open(lock_path.c_str(),O_CREAT|O_RDWR,0600);if(lock_<0 || flock(lock_,LOCK_EX|LOCK_NB))throw std::runtime_error("SUPER controller already running");
    session_="super-"+std::to_string(getpid())+"-"+std::to_string(now().nanoseconds());
    ommpc_core::ParameterSet p;
    auto par=[this](const std::string& n,double v){return declare_parameter<double>(n,v);};
    goal_position_tolerance_=par("goal_position_tolerance",.1);
    goal_speed_tolerance_=par("goal_speed_tolerance",.1);
    goal_snap_max_distance_=par("goal_snap_max_distance",.3);
    if (!std::isfinite(goal_position_tolerance_) || goal_position_tolerance_<=0. || goal_position_tolerance_>.3 ||
        !std::isfinite(goal_speed_tolerance_) || goal_speed_tolerance_<=0. ||
        !std::isfinite(goal_snap_max_distance_) || goal_snap_max_distance_<0.)
      throw std::invalid_argument("Invalid planner goal tolerances");
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
    mpc_.init(p);
    commands_=create_publisher<std_msgs::msg::String>("/zav/super/rates",rclcpp::QoS(1).reliable());
    status_=create_publisher<std_msgs::msg::String>("/zav/super/controller_status",10);
    goals_=create_publisher<geometry_msgs::msg::PoseStamped>("/zav/super/goal",rclcpp::QoS(1).reliable());
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/argo/super/odom",rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::SharedPtr m){odometry(*m);});
    cloud_sub_=create_subscription<sensor_msgs::msg::PointCloud2>("/argo/super/cloud_world",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::PointCloud2::SharedPtr m){
      const double stamp=rclcpp::Time(m->header.stamp).seconds(),dt=now().seconds()-stamp;
      if(m->header.frame_id=="world" && m->width*m->height>0 && dt>=-.02 && dt<.5){cloud_stamp_=stamp;cloud_rx_=Steady::now();}
    });
    bridge_sub_=create_subscription<std_msgs::msg::String>("/zav_real/status",rclcpp::QoS(1).reliable().transient_local(),[this](std_msgs::msg::String::SharedPtr m){bridge(*m);});
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
