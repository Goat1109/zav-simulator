#include <epic_planner/frontier_route.h>
/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2024-02-29 16:54:46
 * @LastEditTime: 2024-03-11 13:22:44
 * @Description:
 * @
 * @Copyright (c) 2024 by ning-zelin, All Rights Reserved.
 */

#include <epic_planner/expl_data.h>
#include <epic_planner/fast_exploration_fsm.h>
#include <epic_planner/fast_exploration_manager.h>
#include <plan_manage/planner_manager.h>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <traj_utils/planning_visualization.h>
using Eigen::Vector3d;
using Eigen::Vector4d;
typedef visualization_msgs::msg::Marker Marker;
typedef visualization_msgs::msg::MarkerArray MarkerArray;

void FastExplorationFSM::FSMCallback(const epic_ros::TimerEvent &e) {
  pubState();
  switch (state_) {
  case INIT: {
    if (!fd_->have_odom_) {
      ROS_WARN_THROTTLE(1.0, "no odom.");
      return;
    }
    transitState(WAIT_TRIGGER, "FSM");
    break;
  }

  case WAIT_TRIGGER: {
    ROS_WARN_THROTTLE(1.0, "wait for trigger.");
    break;
  }

  case WAIT_MAP:
  case FINISH: {
    // stopTraj();
    if (external_local_planner_) {
      // Exploration completion and ordered-polyline completion are distinct.
      // Retire the old guide only after fresh measurements confirm actual rest;
      // never cut a moving local trajectory into a zero-velocity hold.
      if(state_==FINISH && external_motion_started_ && !external_finish_hold_sent_) {
        const bool fresh=externalInputsReady(false);
        const bool resting=fresh && fd_->odom_vel_.allFinite() && fd_->odom_vel_.norm()<.15f;
        if(!resting)external_finish_rest_=InputFreshness{};
        else if(external_finish_rest_.observe(external_odom_stamp_,external_odom_stamp_,true)) {
          Vector3f previous=fd_->odom_pos_+Vector3f(.03f*std::cos(fd_->odom_yaw_),.03f*std::sin(fd_->odom_yaw_),0.f);
          const auto &guide=expl_manager_->ed_->path_next_goal_;
          if(!guide.empty())previous=guide.front();
          holdExternalGuide(previous);
          external_finish_hold_sent_=true;
          ROS_INFO("EPIC finished: retired the guide after measured stationary confirmation");
        }
      }
      break;
    }
    double collision_time = 0.0;
    bool safe = planner_manager_->checkTrajCollision(collision_time);
    if (!safe) {
      stopTraj();
    }
    ROS_WARN_THROTTLE(1.0, "Finished.");
    break;
  }

  case PLAN_TRAJ: {
    if (!fd_->trigger_)
      return;
    if (planner_manager_->topo_graph_->odom_node_->neighbors_.empty() &&
        !(external_local_planner_ && expl_manager_->corridor_pass_enabled_)) {
      stopTraj();
      break;
    }
    if (external_local_planner_) {
      int res = callExplorationPlanner();
      if (res != SUCCEED && external_hold_from_) holdExternalGuide(*external_hold_from_);
      external_hold_from_.reset();
      if (res == SUCCEED) {
        fd_->static_state_ = false;
        transitState(EXEC_TRAJ, "SUPER guide published");
      } else if (res == NO_FRONTIER || res == FRONTIER_PENDING) {
        transitState(res == FRONTIER_PENDING ? WAIT_MAP : FINISH, "external frontier status");
      } else {
        // SUPER owns collision checking, braking and its active trajectory.
        // A missing global guide must not invoke EPIC's unused MINCO state.
        ROS_WARN_THROTTLE(1.0, "External local planner: global guide unavailable");
      }
      break;
    }
    epic_ros::Time start = epic_ros::Time::now();
    // 要报min-step的case
    LocalTrajData *info = &planner_manager_->local_data_;
    double t_cur = (epic_ros::Time::now() - info->start_time_).toSec();
    double time_to_end = info->duration_ - t_cur;
    if (expl_manager_->ed_->global_tour_.size() == 2) {
      Eigen::Vector3f goal = expl_manager_->ed_->global_tour_[1];
      if ((goal - fd_->odom_pos_).norm() < 1e-1) {
        transitState(expl_manager_->frontier_manager_ptr_->hasInspectionFrontiers() ? WAIT_MAP : FINISH, "fsm");
        return;
      }
    }
    epic_ros::Time tplan = epic_ros::Time::now();
    exec_timer_.stop();
    int res = callExplorationPlanner();
    exec_timer_.start();
    ROS_INFO("\033[31m call planner \033[0m: %.3f",
             (epic_ros::Time::now() - tplan).toSec() * 1000.0);

    if (res == SUCCEED) {
      plan_failure_since_ = -1.0;
      poly_yaw_traj_pub_.publish(fd_->newest_yaw_traj_);
      poly_traj_pub_.publish(fd_->newest_traj_);
      fd_->static_state_ = false;
      if (fd_->use_bubble_a_star_) {
        transitState(EXEC_TRAJ,
                     "ParallelBubbleAstar plan success: new traj pub");
      } else {
        transitState(EXEC_TRAJ, "plan success: new traj pub");
      }
      fd_->use_bubble_a_star_ = false;
      fd_->half_resolution = false;

    } else if (res == NO_FRONTIER || res == FRONTIER_PENDING) {
      // if (planner_manager_->topo_graph_->global_view_points_.empty())
      transitState(res == FRONTIER_PENDING ? WAIT_MAP : FINISH, "PLAN_TRAJ: frontier status");
      const auto &active = planner_manager_->local_data_;
      fd_->static_state_ = active.minco_traj_.getPieceNum() == 0 ||
          ((epic_ros::Time::now() - active.start_time_).toSec() >= active.duration_ &&
           fd_->odom_vel_.norm() < 0.15);
    } else if (res == FAIL) {
      // A failed candidate does not invalidate a still-safe active trajectory.
      const double now = epic_ros::Time::now().toSec();
      if (plan_failure_since_ < 0.0) plan_failure_since_ = now;
      double horizon = 0.0;
      const bool safe = planner_manager_->checkTrajCollision(horizon);
      const auto &active = planner_manager_->local_data_;
      const double remaining = active.duration_ - (now - active.start_time_.toSec());
      // A fully collision-free trajectory with a stationary clear terminal
      // state continues as the trajectory server's existing endpoint hold.
      // Its end is not an impending collision. Persistent failure still lands
      // at the SAME 0.6s limit below; a real collision never uses this fallback.
      const bool safe_hold = safe && planner_manager_->hasSafeTerminalHold();
      const double safe_until = safe_hold ? std::numeric_limits<double>::infinity() :
          safe ? remaining : std::min(remaining, horizon);
      const double braking_acc = planner_manager_->gcopter_config_->maxAccMag > 0.0 ?
          planner_manager_->gcopter_config_->maxAccMag : 3.0;
      const double reaction_horizon = std::max(fp_->replan_time_ + 0.2,
          fd_->odom_vel_.norm() / braking_acc + 0.2);
      if (safe_until < reaction_horizon || now - plan_failure_since_ >= 0.6) {
        ROS_WARN("Landing: safe horizon %.3f, required %.3f, failure age %.3f",
                 safe_until, reaction_horizon, now - plan_failure_since_);
        stopTraj();
      } else {
        transitState(PLAN_TRAJ, safe_hold ? "candidate failed; continue certified stop/hold" :
                     "candidate failed; keep safe trajectory", true);
      }

    } else if (res == START_FAIL) {
      transitState(CAUTION, "PLAN_TRAJ: start failed", true);
    } else {
      cout << "330?" << endl;
    }
    break;
  }

  case EXEC_TRAJ: {
    if (external_local_planner_) break;
    // collision check
    double collision_time;
    bool safe = planner_manager_->checkTrajCollision(collision_time);
    if (!safe) {
      transitState(
          PLAN_TRAJ,
          "safetyCallback: not safe, time:" + to_string(collision_time), true);
      const double braking_acc = planner_manager_->gcopter_config_->maxAccMag > 0.0 ?
          planner_manager_->gcopter_config_->maxAccMag : 3.0;
      if (collision_time < std::max(fp_->replan_time_ + 0.2,
                                   fd_->odom_vel_.norm() / braking_acc + 0.2))
        stopTraj();
    } else if (!planner_manager_->checkTrajVelocity()) {
      transitState(PLAN_TRAJ, "velocity too fast", true);
    }

    break;
  }

  case CAUTION: {
    stopTraj();
    break;
  }
  case LAND: {
    // Non-blocking: continue heartbeat/map callbacks while PX4 owns landing.
    global_path_update_timer_.stop();
    epic_ros2::msg::TakeoffLand land_msg;
    land_msg.takeoff_land_cmd = land_msg.LAND;
    land_pub_.publish(land_msg);
    break;
  }
  }
}

void FastExplorationFSM::init(epic_ros::NodeHandle &nh,
                              FastExplorationManager::Ptr &explorer) {
  fp_.reset(new FSMParam);
  fd_.reset(new FSMData);

  nh.param("fsm/external_local_planner", external_local_planner_, false);
  if (external_local_planner_)
    external_path_pub_ = nh.advertise<nav_msgs::msg::Path>("/mission/exploration_path", 1, true);
  /*  Fsm param  */
  nh.param("fsm/thresh_replan", fp_->replan_thresh_, -1.0);
  nh.param("fsm/replan_time", fp_->replan_time_, -1.0);
  nh.param("bubble_astar/resolution_astar", fp_->bubble_a_star_resolution, 0.1);
  nh.param("fsm/emergency_replan_control_error",
           fp_->emergency_replan_control_error, 0.3);
  nh.param("fsm/replan_time_after_traj_start",
           fp_->replan_time_after_traj_start_, 0.5);
  nh.param("fsm/replan_time_before_traj_end", fp_->replan_time_before_traj_end_,
           0.5);
  /* Initialize main modules */
  // expl_manager_.reset(new FastExplorationManager);
  // expl_manager_->initialize(nh);
  expl_manager_ = explorer;
  planner_manager_ = expl_manager_->planner_manager_;

  state_ = EXPL_STATE::INIT;
  fd_->have_odom_ = false;
  fd_->state_str_ = {"INIT",      "WAIT_TRIGGER", "PLAN_TRAJ", "CAUTION",
                     "EXEC_TRAJ", "FINISH",       "LAND", "WAIT_MAP"};
  fd_->static_state_ = true;
  fd_->trigger_ = false;
  fd_->use_bubble_a_star_ = false;
  battary_sub_ =
      nh.subscribe("/mavros/battery", 10, &FastExplorationFSM::battaryCallback,
                   this, epic_ros::TransportHints().tcpNoDelay());

  /* Ros sub, pub and timer */
  exec_timer_ = nh.createTimer(epic_ros::Duration(0.01),
                               &FastExplorationFSM::FSMCallback, this);
  nh.param("global_planning/replan_period",global_replan_period_,.2);
  if(!std::isfinite(global_replan_period_) || global_replan_period_<.1 || global_replan_period_>5.)
    throw std::invalid_argument("Invalid global planning period");
  global_path_update_timer_ = nh.createTimer(
      epic_ros::Duration(global_replan_period_), &FastExplorationFSM::globalPathUpdateCallback, this);
  trigger_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1,
                              &FastExplorationFSM::triggerCallback, this);
  replan_pub_ = nh.advertise<std_msgs::msg::Empty>("/planning/replan", 10);

  heartbeat_pub_ = nh.advertise<std_msgs::msg::Empty>("/planning/heartbeat", 10);
  land_request_sub_ = nh.subscribe("/planning/land_requested", 10,
      &FastExplorationFSM::landRequestedCallback, this);
  land_pub_ =
      nh.advertise<epic_ros2::msg::TakeoffLand>("/px4ctrl/takeoff_land", 10);

  poly_traj_pub_ =
      nh.advertise<epic_ros2::msg::PolyTraj>("/planning/trajectory", 10);
  poly_yaw_traj_pub_ =
      nh.advertise<epic_ros2::msg::PolyTraj>("/planning/yaw_trajectory", 10);
  time_cost_pub_ = nh.advertise<std_msgs::msg::Float32>("/time_cost", 10);
  static_pub_ = nh.advertise<std_msgs::msg::Bool>("/planning/static", 10);
  state_pub_ = nh.advertise<visualization_msgs::msg::Marker>("/planning/state", 10);

  string odom_topic, cloud_topic;
  nh.getParam("odometry_topic", odom_topic);
  nh.getParam("cloud_topic", cloud_topic);
  cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::msg::PointCloud2>(
      nh.native().get(), cloud_topic, rmw_qos_profile_sensor_data));
  auto odom_qos = rmw_qos_profile_sensor_data;
  // SUPER body odometry arrives at200Hz, while its registered cloud arrives
  // after scan processing. Ten entries retain only50ms of odometry and lose
  // the cloud's measurement epoch before the cloud reaches this subscriber.
  // Keep one second in external mode; the existing scan-rate input is unchanged.
  if (external_local_planner_) odom_qos.depth = 200;
  odom_sub_.reset(
      new message_filters::Subscriber<nav_msgs::msg::Odometry>(nh.native().get(), odom_topic, odom_qos));
  SyncPolicyCloudOdom sync_policy(external_local_planner_ ? 200 : 10);
  if (external_local_planner_)
    sync_policy.setMaxIntervalDuration(rclcpp::Duration(0, 100000000));
  sync_cloud_odom_.reset(new message_filters::Synchronizer<SyncPolicyCloudOdom>(
      static_cast<const SyncPolicyCloudOdom &>(sync_policy), *cloud_sub_, *odom_sub_));
  sync_cloud_odom_->registerCallback(
      boost::bind(&FastExplorationFSM::CloudOdomCallback, this, _1, _2));
  if (external_local_planner_) {
    fd_->odom_pos_.setZero();
    fd_->odom_vel_.setZero();
    fd_->odom_yaw_ = 0.f;
    // A guide starts at the current body. Only the independent cloud-synchronised
    // subscription above needs odometry history.
    external_odom_sub_ = nh.native()->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::QoS(1).best_effort(),
        [this](const nav_msgs::msg::Odometry &msg) { odometryCallback(msg); });
    external_clock_sub_=nh.native()->create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock",rclcpp::QoS(1).best_effort(),[this](const rosgraph_msgs::msg::Clock &msg) {
          external_clock_stamp_=double(msg.clock.sec)+1.e-9*msg.clock.nanosec;
        });
    nh.param("exploration/state_max_age",external_input_freshness_.max_age,.10);
    nh.param("exploration/startup_freshness_confirmation",external_input_freshness_.confirmation,.5);
    nh.param("exploration/control_handoff_quiet_time",external_handoff_quiet_duration_,3.);
    if(!std::isfinite(external_handoff_quiet_duration_) || external_handoff_quiet_duration_<0.)
      throw std::invalid_argument("exploration.control_handoff_quiet_time must be nonnegative");
    std::string reference_imu;
    nh.param("exploration/state_reference_imu_topic",reference_imu,std::string{});
    if(!reference_imu.empty())external_reference_imu_sub_=nh.native()->create_subscription<sensor_msgs::msg::Imu>(
        reference_imu,rclcpp::QoS(1).best_effort(),[this](const sensor_msgs::msg::Imu &msg) {
          external_reference_imu_stamp_=double(msg.header.stamp.sec)+1.e-9*msg.header.stamp.nanosec;
        });
  }
}

void FastExplorationFSM::battaryCallback(
    const sensor_msgs::msg::BatteryState::ConstSharedPtr &msg) {
  // if(msg->voltage < 21.0){
  //   transitState(LAND, "battary low");
}

bool FastExplorationFSM::continueStair(const FrontierRoute &suffix, double remaining, double preview) {
  if (suffix.size()<2 || remaining>preview) return false;
  auto through=suffix;
  if (!vertical_remainder_.empty()) {
    if ((through.back()-vertical_remainder_.front()).norm()>.02f) return false;
    through.insert(through.end(),vertical_remainder_.begin()+1,vertical_remainder_.end());
  } else {
    // A narrow connector can start with a height change and finish as a
    // level corridor. Continue the actual observed tail before braking at
    // its last waypoint; the initial climb classification concerns only
    // the earlier portion of this guide.
    if(tracking_level_passage_ &&
       (!expl_manager_->endingInRise(through) || expl_manager_->followsMeasuredFloor(through)) &&
       (expl_manager_->extendLevelGuide(through,remaining,external_guide_clearance_) ||
        expl_manager_->extendThroughPassage(through,external_guide_clearance_))) {
      if(callExplorationPlanner(&through)!=SUCCEED) return false;
      ROS_INFO("EPIC continued the measured level tail of a height-changing guide");
      return true;
    }
    if (!expl_manager_->extendThroughLevelOutlet(through,external_guide_clearance_) &&
        !expl_manager_->continueBeforeStairTip(through,fd_->odom_vel_,external_guide_clearance_) &&
        !expl_manager_->extendObservedAscent(through,external_guide_clearance_,expl_manager_->endingInRise(through)))
      return false;
  }
  if (callExplorationPlanner(&through)!=SUCCEED) return false;
  ROS_INFO("EPIC continued observed stair before its endpoint, %.2fm remaining",remaining);
  return true;
}

void FastExplorationFSM::renewGuideFromScan(double stamp) {
  if (!external_local_planner_ || state_!=EXEC_TRAJ || !expl_manager_->corridor_pass_enabled_ ||
      (!tracking_level_passage_ && !tracking_vertical_step_) || external_guide_stamp_<0.) return;
  const double safe=planner_manager_->topo_graph_->parallel_bubble_astar_->safe_distance_;
  // Compare measurement epochs. A fresh point cloud commonly reaches the
  // planner after a guide was published but carries an earlier sensor stamp.
  if (stamp<=external_guide_map_stamp_ || stamp-last_guide_scan_<.20) return;
  refreshExternalOdom();
  double progress=external_route_progress_;
  auto suffix=remainingFrontierRoute(expl_manager_->ed_->path_next_goal_,fd_->odom_pos_,
                                     external_previous_position_,progress);
  if(suffix.size()<2) return;
  double remaining=0.;
  for(size_t i=1;i<suffix.size();++i) remaining+=(suffix[i]-suffix[i-1]).norm();
  const auto &limits=planner_manager_->gcopter_config_;
  const double preview=levelGuideRenewalDistance(fd_->odom_vel_.norm(),limits->maxAccMag,
      limits->maxJerkMag,global_replan_period_+.2,safe);
  if(tracking_level_passage_ && stamp-last_ascent_join_scan_>.45 &&
     (fd_->odom_pos_-last_ascent_join_position_).norm()>.65f*safe) {
    last_ascent_join_scan_=stamp;last_ascent_join_position_=fd_->odom_pos_;
    if(expl_manager_->interceptReturnAscent(suffix,fd_->odom_vel_,external_guide_clearance_) &&
       callExplorationPlanner(&suffix)==SUCCEED) return;
  }
  if(!expl_manager_->followingStoreyOutlet() && (tracking_vertical_step_ || expl_manager_->endingInRise(suffix)) &&
     stamp-last_outlet_scan_>.40 && (fd_->odom_pos_-last_outlet_observer_).norm()>.40f*safe) {
    last_outlet_scan_=stamp;last_outlet_observer_=fd_->odom_pos_;
    // Look from a near future point on the already certified stair path
    // while the vehicle still has speed. That measured approach can lead
    // into the new storey's door without first aiming back from the body.
    if(fd_->odom_vel_.norm()>.6f && remaining>1.4*safe) {
      auto moving=suffix;
      splitFrontierRoute(moving,std::min(.78*remaining,std::clamp(.75*fd_->odom_vel_.norm(),.8*safe,2.8*safe)));
      if(expl_manager_->inspectStoreyOutlet(moving,external_guide_clearance_) && callExplorationPlanner(&moving)==SUCCEED)
        return;
    }
    FrontierRoute outlet{fd_->odom_pos_};
    if(expl_manager_->inspectStoreyOutlet(outlet,safe) && callExplorationPlanner(&outlet)==SUCCEED) return;
  }
  auto improve_preview=[&]() {
    if(expl_manager_->intermediate_observation_ && !expl_manager_->followingEndTurn() &&
       stamp-last_corner_scan_>.20 && (fd_->odom_pos_-last_corner_position_).norm()>.25f*safe &&
       remaining>3.5*safe && vertical_remainder_.empty() &&
       (suffix.back()-expl_manager_->ed_->next_goal_node_->center_).norm()<.02f) {
      last_corner_position_=fd_->odom_pos_;
      last_corner_scan_=stamp;
      const float margin=external_guide_clearance_<0. ? safe : external_guide_clearance_;
      const double previous=expl_manager_->selected_pass_clearance_;
      double certified=previous;
      auto improved=expl_manager_->previewMeasuredPass(suffix,fd_->odom_vel_,
          tracking_level_passage_ ? float(preview+2.*safe) : 0.f,margin,certified);
      if(!improved.empty()) {
        expl_manager_->selected_pass_clearance_=certified;
        if(callExplorationPlanner(&improved)==SUCCEED) {
          ROS_INFO("EPIC rejoined a longer observed sightline at %.2fm clearance",certified);
          return true;
        }
        expl_manager_->selected_pass_clearance_=previous;
      }
    }
    return false;
  };
  // A shortcut ending at the same imminent intermediate observation
  // cannot supply braking distance beyond it. Attempt onward observation
  // first and shape the existing guide if the map has no further segment.
  const double stair_preview=tracking_vertical_step_ ?
      std::max(preview,2.*fd_->odom_vel_.norm()+3.*safe) : preview;
  if(remaining>stair_preview) {improve_preview();return;}
  if(remaining>3.*safe+.6*fd_->odom_vel_.norm() && stamp-last_guide_scan_<global_replan_period_ &&
     (fd_->odom_pos_-last_guide_scan_position_).norm()<.5*safe) {improve_preview();return;}
  last_guide_scan_=stamp;
  last_guide_scan_position_=fd_->odom_pos_;
  if(expl_manager_->blockedAhead(suffix,external_guide_clearance_)) {
    if(tracking_vertical_step_ && !expl_manager_->followingEndTurn() &&
       (expl_manager_->continueBeforeStairTip(suffix,fd_->odom_vel_,external_guide_clearance_) ||
        expl_manager_->repairStairGuide(suffix,external_guide_clearance_))) {
      callExplorationPlanner(&suffix);
      return;
    }
    if(tracking_level_passage_ && !tracking_vertical_step_ &&
        expl_manager_->repairLevelGuide(suffix,remaining,external_guide_clearance_,fd_->odom_vel_))
      callExplorationPlanner(&suffix);
    return;
  }
  if(expl_manager_->followingEndTurn()) return;
  if(tracking_vertical_step_) {
    if(continueStair(suffix,remaining,stair_preview)) return;
  } else {
    FrontierRoute forward_branch;
    if(expl_manager_->continueAcrossJunction(suffix,fd_->odom_vel_,forward_branch) &&
        callExplorationPlanner(&forward_branch)==SUCCEED) return;
    if((expl_manager_->endingInRise(suffix) || expl_manager_->onMeasuredStairLanding(suffix)) &&
       (expl_manager_->extendThroughLevelOutlet(suffix,external_guide_clearance_) ||
        expl_manager_->extendObservedAscent(suffix,external_guide_clearance_,true)) &&
       callExplorationPlanner(&suffix)==SUCCEED) return;
    const Vector3f old_tip=suffix.back();
    if(expl_manager_->planObservedEndTurn(suffix,fd_->odom_vel_,external_guide_clearance_)) {
      callExplorationPlanner(&suffix);
      return;
    }
    if((expl_manager_->extendLevelGuide(suffix,remaining,external_guide_clearance_) ||
        expl_manager_->extendThroughPassage(suffix,external_guide_clearance_)) && callExplorationPlanner(&suffix)==SUCCEED) {
      ROS_INFO("EPIC extended level guide by %.2fm with %.2fm remaining after scan",
               double((suffix.back()-old_tip).norm()),remaining);
      return;
    }
  }
  improve_preview();
}

void FastExplorationFSM::updateTopoAndGlobalPath() {
  if(external_local_planner_ && !externalInputsReady(!external_motion_started_)) return;
  if (!(state_ == WAIT_TRIGGER || state_ == PLAN_TRAJ || state_ == EXEC_TRAJ ||
        state_ == FINISH || state_ == WAIT_MAP)) {
    global_path_update_timer_.stop();
    // expl_manager_->frontier_manager_ptr_->viz_pocc();
    expl_manager_->frontier_manager_ptr_->visfrtcluster();
    global_path_update_timer_.start();
    return;
  }
  static int cnt = 0;
  cnt++;

  global_path_update_timer_.stop();
  epic_ros::Time t2 = epic_ros::Time::now();
  planner_manager_->topo_graph_->getRegionsToUpdate();
  // cout << "getRegionsToUpdate time cost:" << (epic_ros::Time::now() - t2).toSec()
  // * 1000 << "ms" << endl;
  planner_manager_->topo_graph_->updateSkeleton();

  epic_ros::Time t3 = epic_ros::Time::now();
  if(external_local_planner_) refreshExternalOdom();
  planner_manager_->topo_graph_->updateOdomNode(fd_->odom_pos_, fd_->odom_yaw_);
  planner_manager_->topo_graph_->updateHistoricalOdoms();

  // In external mode the trigger is sent after takeoff. Map and topology
  // acquisition can proceed on the pad; a ground-level exploration search
  // would compete with LIO startup and seed an intent before the mission.
  if (external_local_planner_ && state_ == WAIT_TRIGGER) {
    global_path_update_timer_.start();
    return;
  }
  if (planner_manager_->topo_graph_->odom_node_->neighbors_.empty() &&
      !(external_local_planner_ && expl_manager_->corridor_pass_enabled_)) {
    double time;
    if (external_local_planner_) {
      global_path_update_timer_.start();
      return;
    }
    if (planner_manager_->local_data_.traj_id_ > 1) {
      bool safe = planner_manager_->checkTrajCollision(time);
      if (!safe) {
        transitState(CAUTION, "odom_node no nbrs");
      } else {
        global_path_update_timer_.start();

        return;
      }
    } else {
      transitState(CAUTION, "odom_node no nbrs");
    }
    global_path_update_timer_.start();
    return;
  }
  if(external_local_planner_) refreshExternalOdom();
  const bool corridor = expl_manager_->corridor_pass_enabled_;
  const bool monitoring = state_ == EXEC_TRAJ || (corridor && (state_ == WAIT_MAP || state_ == PLAN_TRAJ));
  if (external_local_planner_ && expl_manager_->directional_frontiers_ && monitoring && external_guide_stamp_ >= 0.) {
    const Vector3f previous = external_previous_position_;
    auto suffix = remainingFrontierRoute(expl_manager_->ed_->path_next_goal_, fd_->odom_pos_, previous,
                                         external_route_progress_);
    external_previous_position_ = fd_->odom_pos_;
    double remaining = 0.;
    for (size_t i = 1; i < suffix.size(); ++i) remaining += (suffix[i] - suffix[i-1]).norm();
    const double now = epic_ros::Time::now().toSec();
    const bool moved = corridor ? (fd_->odom_pos_ - external_watch_position_).norm() > .3 :
        external_route_progress_ > external_progress_watch_ + .3;
    if (external_progress_watch_stamp_ < 0. || moved) {
      external_watch_position_ = fd_->odom_pos_;
      external_progress_watch_ = external_route_progress_;
      external_progress_watch_stamp_ = now;
    }
    const double still = now - external_progress_watch_stamp_;
    auto blocked = corridor ? expl_manager_->blockedAhead(suffix, external_guide_clearance_) : std::nullopt;
    const bool lost = corridor && suffix.empty();
    if(blocked && state_==EXEC_TRAJ && tracking_vertical_step_ && still<3. && !suffix.empty()) {
      auto same_goal=suffix;
      if(!vertical_remainder_.empty() && (vertical_remainder_.front()-same_goal.back()).norm()<.02f)
        same_goal.insert(same_goal.end(),vertical_remainder_.begin()+1,vertical_remainder_.end());
      if((expl_manager_->continueBeforeStairTip(same_goal,fd_->odom_vel_,external_guide_clearance_) ||
          expl_manager_->repairStairGuide(same_goal,external_guide_clearance_)) &&
         callExplorationPlanner(&same_goal)==SUCCEED) {
        global_path_update_timer_.start();
        return;
      }
    }
    if (blocked && state_ == EXEC_TRAJ && tracking_level_passage_ && !tracking_vertical_step_ &&
        still < 3. && expl_manager_->repairLevelGuide(suffix, remaining, external_guide_clearance_,fd_->odom_vel_) &&
        callExplorationPlanner(&suffix) == SUCCEED) {
      global_path_update_timer_.start();
      return;
    }
    if (blocked || lost || (remaining > .5 && still >= 3.)) {
      ROS_WARN("External guide invalid in %s: %s, still %.2fs, remaining %.2fm",
          fd_->state_str_[int(state_)].c_str(), blocked ? "new scan obstructs guide" :
          lost ? "body left guide" : "progress stalled", still, remaining);
      if (corridor) {
        if (!blocked && !lost) {
          const auto ahead = FastPlannerManager::boundedGuidePrefix(suffix, 1.5);
          if (ahead.size() > 1) blocked = ahead.back().cast<float>();
        }
        if (blocked) expl_manager_->avoidBlockedPoint(*blocked, still>=3.);
        external_hold_from_ = previous;
      } else {
        expl_manager_->frontier_manager_ptr_->deferViewpoint(external_route_goal_, 15.);
      }
      external_guide_stamp_ = -1.;
      external_progress_watch_stamp_ = -1.;
      vertical_remainder_.clear();
      expl_manager_->external_reselect_from_current_ = true;
    } else if (expl_manager_->followingEndTurn() && remaining>.55 && still<3.) {
      global_path_update_timer_.start();
      return;
    } else if (tracking_vertical_step_ && remaining > .55 && still < 3.) {
      // A short climbing guide can span an entire doorway. Look sideways
      // as the lidar gains altitude, before committing to the next flight.
      if (!tracking_level_passage_ && (!std::isfinite(last_landing_probe_z_) ||
          std::abs(fd_->odom_pos_.z()-last_landing_probe_z_)>.45)) {
        last_landing_probe_z_ = fd_->odom_pos_.z();
        if (expl_manager_->selectLevelPassage(fd_->odom_pos_, true)) {
          vertical_remainder_.clear();
          transitState(PLAN_TRAJ,"survey observed landing opening");
          global_path_update_timer_.start();
          return;
        }
      }
      if (state_ == EXEC_TRAJ) {
        const double safe = planner_manager_->topo_graph_->parallel_bubble_astar_->safe_distance_;
        const auto &limits = planner_manager_->gcopter_config_;
        const double preview = levelGuideRenewalDistance(fd_->odom_vel_.norm(), limits->maxAccMag,
            limits->maxJerkMag, global_replan_period_ + .2, safe);
        continueStair(suffix,remaining,preview);
      }
      // Keep a progressing observed landing step while its freshly checked
      // approach is clear. Changing a remote viewpoint mid-turn changes yaw.
      global_path_update_timer_.start();
      return;
    } else if (tracking_vertical_step_ && !vertical_remainder_.empty()) {
      if (!tracking_level_passage_ && expl_manager_->selectLevelPassage(fd_->odom_pos_, true)) {
        vertical_remainder_.clear();
        transitState(PLAN_TRAJ,"explore connected landing passage");
        global_path_update_timer_.start();
        return;
      }
      // Finishing a short segment is not a new frontier decision. Carry the
      // witnessed suffix through the turn if the latest scan still permits it.
      if (callExplorationPlanner(&vertical_remainder_) == SUCCEED) {
        transitState(EXEC_TRAJ, "continue measured vertical route");
        global_path_update_timer_.start();
        return;
      }
      vertical_remainder_.clear();
    } else if (tracking_level_passage_ && !tracking_vertical_step_ && still < 3.) {
      const double safe = planner_manager_->topo_graph_->parallel_bubble_astar_->safe_distance_;
      const auto &limits = planner_manager_->gcopter_config_;
      const double preview = levelGuideRenewalDistance(fd_->odom_vel_.head<2>().norm(),
          limits->maxAccMag, limits->maxJerkMag, global_replan_period_ + .2, safe);
      const Vector3f old_tip = suffix.back();
      if (state_==EXEC_TRAJ && remaining<=preview &&
          expl_manager_->planObservedEndTurn(suffix,fd_->odom_vel_,external_guide_clearance_) &&
          callExplorationPlanner(&suffix)==SUCCEED) {
        global_path_update_timer_.start();
        return;
      }
      FrontierRoute forward_branch;
      if (state_ == EXEC_TRAJ && expl_manager_->continueAcrossJunction(suffix,fd_->odom_vel_,forward_branch) &&
          callExplorationPlanner(&forward_branch)==SUCCEED) {
        global_path_update_timer_.start();
        return;
      }
      if (state_ == EXEC_TRAJ && remaining <= preview &&
          (expl_manager_->extendLevelGuide(suffix, remaining, external_guide_clearance_) ||
           expl_manager_->extendThroughPassage(suffix,external_guide_clearance_)) &&
          callExplorationPlanner(&suffix) == SUCCEED) {
        ROS_INFO("EPIC extended level guide by %.2fm with %.2fm remaining",
                 double((suffix.back() - old_tip).norm()), remaining);
        global_path_update_timer_.start();
        return;
      }
      // While a clear pass is still progressing, an unsuccessful preview
      // leaves its active route and the normal end-of-passage decision intact.
      if (remaining > 3. * safe) {
        global_path_update_timer_.start();
        return;
      }
    } else if (state_ == EXEC_TRAJ && remaining > 12. && still < 3. &&
               (!corridor || now - external_guide_stamp_ < 3.)) {
      auto prefix = FastPlannerManager::boundedGuidePrefix(suffix, 12.);
      bool clear = prefix.size() > 1;
      for (size_t i = 1; clear && i < prefix.size(); ++i) {
        FrontierRoute segment{prefix[i-1].cast<float>(), prefix[i].cast<float>()};
        clear = planner_manager_->topo_graph_->parallel_bubble_astar_->collisionCheck_shortenPath(segment);
      }
      if (clear) { global_path_update_timer_.start(); return; }
    }
  }
  if (!external_local_planner_ && planner_manager_->local_data_.traj_id_ > 1) {

    double curr_time =
        (epic_ros::Time::now() - planner_manager_->local_data_.start_time_).toSec();
    double time;
    bool safe = planner_manager_->checkTrajCollision(time);
    double total_time = planner_manager_->local_data_.duration_;
    double time2end = total_time - curr_time;

    if (safe && curr_time < fp_->replan_time_after_traj_start_ &&
        time2end > fp_->replan_time_before_traj_end_) {
      global_path_update_timer_.start();
      return;
    }
  }
  cout << endl << endl;
  cout << "\033[1;33m------------- <" << cnt
       << "> Plan Global Path start---------------" << "\033[0m" << endl;
  epic_ros::Time t4 = epic_ros::Time::now();
  // cout << "updateSkeleton time cost:" << (t3 - t2).toSec() * 1000 << "ms" <<
  // endl; if( (t3 - t1).toSec() * 1000 > 100){
  //   ROS_ERROR("time too long");
  //   exit(0);
  ROS_INFO("update topo skeleton cost: %fms, update odom vertex cost:%fms ",
           (t3 - t2).toSec() * 1000, (t4 - t3).toSec() * 1000);
  if(external_local_planner_) refreshExternalOdom();
  Eigen::Vector3d vel = fd_->odom_vel_.cast<double>();
  Eigen::Vector3d odom = fd_->odom_pos_.cast<double>();
  int res = expl_manager_->planGlobalPath(odom, vel);
  if (res != SUCCEED && external_hold_from_) {
    holdExternalGuide(*external_hold_from_);
    external_hold_from_.reset();
  }
  epic_ros::Time t5 = epic_ros::Time::now();

  cout << "\033[1;33m-------------Plan Global Path end-----------------"
       << "\033[0m" << endl
       << endl;

  planner_manager_->graph_visualizer_->vizBox(planner_manager_->topo_graph_);
  if(expl_manager_->ep_->view_graph_)
    planner_manager_->graph_visualizer_->vizGraph(planner_manager_->topo_graph_);
  std_msgs::msg::Float32 time_cost;
  double time_cost_now = (t5 - t2).toSec() * 1000;
  time_cost.data = time_cost_now;
  time_cost_pub_.publish(time_cost);

  cout << "total time cost: " << time_cost_now << "ms" << endl;
  if ((res == NO_FRONTIER || res == FRONTIER_PENDING) && state_ != WAIT_TRIGGER) {
    transitState(res == FRONTIER_PENDING ? WAIT_MAP : FINISH, "planGlobalPath: frontier status");
  } else if (res == SUCCEED && state_ != WAIT_TRIGGER) {
    transitState(PLAN_TRAJ, "planGlobalPath: succeed");
  }

  expl_manager_->frontier_manager_ptr_->viz_pocc();
  expl_manager_->frontier_manager_ptr_->visfrtcluster();
  static epic_ros::Time t_p = epic_ros::Time::now();
  if ((epic_ros::Time::now() - t_p).toSec() > 5.0) {
    expl_manager_->frontier_manager_ptr_->printMemoryCost();
    t_p = epic_ros::Time::now();
  }
  global_path_update_timer_.start();
  cout << "viz&&print cost:" << (epic_ros::Time::now() - t5).toSec() * 1000 << "ms"
       << endl;
}

void FastExplorationFSM::globalPathUpdateCallback(const epic_ros::TimerEvent &e) {
  if (state_ == LAND) return;
  updateTopoAndGlobalPath();
}
