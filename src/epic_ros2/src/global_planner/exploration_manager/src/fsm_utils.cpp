#include <epic_planner/frontier_route.h>
#include <epic_planner/expl_data.h>
#include <epic_planner/fast_exploration_fsm.h>

void FastExplorationFSM::pubState() {

  std_msgs::msg::Empty heartbeat_msg;
  heartbeat_pub_.publish(heartbeat_msg);
  std_msgs::msg::Bool msg;
  msg.data = fd_->static_state_;
  static_pub_.publish(msg);
  Marker state_marker;
  state_marker.type = Marker::TEXT_VIEW_FACING;
  state_marker.pose.position.x = fd_->odom_pos_.x();
  state_marker.pose.position.y = fd_->odom_pos_.y();
  state_marker.pose.position.z = fd_->odom_pos_.z();
  state_marker.pose.orientation.w = 1.0;
  state_marker.scale.x = state_marker.scale.y = state_marker.scale.z = 0.5;
  state_marker.action = Marker::ADD;
  state_marker.color.r = 1.0;
  state_marker.color.a = 1.0;
  state_marker.text = fd_->state_str_[int(state_)];
  state_marker.header.frame_id = "world";
  state_marker.header.stamp = epic_ros::Time::now();

  state_pub_.publish(state_marker);
}

int FastExplorationFSM::callExplorationPlanner(const FrontierRoute *continuation) {
  if(external_local_planner_ && !externalInputsReady(!external_motion_started_)) return FRONTIER_PENDING;
  // if (planner_manager_->lidar_map_interface_->getDisToOcc(fd_->odom_pos_) < planner_manager_->gcopter_config_->dilateRadiusHard)
  //   return START_FAIL;
  if (planner_manager_->topo_graph_->odom_node_->neighbors_.empty() &&
      !(external_local_planner_ && expl_manager_->corridor_pass_enabled_)) return START_FAIL;
  if (expl_manager_->ed_->global_tour_.size() < 2)
    return expl_manager_->frontierStatus();

  // debug
  if (planner_manager_->lidar_map_interface_->getDisToOcc(expl_manager_->ed_->next_goal_node_->center_) <
      planner_manager_->topo_graph_->bubble_min_radius_) { // TODO:
    cout << "410:  next goal in occ, update it" << endl;
    if (!continuation) updateTopoAndGlobalPath();
    return FAIL;
  }
  vector<Eigen::Vector3f> path_next_goal;

  if ((expl_manager_->ed_->next_goal_node_->center_-expl_manager_->ed_->global_tour_[1]).norm()>.01f) {
    ROS_ERROR("Selected goal node disagrees with global tour; reconnect before planning");
    expl_manager_->updateGoalNode();
    return FAIL;
  }

  const bool verified_pass = external_local_planner_ && (continuation ?
      expl_manager_->continuePassGuide(fd_->odom_pos_, *continuation, path_next_goal) :
      expl_manager_->selectedPassGuide(fd_->odom_pos_, path_next_goal));
  if (continuation && !verified_pass) return FAIL;
  int res = verified_pass ? int(ParallelBubbleAstar::REACH_END) :
      planner_manager_->fast_searcher_->search(planner_manager_->topo_graph_->odom_node_,
          fd_->odom_vel_, expl_manager_->ed_->next_goal_node_, 0.2, path_next_goal);
  if (res == ParallelBubbleAstar::NO_PATH) {
    ROS_ERROR("ExplorationPlanner: No path to goal");
    return FAIL;

  } else if (res == ParallelBubbleAstar::START_FAIL) {
    ROS_ERROR("ExplorationPlanner: Start point in occ");
    return START_FAIL;
  } else if (res == ParallelBubbleAstar::END_FAIL) {
    ROS_ERROR("ExplorationPlanner: End point in occ");
    return FAIL;
  } else if (res == ParallelBubbleAstar::TIME_OUT) {
    ROS_ERROR("ExplorationPlanner: Time out");
    return FAIL;
  }

  auto info = &planner_manager_->local_data_;

  if (path_next_goal.size()<2 ||
      (path_next_goal.back()-expl_manager_->ed_->next_goal_node_->center_).norm()>.2f) {
    ROS_ERROR("Reject local guide that does not reach selected global goal");
    return FAIL;
  }

  if (verified_pass && !continuation && !external_hold_from_ &&
      expl_manager_->intermediate_observation_ && !expl_manager_->followingEndTurn()) {
    const auto astar=planner_manager_->topo_graph_->parallel_bubble_astar_;
    const auto &limits=planner_manager_->gcopter_config_;
    const double safe=astar->safe_distance_,margin=expl_manager_->selected_pass_clearance_;
    double length=0.;
    for(size_t i=1;i<path_next_goal.size();++i) length+=(path_next_goal[i]-path_next_goal[i-1]).norm();
    const double preview=levelGuideRenewalDistance(fd_->odom_vel_.norm(),limits->maxAccMag,
        limits->maxJerkMag,global_replan_period_+.2,safe);
    if(length<=std::max(preview+safe,5.*safe)) {
      const auto previous=path_next_goal.back();
      if(expl_manager_->extendNewPass(path_next_goal,length,margin)) ROS_INFO("EPIC connected %.2fm of observed onward travel before the first handoff",
                           double((path_next_goal.back()-previous).norm()));
    }
  }
  double view_yaw=planner_manager_->local_data_.end_yaw_;
  epic_ros::NodeHandle observation_nh;bool forward_view_yaw=false;
  observation_nh.param("exploration/forward_viewpoint_yaw",forward_view_yaw,false);
  if (external_local_planner_) {
    if(forward_view_yaw && !std::isfinite(view_yaw))return FAIL;
    // SUPER follows the published polyline in order. Its first leg must not
    // lead back to the stale topology departure and create an artificial
    // reversal. Reuse the native collision-checked boundary reconnect before
    // sweep extension; preserve the selected global goal and safe suffix.
    if (!verified_pass && !expl_manager_->reanchorLocalPath(path_next_goal, fd_->odom_pos_, fd_->odom_vel_))
      return FAIL;
  } else if (!fd_->static_state_) {
    double plan_finish_time_exp = (epic_ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;
    if (plan_finish_time_exp > info->duration_) {
      plan_finish_time_exp = info->duration_;
    }
    Eigen::Vector3d start_exp = info->minco_traj_.getPos(plan_finish_time_exp);
    if(!expl_manager_->reanchorLocalPath(path_next_goal,start_exp.cast<float>(),
        info->minco_traj_.getVel(plan_finish_time_exp).cast<float>()))return FAIL;
  }
  if (!continuation && !expl_manager_->acceptPassDeparture(path_next_goal, verified_pass)) return FRONTIER_PENDING;
  bool vertical_step = false;
  FrontierRoute next_vertical_remainder;
  if(external_local_planner_) {
    const auto astar = planner_manager_->topo_graph_->parallel_bubble_astar_;
    if (expl_manager_->corridor_pass_enabled_) {
      const auto near = FastPlannerManager::boundedGuidePrefix(path_next_goal, 6. * astar->safe_distance_);
      double low = fd_->odom_pos_.z(), high = low, room = std::numeric_limits<double>::infinity();
      for (const auto &p : near) {
        low = std::min(low, p.z());
        high = std::max(high, p.z());
        room = std::min(room, astar->lidar_map_interface_->getDisToOcc(p));
      }
      vertical_step = (high-low>.65*astar->safe_distance_ && room<2.*astar->safe_distance_ &&
                       !(verified_pass && expl_manager_->followsMeasuredFloor(path_next_goal))) ||
          (verified_pass && !expl_manager_->level_passage_ && path_next_goal.back().z()>fd_->odom_pos_.z()+astar->safe_distance_ &&
           expl_manager_->endingInRise(path_next_goal));
    }
    if (vertical_step) {
      double length = 0.;
      for (size_t i = 1; i < path_next_goal.size(); ++i) length += (path_next_goal[i] - path_next_goal[i-1]).norm();
      // Carry the measured bend into the guide so the follower can bound
      // its lookahead against the actual three-dimensional stair polyline.
      // A fully witnessed native guide already contains the safe bends. SUPER's
      // chord check determines its instantaneous spatial preview. Truncating a
      // verified flight midway through the same flight of stairs imposes a
      // false terminal condition and resets the rolling preview.
      double limit = verified_pass ? length : 6. * astar->safe_distance_;
      const double max_yaw_step = .5 * planner_manager_->gcopter_config_->yaw_max_vel;
      const double turn = std::remainder(view_yaw - fd_->odom_yaw_, 2. * M_PI);
      // Align while crossing an intermediate landing. Reserve the inspection
      // yaw for the actual visible pose, where the vehicle can settle.
      if (!expl_manager_->intermediate_observation_ && std::abs(turn) > max_yaw_step && length > .9)
        limit = std::min(limit, length - .45);
      const bool transit = length > limit + .01 || expl_manager_->intermediate_observation_;
      if (length > limit + .01) {
        next_vertical_remainder = splitFrontierRoute(path_next_goal, limit);
      }
      if (transit && path_next_goal.size() > 1) {
        const Vector3f ahead = path_next_goal.back() - path_next_goal.front();
        const double desired = ahead.head<2>().norm() > .2 ? std::atan2(ahead.y(), ahead.x()) : fd_->odom_yaw_;
        view_yaw = fd_->odom_yaw_ + std::clamp(std::remainder(desired - fd_->odom_yaw_, 2. * M_PI),
                                               -max_yaw_step, max_yaw_step);
      }
      if (path_next_goal.size() > 1) {
        const auto &end = path_next_goal.back();
        ROS_INFO("EPIC narrow vertical step ends at (%.2f %.2f %.2f), yaw=%.2f, intermediate=%d",
            end.x(), end.y(), end.z(), view_yaw, int(transit));
      }
    } else {
      expl_manager_->extendTourPath(path_next_goal);
    }
    if(expl_manager_->directional_frontiers_ && !expl_manager_->followingEndTurn())path_next_goal=simplifyFrontierRoute(path_next_goal,1.5f,
        [&](const Eigen::Vector3f&a,const Eigen::Vector3f&b) {
          FrontierRoute segment{a,b};
          return planner_manager_->topo_graph_->parallel_bubble_astar_->collisionCheck_shortenPath(segment,
                     verified_pass ? expl_manager_->selected_pass_clearance_ : -1.) &&
              (!expl_manager_->corridor_pass_enabled_ || expl_manager_->observedPass(segment));
        });
    if (verified_pass && (expl_manager_->level_passage_ || vertical_step)) {
      // Map search runs on this executor too. While it worked, the aircraft
      // could have flown beyond the departure pose of this replacement guide.
      refreshExternalOdom();
      const Vector3f departure = path_next_goal.front();
      const float moved = (fd_->odom_pos_ - departure).norm();
      if (moved > .01f) {
        double progress = 0.;
        auto suffix = remainingFrontierRoute(path_next_goal, fd_->odom_pos_, departure, progress);
        const auto *cut = next_vertical_remainder.empty() ? nullptr : &next_vertical_remainder.front();
        if (!expl_manager_->continuePassGuide(fd_->odom_pos_, suffix, path_next_goal, cut)) return FAIL;
        if (moved > .5f)
          ROS_INFO("Reconnected EPIC level guide after %.2fm of measured body motion", moved);
      }
    }
    if (verified_pass && !expl_manager_->followingEndTurn() && (vertical_step || expl_manager_->level_passage_)) {
      const auto clear = [&](const Vector3f &from, const Vector3f &to) {
        FrontierRoute edge{from, to};
        return expl_manager_->observedPass(edge) &&
            astar->collisionCheck_shortenPath(edge, expl_manager_->selected_pass_clearance_);
      };
      path_next_goal = roundObservedTurns(path_next_goal, 2.5f*astar->safe_distance_,
                                          2.f*astar->resolution_, clear);
      const float margin=expl_manager_->selected_pass_clearance_<0. ? astar->safe_distance_ :
                         expl_manager_->selected_pass_clearance_;
      path_next_goal=smoothMeasuredBand(path_next_goal,margin,.85f*astar->safe_distance_,
          [&](const Vector3f &p) {return float(astar->lidar_map_interface_->getDisToOcc(p));},clear);
      expl_manager_->improveTransitBend(path_next_goal,fd_->odom_vel_);
      auto departure=alignMeasuredDeparture(path_next_goal, fd_->odom_vel_, astar->safe_distance_,
                                            1.5f*astar->resolution_, clear);
      if (departure!=path_next_goal) {
        path_next_goal.swap(departure);
        ROS_INFO("EPIC joined the observed guide along the measured departure velocity");
      }
      if(expl_manager_->level_passage_ && expl_manager_->intermediate_observation_) {
        auto lane=smoothLevelApproach(path_next_goal,fd_->odom_vel_,margin,astar->resolution_,clear);
        if(!lane.empty()) {
          path_next_goal.swap(lane);
          ROS_INFO("EPIC continued a measured corridor with a gradual height correction");
        }
      }
      const auto &limits=planner_manager_->gcopter_config_;
      const float required_preview=expl_manager_->level_passage_ ? float(2.*astar->safe_distance_+
          levelGuideRenewalDistance(fd_->odom_vel_.norm(),limits->maxAccMag,limits->maxJerkMag,
                                    global_replan_period_+.2,astar->safe_distance_)) : 0.f;
      double verified=expl_manager_->selected_pass_clearance_;
      auto visible=expl_manager_->previewMeasuredPass(path_next_goal,fd_->odom_vel_,required_preview,verified,verified);
      if(visible.size()>1) {
        path_next_goal.swap(visible);
        expl_manager_->selected_pass_clearance_=verified;
      }
      if(expl_manager_->intermediate_observation_ && expl_manager_->followsMeasuredFloor(path_next_goal)) {
        const auto map=astar->lidar_map_interface_;
        const float active=expl_manager_->selected_pass_clearance_<0. ? astar->safe_distance_ :
                           expl_manager_->selected_pass_clearance_;
        auto interior=centerObservedAisle(fd_->odom_vel_,path_next_goal,active,[&](const Vector3f &p) {
          if(!map->IsInBox(p) || p.z()<astar->flight_min_z_ || p.z()>astar->flight_max_z_ ||
             !expl_manager_->frontier_manager_ptr_->observedFree(p)) return 0.f;
          return float(map->getDisToOcc(p));
        },clear);
        if(interior.size()>1) {
          path_next_goal.swap(interior);
          ROS_INFO("EPIC continued through greater measured clearance along the same aisle");
        }
      }
      if(expl_manager_->intermediate_observation_) {
        const float active=expl_manager_->selected_pass_clearance_<0. ? astar->safe_distance_ :
                           expl_manager_->selected_pass_clearance_;
        auto rounded=shapeTransitApproach(path_next_goal,fd_->odom_vel_,active,astar->resolution_,
            [&](const Vector3f &p){return float(astar->lidar_map_interface_->getDisToOcc(p));},clear);
        if(!rounded.empty()) {
          path_next_goal.swap(rounded);
          ROS_INFO("EPIC retained moving preview and clearance through the measured transit bend");
        }
      }
      if(expl_manager_->intermediate_observation_) {
        const float active=expl_manager_->selected_pass_clearance_<0. ? astar->safe_distance_ :
                           expl_manager_->selected_pass_clearance_;
        auto ahead=trimDepartureRetrace(path_next_goal,fd_->odom_vel_,active,clear);
        if(!ahead.empty()) {
          path_next_goal.swap(ahead);
          ROS_INFO("EPIC rejoined ahead of a short reverse projection at departure");
        }
      }
    }
  }
  auto path_next_goal_tmp=evenlySubdivideRoute(path_next_goal,1.f);
  if(path_next_goal_tmp.empty())return FAIL;
  if (external_local_planner_) {
    // Planning can be expensive. Take the latest independent clock and live
    // body state before the first handoff, rather than a queued old epoch.
    if(!external_motion_started_) {
      if(!externalInputsReady(false)) return FAIL;
      if(path_next_goal_tmp.size()<2) return FAIL;
      const float safe=planner_manager_->topo_graph_->parallel_bubble_astar_->safe_distance_;
      if((path_next_goal_tmp.front()-fd_->odom_pos_).norm()>safe) return FAIL;
      FrontierRoute join{fd_->odom_pos_,path_next_goal_tmp[1]};
      const double margin=verified_pass?expl_manager_->selected_pass_clearance_:-1.;
      if(!expl_manager_->observedPass(join) ||
         !planner_manager_->topo_graph_->parallel_bubble_astar_->collisionCheck_shortenPath(join,margin)) return FAIL;
      path_next_goal_tmp.front()=fd_->odom_pos_;
    }
    if(forward_view_yaw) {
      const float safe=planner_manager_->topo_graph_->parallel_bubble_astar_->safe_distance_;
      double horizon=startup_heading_.apply(fd_->odom_yaw_,view_yaw) ?
          1.5*safe : std::numeric_limits<double>::infinity();
      const double departure=startup_heading_.departurePreview(fd_->odom_yaw_,external_odom_stamp_,safe,view_yaw);
      if(departure<0.)return FAIL;
      if(departure>0.) {
        if(!externalInputsReady(false))return FAIL;
        horizon=std::min(horizon,departure);
      }
      if(std::isfinite(horizon)) {
        const auto prefix=FastPlannerManager::boundedGuidePrefix(path_next_goal_tmp,horizon);
        if(prefix.size()<2) return FAIL;
        path_next_goal_tmp.clear();
        for(const auto &p:prefix)path_next_goal_tmp.push_back(p.cast<float>());
        ROS_INFO("EPIC startup heading preview: body_yaw=%.3f reference_yaw=%.3f limited_translation=%.2fm",
            double(fd_->odom_yaw_),view_yaw,horizon);
      }
    }
    nav_msgs::msg::Path guide;
    guide.header.frame_id = "world";
    guide.header.stamp = epic_ros::Time::now();
    if (!fd_->odom_pos_.allFinite()) return FAIL;
    if (path_next_goal_tmp.empty() || (path_next_goal_tmp.front() - fd_->odom_pos_).norm() > .01f)
      return FAIL;
    double guide_length = 0., travelled = 0.;
    for (size_t i = 1; i < path_next_goal_tmp.size(); ++i) guide_length += (path_next_goal_tmp[i]-path_next_goal_tmp[i-1]).norm();
    // Complete the observation turn before the end of a long straight guide.
    // Spreading a modest turn over the entire corridor can postpone the view
    // of a connecting opening until its provisional observation task expires.
    double heading_length=guide_length;
    if(!vertical_step) {
      const auto &limits=planner_manager_->gcopter_config_;
      if(limits->yaw_max_vel>0.) {
        const double speed=std::max(double(fd_->odom_vel_.norm()),limits->maxVelMag);
        const double turn=std::abs(std::remainder(view_yaw-fd_->odom_yaw_,2.*M_PI));
        const double safe=planner_manager_->topo_graph_->parallel_bubble_astar_->safe_distance_;
        heading_length=std::min(guide_length,std::max(safe,turn*speed/limits->yaw_max_vel));
      }
    }
    double previous_yaw=fd_->odom_yaw_;
    for (size_t i = 0; i < path_next_goal_tmp.size(); ++i) {
      const auto &point = path_next_goal_tmp[i];
      if (!point.allFinite()) return FAIL;
      geometry_msgs::msg::PoseStamped pose;
      pose.header = guide.header;
      if (i) travelled += (point-path_next_goal_tmp[i-1]).norm();
      double yaw = guideHeading(fd_->odom_yaw_,view_yaw,travelled,heading_length);
      if(expl_manager_->followingEndTurn()) {
        yaw=i ? routeTangentYaw(path_next_goal_tmp,i,.55f*planner_manager_->topo_graph_->parallel_bubble_astar_->safe_distance_,previous_yaw) : previous_yaw;
        previous_yaw=yaw;
      }
      pose.pose.orientation.z = forward_view_yaw ? std::sin(.5*yaw) : 0.;
      pose.pose.orientation.w = forward_view_yaw ? std::cos(.5*yaw) : 0.;
      pose.pose.position.x = point.x();
      pose.pose.position.y = point.y();
      pose.pose.position.z = point.z();
      // Native viewpoint visibility assumes this body yaw, independently of
      // the direction of translation along the continuing inspection route.
      guide.poses.push_back(pose);
    }
    if(!external_motion_started_) {
      const double reference=external_reference_imu_sub_ ? external_reference_imu_stamp_ :
          external_clock_stamp_>=0. ? external_clock_stamp_ : epic_ros::Time::now().toSec();
      ROS_INFO("EPIC first guide: timestamp_reference=%.3f live_odom=%.3f feedback_age=%.3fs reference_source=%s",
          reference,external_odom_stamp_,reference-external_odom_stamp_,
          external_reference_imu_sub_ ? "raw_imu" : "ros_clock");
    }
    external_path_pub_.publish(guide);
    if(!external_motion_started_) {
      const double reference=external_reference_imu_sub_ ? external_reference_imu_stamp_ :
          external_clock_stamp_>=0. ? external_clock_stamp_ : epic_ros::Time::now().toSec();
      external_handoff_quiet_until_=reference+external_handoff_quiet_duration_;
      ROS_INFO("EPIC initial control handoff: defer heavy planning and guide renewal for %.2fs of sensor time",
          external_handoff_quiet_duration_);
    }
    external_motion_started_=true;
    vertical_remainder_ = std::move(next_vertical_remainder);
    tracking_vertical_step_ = vertical_step;
    tracking_level_passage_ = expl_manager_->level_passage_;
    external_guide_clearance_ = verified_pass ? expl_manager_->selected_pass_clearance_ : -1.;
    expl_manager_->ed_->path_next_goal_.swap(path_next_goal_tmp);
    expl_manager_->commitPass(expl_manager_->ed_->path_next_goal_);
    const auto target=expl_manager_->ed_->next_goal_node_->center_;
    ROS_INFO("External native goal=(%.2f, %.2f, %.2f), body=(%.2f, %.2f, %.2f), guide_vertices=%zu view_yaw=%.3f body_yaw=%.3f forward_yaw=%d",
        target.x(),target.y(),target.z(),fd_->odom_pos_.x(),fd_->odom_pos_.y(),fd_->odom_pos_.z(),guide.poses.size(),view_yaw,double(fd_->odom_yaw_),int(forward_view_yaw));
    external_route_progress_=0.;external_previous_position_=fd_->odom_pos_;
    external_route_goal_ = target;
    external_guide_stamp_ = epic_ros::Time::now().toSec();
    external_guide_map_stamp_=expl_manager_->latestPassScan();
    external_progress_watch_=0.;
    if (!expl_manager_->corridor_pass_enabled_ || external_progress_watch_stamp_ < 0.) {
      external_progress_watch_stamp_ = external_guide_stamp_;
      external_watch_position_ = fd_->odom_pos_;
    }
    return SUCCEED;
  }
  expl_manager_->ed_->path_next_goal_.swap(path_next_goal_tmp);
  if (planner_manager_->planExploreTraj(expl_manager_->ed_->path_next_goal_, fd_->static_state_)) {
    expl_manager_->commitPass(expl_manager_->ed_->path_next_goal_);
    epic_ros2::msg::PolyTraj poly_traj_msg;
    planner_manager_->polyTraj2ROSMsg(poly_traj_msg, info->start_time_);
    fd_->newest_traj_ = poly_traj_msg;
    epic_ros2::msg::PolyTraj poly_yaw_traj_msg;
    planner_manager_->polyYawTraj2ROSMsg(poly_yaw_traj_msg, info->start_time_);
    fd_->newest_yaw_traj_ = poly_yaw_traj_msg;
    return SUCCEED;
  } else {
    return FAIL;
  }
}

void FastExplorationFSM::holdExternalGuide(const Vector3f &previous) {
  // The external path follower accepts two distinct vertices. End precisely
  // at the measured body position so it retires the old guide and holds.
  const Vector3f body = fd_->odom_pos_;
  Vector3f toward = previous - body;
  if (toward.norm() < .001f) {
    const auto &published = expl_manager_->ed_->path_next_goal_;
    for (const auto &p : published) {
      toward = p - body;
      if (toward.norm() > .001f) break;
    }
  }
  if (toward.norm() < .001f) return;
  const Vector3f from = body + std::min(.03f, toward.norm()) * toward.normalized();
  nav_msgs::msg::Path hold;
  hold.header.frame_id = "world";
  hold.header.stamp = epic_ros::Time::now();
  for (const auto &p : {from, body}) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = hold.header;
    pose.pose.position.x = p.x(); pose.pose.position.y = p.y(); pose.pose.position.z = p.z();
    pose.pose.orientation.z = std::sin(.5 * fd_->odom_yaw_);
    pose.pose.orientation.w = std::cos(.5 * fd_->odom_yaw_);
    hold.poses.push_back(pose);
  }
  external_path_pub_.publish(hold);
}

void FastExplorationFSM::triggerCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg) {
  if (msg->poses.empty() || msg->poses[0].pose.position.z < -0.1)
    return;

  if (state_ != WAIT_TRIGGER)
    return;
  fd_->trigger_ = true;
  expl_manager_->frontier_empty_since_=-1.;
  cout << "Triggered!" << endl;
  total_time_ = epic_ros::Time::now().toSec();
  transitState(PLAN_TRAJ, "triggerCallback");
  if (external_local_planner_) updateTopoAndGlobalPath();
}

void FastExplorationFSM::CloudOdomCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, const nav_msgs::msg::Odometry::ConstSharedPtr &odom_) {
  // Let the estimator catch up instead of spending CPU on
  // historical frontier updates while the live vehicle feedback is stale.
  if(external_local_planner_ && !externalInputsReady(false)) return;
  epic_ros::Time t1 = epic_ros::Time::now();
  planner_manager_->lidar_map_interface_->updateCloudMapOdometry(msg, odom_);
  double collision_time;
  bool safe = external_local_planner_ || planner_manager_->checkTrajCollision(collision_time);
  if (!safe) {
    transitState(PLAN_TRAJ, "safetyCallback: not safe, time:" + to_string(collision_time), true);
    const double braking_acc = planner_manager_->gcopter_config_->maxAccMag > 0.0 ?
        planner_manager_->gcopter_config_->maxAccMag : 3.0;
    if (collision_time < std::max(fp_->replan_time_ + 0.2,
                                 fd_->odom_vel_.norm() / braking_acc + 0.2))
      stopTraj();
  }
  epic_ros::Time t2 = epic_ros::Time::now();
  epic_ros::Time t3 = epic_ros::Time::now();

  if (planner_manager_->lidar_map_interface_->ld_->lidar_cloud_.points.empty())
    return;
  auto& ld = planner_manager_->lidar_map_interface_->ld_;
  if (!external_local_planner_) {
    fd_->odom_pos_ = ld->lidar_pose_;
    fd_->odom_vel_ = ld->lidar_vel_;
    fd_->odom_yaw_ = (float)tf2::getYaw(odom_->pose.pose.orientation);
  } else if (!external_state_received_) {
    odometryCallback(*odom_);
    if (!external_state_received_) return;
  }
  planner_manager_->local_data_.curr_pos_ = fd_->odom_pos_.cast<double>();
  planner_manager_->local_data_.curr_vel_ = fd_->odom_vel_.cast<double>();
  planner_manager_->topo_graph_->odom_node_->center_ = fd_->odom_pos_;
  fd_->have_odom_ = true;
  vector<ClusterInfo::Ptr> new_clusters;
  vector<int> cluster_removed;
  expl_manager_->frontier_manager_ptr_->updateFrontierClusters(new_clusters, cluster_removed);
  const double scan_stamp=epic_ros::Time(msg->header.stamp).toSec();
  expl_manager_->observePass(fd_->odom_pos_,scan_stamp);
  renewGuideFromScan(scan_stamp);
  for (auto &cls : new_clusters) {
    cls->odom_id_ = planner_manager_->topo_graph_->history_odom_nodes_.size() - 1;
  }
  epic_ros::Time t4 = epic_ros::Time::now();

  ROS_INFO_STREAM_THROTTLE(1.0, "cloud odom callback cost: " << "ikd-tree insert:" << (t2 - t1).toSec() * 1000 << "ms  "
                                                             << "update frontier clusters: " << (t4 - t3).toSec() * 1000 << "ms  "
                                                             << "total: " << (t4 - t1).toSec() * 1000 << "ms" << endl);
}

bool FastExplorationFSM::externalInputsReady(bool settle) {
  if(!external_local_planner_) return true;
  refreshExternalOdom();
  if(external_clock_sub_) {
    rosgraph_msgs::msg::Clock latest;rclcpp::MessageInfo info;
    if(external_clock_sub_->take(latest,info))
      external_clock_stamp_=double(latest.clock.sec)+1.e-9*latest.clock.nanosec;
  }
  if(external_reference_imu_sub_) {
    sensor_msgs::msg::Imu latest;rclcpp::MessageInfo info;
    if(external_reference_imu_sub_->take(latest,info))
      external_reference_imu_stamp_=double(latest.header.stamp.sec)+1.e-9*latest.header.stamp.nanosec;
  }
  // /clock and body odometry can both be republished by the same delayed
  // adapter. Compare against the raw sensor epoch, not that circular clock.
  // The configured independent source is mandatory; do not fall back while
  // its first sample is missing. Only timestamps are used, never truth poses.
  const double reference=external_reference_imu_sub_ ? external_reference_imu_stamp_ :
      external_clock_stamp_>=0. ? external_clock_stamp_ : epic_ros::Time::now().toSec();
  const bool ready=external_state_received_ && external_input_freshness_.observe(reference,external_odom_stamp_,settle);
  if(!ready) ROS_WARN_THROTTLE(2.,"EPIC waiting for fresh live state: reference=%.3f odometry=%.3f age=%.3fs startup_confirmation=%d",
      reference,external_odom_stamp_,reference-external_odom_stamp_,int(settle));
  // A deliberately short first preview must not trigger the ordinary short-
  // remaining-route extension loop while the local pipeline is starting.
  // Keep receiving raw sensor/body state, but yield heavy work until that
  // first local handoff has had time to execute and settle.
  return ready && (!external_motion_started_ || reference>=external_handoff_quiet_until_);
}

void FastExplorationFSM::refreshExternalOdom() {
  if (!external_odom_sub_) return;
  nav_msgs::msg::Odometry latest;
  rclcpp::MessageInfo info;
  if (external_odom_sub_->take(latest,info)) odometryCallback(latest);
}

void FastExplorationFSM::odometryCallback(const nav_msgs::msg::Odometry &msg) {
  if (!external_local_planner_ || msg.header.frame_id != "world") return;
  const auto &p = msg.pose.pose.position;
  const auto &v = msg.twist.twist.linear;
  const auto &q = msg.pose.pose.orientation;
  Eigen::Vector3f position(p.x, p.y, p.z), velocity(v.x, v.y, v.z);
  Eigen::Quaternionf attitude(q.w, q.x, q.y, q.z);
  const double stamp = double(msg.header.stamp.sec) + double(msg.header.stamp.nanosec) * 1e-9;
  if (!position.allFinite() || !velocity.allFinite() || !attitude.coeffs().allFinite() ||
      std::abs(attitude.norm() - 1.f) > .01f || stamp <= external_odom_stamp_) return;
  external_odom_stamp_ = stamp;
  external_state_received_ = true;
  fd_->odom_pos_ = position;
  // SUPER publishes twist in world; its matched cloud still uses its original
  // measurement epoch in updateCloudMapOdometry. Never overwrite current body
  // state with that older scan pair when selecting/publishing global guides.
  fd_->odom_vel_ = planner_manager_->lidar_map_interface_->lp_->odom_twist_in_world_ ?
      velocity : attitude * velocity;
  fd_->odom_yaw_ = float(tf2::getYaw(q));
  planner_manager_->local_data_.curr_pos_ = fd_->odom_pos_.cast<double>();
  planner_manager_->local_data_.curr_vel_ = fd_->odom_vel_.cast<double>();
  planner_manager_->local_data_.curr_yaw_ = fd_->odom_yaw_;
}

void FastExplorationFSM::transitState(EXPL_STATE new_state, string pos_call, bool red) {
  if (state_ == LAND && new_state != LAND) return;
  if (!fd_->trigger_ && new_state == CAUTION) return;
  int pre_s = int(state_);
  if(pre_s!=int(new_state)) {
    external_finish_rest_=InputFreshness{};
    external_finish_hold_sent_=false;
  }
  if (external_local_planner_ && (new_state==WAIT_MAP || new_state==FINISH))
    external_progress_watch_stamp_=-1.;
  state_ = new_state;
  if (!red) {
    cout << "\033[32m[" + pos_call + "]\033[0m: from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)] << endl;
  } else {
    cout << "\033[31m[" + pos_call + "]\033[0m: from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)] << endl;
  }
}

void FastExplorationFSM::stopTraj() {
  if (state_ == LAND) return;
  // Never truncate a moving polynomial into a zero-velocity hold.
  // PX4 NAV_LAND owns braking and descent once exploration becomes unsafe.
  fd_->trigger_ = false;
  transitState(LAND, "exploration stopped; request PX4 landing", true);
  epic_ros2::msg::TakeoffLand land_msg;
  land_msg.takeoff_land_cmd = land_msg.LAND;
  land_pub_.publish(land_msg);
}

void FastExplorationFSM::landRequestedCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg) {
  if (msg->data && fd_->trigger_ && state_ != LAND) stopTraj();
}
