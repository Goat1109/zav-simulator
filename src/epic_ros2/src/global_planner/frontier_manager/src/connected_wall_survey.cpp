#include <frontier_manager/frontier_manager.h>
#include <epic_ros2/planning_budget.hpp>

TopoNode::Ptr FrontierManager::connectedWallViewpoint(const Eigen::Vector3f& body) {
  using V3=Eigen::Vector3f;using Route=std::vector<V3>;
  const float radius=graph_->bubble_min_radius_,cell=frtp_.cell_size_;
  // recordPassage() is the sole measurement stream feeding visitation.
  // Mixing its scan-synchronised pose with this newer planning pose
  // would count latency as repeated forward/backward motion.
  surface_body_=body;wall_traversal_.clearProposedEntry();
  // Sample locally while the committed chain runs. Other measured departures
  // remain in the journal instead of competing with its unfinished suffix.
  vector<TopoNode::Ptr> samples;
  {
    epic_ros::PlanningBudget::Scope sampling(.35);
    V3 sample_origin=body;generateTSPViewpoints(sample_origin,samples);
  }
  // Native topology/frontier stances also reveal branches that a local
  // outline has not yet joined. Give each a current, certified connector;
  // an unconnected viewpoint never becomes an executable journal edge.
  for(const auto& sample:samples){
    if(epic_ros::PlanningBudget::expired())break;
    auto it=observations_.find(sample.get());if(it==observations_.end() || !it->second->contour_approach_.empty())continue;
    Route join;
    if(connectObserved(body,sample->center_,.015,join) && certifyFlightGuide(join)){
      auto copy=std::make_shared<ClusterInfo>(*it->second);
      copy->contour_approach_=join;copy->contour_preview_={join.back()};it->second=copy;
    }
  }
  for(const auto& sample:samples){
    const auto it=observations_.find(sample.get());if(it==observations_.end())continue;
    auto route=it->second->contour_approach_;
    const auto& preview=it->second->contour_preview_;
    if(route.size()>1 && preview.size()>1 && (route.back()-preview.front()).norm()<.1f)
      route.insert(route.end(),preview.begin()+1,preview.end());
    if(certifyFlightGuide(route))wall_traversal_.offer(route);
  }
  const auto make=[&](Route route,const char* reason)->TopoNode::Ptr {
    if(route.size()<2 || !certifyFlightGuide(route))return {};
    // Keep a bounded first leg and its actual corners. Long routes can be
    // resumed from measured motion; a short branch needs no minimum gain.
    Route bounded{route.front()};float arc=0.f;V3 first_direction=V3::Zero();
    for(size_t i=1;i<route.size();++i){
      const V3 edge=route[i]-bounded.back();const float length=edge.norm();
      if(arc+length>sensorUpdateRange() && arc>2.f*radius)break;
      if(length>.1f){
        if(first_direction.squaredNorm()<.1f)first_direction=edge/length;
        // Stop at a measured sharp junction before choosing the next wall.
        // SUPER can brake on this prefix instead of reversing a moving goal.
        if(arc>2.f*radius && first_direction.dot(edge/length)<.5f)break;
      }
      bounded.push_back(route[i]);arc+=length;
    }
    // Intermediate measured constrictions may use the aperture clearance,
    // but an external-planner endpoint must satisfy its normal stop clearance.
    while(bounded.size()>1 && lidar_map_interface_->getDisToOcc(bounded.back())<
          graph_->parallel_bubble_astar_->safe_distance_+.01f)bounded.pop_back();
    if(bounded.size()<2 || (bounded.back()-body).norm()<.35f || !wall_traversal_.mayFollow(bounded) || !certifyFlightGuide(bounded))return {};
    auto view=std::make_shared<TopoNode>();view->is_viewpoint_=true;view->center_=bounded.back();
    const V3 last=bounded.back()-bounded[bounded.size()-2];view->yaw_=std::atan2(last.y(),last.x());
    auto observation=std::make_shared<ClusterInfo>();observation->center_=view->center_;
    observation->follows_local_outline_=true;observation->close_surface_sample_=true;
    observation->contour_approach_=bounded;observation->contour_preview_={bounded.back()};
    observation->fresh_contour_length_=wall_traversal_.freshLength(bounded);
    observations_[view.get()]=observation;perimeter_target_=view->center_;
    if(wall_traversal_reason_!=reason){
      ROS_INFO("Connected wall: %s, pending branches %zu, measured travel %.2fm",reason,
        wall_traversal_.branches().size(),wall_traversal_.travel());wall_traversal_reason_=reason;
    }
    return view;
  };
  V3 heading(std::cos(graph_->odom_node_->yaw_),std::sin(graph_->odom_node_->yaw_),0.f);
  if(wall_traversal_.heading())heading=*wall_traversal_.heading();
  else if(auto recent=recentFlightDirection(body))heading={recent->x(),recent->y(),0.f};
  // A newly exposed, certified right-hand entrance is part of the same
  // boundary, not an arbitrary competing observation. A provisional guide
  // sampled before the junction was visible must not lock it out.
  const auto opening=[&](const Route& route){
    return route.size()>1 && wall_traversal_.freshLength(route)>=cell &&
      surveyDeparture(body,heading.head<2>(),route,true,true)==Departure::RightOpening &&
      followsRightWall(route);
  };
  auto remaining=wall_traversal_.remaining(body);
  if(remaining.size()>1){
    if(auto view=make(remaining,"retain unfinished directed wall")){wall_blocked_scans_=0;return view;}
    // A changed map may invalidate an issued suffix. Retry using a certified
    // join, then require independent new scans before abandoning the chain.
    Route joined;
    if(!epic_ros::PlanningBudget::expired() && connectObserved(body,remaining.back(),.012,joined))
      if(auto view=make(joined,"reattach to unfinished wall"))return view;
    if(wall_checked_scan_!=surface_scan_){wall_checked_scan_=surface_scan_;++wall_blocked_scans_;}
    if(wall_blocked_scans_<3)return {};
    wall_traversal_.offer(remaining);wall_traversal_.clearActive();
  }
  if(lidar_map_interface_->ld_->lidar_vel_.norm()>=.3f)return {};
  if(!opening(remaining))for(const auto& sample:samples){
    const auto it=observations_.find(sample.get());if(it==observations_.end())continue;
    auto route=it->second->contour_approach_;const auto& preview=it->second->contour_preview_;
    if(route.size()>1 && preview.size()>1 && (route.back()-preview.front()).norm()<.1f)
      route.insert(route.end(),preview.begin()+1,preview.end());
    if(opening(route))if(auto view=make(route,"enter newly measured connected right branch")){
      if(remaining.size()>1)wall_traversal_.offer(remaining);
      wall_traversal_.proposeEntry(body,V3(heading.y(),-heading.x(),0.f));
      wall_blocked_scans_=0;return view;
    }
  }

  if(!opening(remaining)){
    // A short branch can be fully visible from the junction and therefore
    // have no frontier information gain. Probe certified free air beside
    // the incoming wall; directed motion coverage, not gain, decides entry.
    const Eigen::Vector2f forward=heading.head<2>().normalized(),right(forward.y(),-forward.x());
    for(float along=0.f;along<=4.f*radius && !epic_ros::PlanningBudget::expired();along+=radius){
      for(float side=frtp_.good_observation_trust_length_;side>=2.f*radius && !epic_ros::PlanningBudget::expired();side-=radius){
        V3 target=body;target.head<2>()+=along*forward+side*right;
        if(!withinFlightHeight(target) || !observedAir(target) ||
           wall_traversal_.visited(target,V3(right.x(),right.y(),0.f)) ||
           lidar_map_interface_->getDisToOcc(target)<measuredStanceClearance())continue;
        Route route;
        if(connectObserved(body,target,.008,route) && opening(route))
          if(auto view=make(route,"enter measured unvisited right-side air")){
            if(remaining.size()>1)wall_traversal_.offer(remaining);
            wall_traversal_.proposeEntry(body,V3(heading.y(),-heading.x(),0.f));
            wall_blocked_scans_=0;return view;
          }
      }
    }
  }
  const auto departure=[&](const Route& route){
    for(const auto& p:route)if((p-body).head<2>().norm()>radius)return (p-body).head<2>().normalized().dot(heading.head<2>().normalized());
    return -1.f;
  };
  if(!epic_ros::PlanningBudget::expired()){
    auto rolling=previewRightWall(body,heading.head<2>());
    if(rolling.size()>1 && wall_traversal_.freshLength(rolling)>=cell)
      if(auto view=make(rolling,"continue measured right face through corner")){wall_blocked_scans_=0;return view;}
  }
  // The native contour samples are already ordered around the right face.
  // Do not replace a connected forward arc with a remote information gain.
  for(const auto& sample:samples){
    const auto it=observations_.find(sample.get());if(it==observations_.end())continue;
    auto route=it->second->contour_approach_;const auto& preview=it->second->contour_preview_;
    if(route.size()>1 && preview.size()>1 && (route.back()-preview.front()).norm()<.1f)
      route.insert(route.end(),preview.begin()+1,preview.end());
    if(wall_traversal_.freshLength(route)<cell || departure(route)<-.25f)continue;
    const bool attached=it->second->follows_local_outline_ ||
        keepsRightWall(route,std::min(epic_ros::WallTraversal::length(route),frtp_.good_observation_trust_length_));
    if(!attached)continue;
    if(auto view=make(route,"advance connected unvisited wall")){wall_blocked_scans_=0;return view;}
  }
  // A sparse wall slice is not a measured dead end. Move only through
  // already observed, certified forward air to obtain a new baseline while
  // retaining the same wall direction. Never extrapolate into unknown air.
  for(float distance=frtp_.good_observation_trust_length_;distance>=2.f*radius && !epic_ros::PlanningBudget::expired();distance-=radius){
    V3 target=body+distance*heading;
    if(!withinFlightHeight(target) || !observedAir(target) || wall_traversal_.visited(target,heading))continue;
    if(auto view=make(Route{body,target},"remeasure continuation through known forward air")){wall_blocked_scans_=0;return view;}
  }
  if(wall_checked_scan_!=surface_scan_){wall_checked_scan_=surface_scan_;++wall_blocked_scans_;}
  if(wall_blocked_scans_<3)return {};
  if(lidar_map_interface_->ld_->lidar_vel_.norm()>=.3f)return {};
  // A dead end can legitimately require reversing. Pick a deferred directed
  // branch only after forward continuation failed on multiple real scans.
  // Arrival at a nearby body voxel never retires the opposite directed pass.
  wall_traversal_.beginReturn();
  std::vector<epic_ros::WallTraversal::Branch> pending=wall_traversal_.branches();
  std::sort(pending.begin(),pending.end(),[&](const auto& a,const auto& b){
    return (a.route.front()-body).squaredNorm()<(b.route.front()-body).squaredNorm();});
  for(const auto& branch:pending){
    if(epic_ros::PlanningBudget::expired())break;
    const auto& route=branch.route;
    for(size_t i=1;i<route.size();++i){
      const V3 edge=route[i]-route[i-1];if(edge.norm()<.01f || wall_traversal_.visited(route[i],edge.normalized()))continue;
      Route join;
      if(!connectObserved(body,route[i],.008,join))continue;
      join.insert(join.end(),route.begin()+i+1,route.end());
      if(auto view=make(join,"resume deferred unvisited branch"))return view;
      break;
    }
  }
  // Initial acquisition uses only measured free air; no assumed corridor end.
  if(wall_traversal_.travel()<radius){
    Route baseline;if(initialSurveyGuide(body,baseline))return make(baseline,"acquire measured wall");
  }
  return {};
}
