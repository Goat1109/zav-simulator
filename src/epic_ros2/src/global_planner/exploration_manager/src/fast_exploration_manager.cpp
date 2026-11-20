#include <epic_ros2/planning_workers.hpp>
#include <numeric>
#include <epic_planner/frontier_route.h>
/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2024-02-25 15:00:51
 * @LastEditTime: 2024-03-12 22:15:11
 * @Description:
 * @
 * @Copyright (c) 2024 by ning-zelin, All Rights Reserved.
 */

#include <boost/lexical_cast.hpp>
#include <epic_planner/expl_data.h>
#include <epic_planner/fast_exploration_manager.h>
#include <fstream>
#include <iostream>
#include <lkh_tsp_solver/lkh_interface.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <plan_manage/planner_manager.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <visualization_msgs/msg/marker.hpp>
using namespace std;
using namespace Eigen;

namespace fast_planner {
namespace {
constexpr float narrowObservationRadiusFactor = 11.f;
}
// SECTION interfaces for setup and query

FastExplorationManager::FastExplorationManager() {}

FastExplorationManager::~FastExplorationManager() {}

void FastExplorationManager::initialize(
    epic_ros::NodeHandle &nh, FrontierManager::Ptr frt_manager,
    FastPlannerManager::Ptr planner_manager) {

  frontier_manager_ptr_ = frt_manager;
  planner_manager_ = planner_manager;

  ed_.reset(new ExplorationData);
  ep_.reset(new ExplorationParam);
  ed_->next_goal_node_ = make_shared<TopoNode>();
  ed_->next_goal_node_->center_.setZero();
  ed_->next_goal_node_->yaw_ = 0.f;

  ep_->a_avg_ = tan(planner_manager_->gcopter_config_->maxTiltAngle) *
                planner_manager_->gcopter_config_->gravAcc;
  ep_->v_max_ = planner_manager_->gcopter_config_->maxVelMag;
  ep_->yaw_v_max_ = planner_manager_->gcopter_config_->yaw_max_vel;
  nh.param("exploration/tsp_dir", ep_->tsp_dir_, string("null"));
  nh.getParam("viewpoint_param/global_viewpoint_num",
              ep_->global_viewpoint_num_);
  nh.getParam("view_graph", ep_->view_graph_);
  nh.getParam("viewpoint_param/local_viewpoint_num", ep_->local_viewpoint_num_);
  nh.getParam("global_planning/w_vdir", ep_->w_vdir_);
  nh.param("global_planning/use_velocity_direction_cost",use_velocity_direction_cost_,false);
  nh.param("global_planning/directional_frontiers",directional_frontiers_,false);
  nh.param("global_planning/corridor_pass", corridor_pass_enabled_, false);
  if (corridor_pass_enabled_) {
    nh.param("global_planning/floor_by_floor",floor_by_floor_,false);
    nh.param("exploration/tour_lookahead_distance", level_preview_distance_, 0.);
    if (!std::isfinite(level_preview_distance_))
      throw std::invalid_argument("Invalid level guide preview distance");
    double confirmation, gain_ratio;
    nh.param("bubble_astar/endpoint_height_margin", pass_height_margin_, -1.);
    nh.param("global_planning/no_forward_confirmation", confirmation, 1.5);
    nh.param("global_planning/return_gain_ratio", gain_ratio, 2.5);
    pass_.configure(planner_manager_->topo_graph_->parallel_bubble_astar_->safe_distance_,
                    confirmation, gain_ratio);
  }
  if(use_velocity_direction_cost_)ROS_INFO("Global first-leg direction cost enabled: w=%.2f, native LKH retained",ep_->w_vdir_);
  nh.getParam("global_planning/w_yawdir", ep_->w_yawdir_);
  Eigen::Vector3d origin, size;
  ofstream par_file(ep_->tsp_dir_ + "/single.par");
  par_file << "PROBLEM_FILE = " << ep_->tsp_dir_ << "/single.tsp\n";
  par_file << "GAIN23 = NO\n";
  par_file << "MOVE_TYPE = 2\n";
  par_file << "OUTPUT_TOUR_FILE =" << ep_->tsp_dir_ << "/single.txt\n";
  par_file << "RUNS = 10\n";
  epic_ros::Duration(1.0).sleep();
}

void FastExplorationManager::goalCallback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg) {
  // 提取四元数
  double roll, pitch;
  tf2::Quaternion quat;
  tf2::fromMsg(msg->pose.orientation, quat);

  // 将四元数转换为Euler角
  tf2::Matrix3x3(quat).getRPY(roll, pitch, goal_yaw);
}

double FastExplorationManager::initialDirectionCost(
    const vector<Eigen::Vector3f> &path,const Eigen::Vector3d &velocity,
    double weight,double yaw_rate) {
  if(path.size()<2 || !velocity.allFinite() || velocity.head<2>().norm()<=.5 ||
      !std::isfinite(weight) || weight<=0. || !std::isfinite(yaw_rate) || yaw_rate<=0.)return 0.;
  // Use the first2m of the certified native graph route, not the direct
  // goal bearing which can point through a shelf on an otherwise forward route.
  Eigen::Vector3f point=path.front();double remaining=2.;
  if(!point.allFinite())return 0.;
  for(size_t i=1;i<path.size();++i) {
    if(!path[i].allFinite())return 0.;
    const Eigen::Vector3f segment=path[i]-point;const double length=segment.norm();
    if(length<1.e-6)continue;
    if(length>=remaining){point+=segment*float(remaining/length);break;}
    point=path[i];remaining-=length;
  }
  const Eigen::Vector2d direction=(point-path.front()).head<2>().cast<double>();
  if(direction.norm()<.1)return 0.;
  const double angle=std::abs(std::remainder(atan2(direction.y(),direction.x())-
                     atan2(velocity.y(),velocity.x()),2.*M_PI));
  return weight*angle/yaw_rate;
}

double FastExplorationManager::getPathCost(TopoNode::Ptr &n1,
                                           Eigen::Vector3d v1, float &yaw1,
                                           TopoNode::Ptr &n2, float &yaw2,
                                           double *initial_turn_cost, FrontierRoute *verified_path) {
  if(initial_turn_cost)*initial_turn_cost=0.;
  auto estimateCost = [&](TopoNode::Ptr &n1, Eigen::Vector3d v1, float &yaw1,
                          TopoNode::Ptr &n2, float &yaw2, int res,
                          vector<Eigen::Vector3f> &path) -> double {
    double len_cost, yaw_cost, dir_cost;
    len_cost = yaw_cost = dir_cost = 0.0;
    if (res != BubbleAstar::REACH_END || path.size() < 2)
      return 2e3 +
             (n1->center_ - n2->center_).norm(); // 同上，用于不同的错误情况

    len_cost = 0.0;
    for (int i = 0; i < path.size() - 1; ++i)
      len_cost += ((path[i + 1] - path[i]).norm() +
                   0.5 * fabs(path[i + 1].z() - path[i].z()));
    len_cost /= (ep_->v_max_ / 2.0);
    if(corridor_pass_enabled_) {
      const auto &limits=planner_manager_->gcopter_config_;
      const double predicted=estimateTransitTime(path,v1.cast<float>(),
          planner_manager_->topo_graph_->parallel_bubble_astar_->safe_distance_,
          limits->maxVelMag,limits->maxAccMag,limits->maxJerkMag);
      if(std::isfinite(predicted)) len_cost=predicted;
    }

    // if (v1.norm() > 1e-3) {
    //   Eigen::Vector3f dir = n2->center_ - n1->center_;
    //   dir.normalize();
    //   Eigen::Vector3f v_dir = v1.normalized().cast<float>();
    //   float yaw1 = atan2(dir.y(), dir.x());
    //   float yaw2 = atan2(v_dir.y(), v_dir.x());
    //   float diff = yaw1 - yaw2;
    //   while (diff > M_PI)
    //     diff -= 2.0 * M_PI;
    //   while (diff < -M_PI)
    //     diff += 2.0 * M_PI;
    //   dir_cost = ep_->w_vdir_ * (fabs(diff) /
    //   planner_manager_->gcopter_config_->yaw_max_vel);
    // }

    // if (path.size() >= 2) {
    //   planner_manager_->calculateTimelb(path, yaw1, yaw2, yaw_cost);
    //   yaw_cost *= ep_->w_yawdir_;
    // }

    return len_cost + dir_cost;
    // return len_cost + dir_cost;
  };
  vector<Eigen::Vector3f> path;
  int res = planner_manager_->fast_searcher_->topoSearch(n1, n2, 1e-2, path);
  if (verified_path) {
    verified_path->clear();
    if (res != BubbleAstar::REACH_END || path.size() < 2 ||
        (path.back() - n2->center_).norm() > .2f) return 2e3;
    path.front() = n1->center_;
    path.back() = n2->center_;
    if (!certifyPassGuide(path))
      return 2e3;
    *verified_path = path;
  }
  const double cost=estimateCost(n1, v1, yaw1, n2, yaw2, res, path);
  if(initial_turn_cost && cost<2e3)
    *initial_turn_cost=directional_frontiers_ ? frontierContinuationCost(path,v1.cast<float>()) :
        initialDirectionCost(path,v1,ep_->w_vdir_,planner_manager_->gcopter_config_->yaw_max_vel);
  return cost;
}

double FastExplorationManager::getPathCostWithoutTopo(TopoNode::Ptr &n1,
                                                      Eigen::Vector3d v1,
                                                      float &yaw1,
                                                      TopoNode::Ptr &n2,
                                                      float &yaw2) {
  vector<Eigen::Vector3f> path;
  int res = planner_manager_->parallel_path_finder_->search(
      n1->center_, n2->center_, path, 1.0, false);
  if (res != ParallelBubbleAstar::REACH_END)
    return 2e3;
  double cost;
  planner_manager_->parallel_path_finder_->calculatePathCost(path, cost);
  return cost;
}

bool FastExplorationManager::hasReturnWork(const Vector3f &body,float clearance) const {
  if(!floor_model_supported_ || !floor_trace_.needsReturn(body,clearance) || !floor_trace_.entrance())
    return false;
  const Vector3f entrance=*floor_trace_.entrance();
  return frontier_manager_ptr_->unobservedViewGain(entrance,clearance)>0.f ||
      !measuredRises(entrance,clearance).empty();
}

int FastExplorationManager::frontierStatus(bool screened) {
  if (floor_by_floor_ && hasReturnWork(planner_manager_->topo_graph_->odom_node_->center_,
      planner_manager_->topo_graph_->parallel_bubble_astar_->safe_distance_)) return FRONTIER_PENDING;
  if(!screened && frontier_manager_ptr_->hasInspectionFrontiers()) {
    frontier_empty_since_=-1.;return FRONTIER_PENDING;
  }
  // Completion needs fresh sensor time, even when a planning callback was
  // expensive. Repeated computation on one old scan cannot confirm closure.
  const double now=corridor_pass_enabled_?pass_scan_stamp_:epic_ros::Time::now().toSec();
  if(now<0.) return FRONTIER_PENDING;
  if(frontier_empty_since_<0.)frontier_empty_since_=now;
  epic_ros::NodeHandle nh;double confirm;
  nh.param("exploration/frontier_empty_confirmation",confirm,0.);
  return now-frontier_empty_since_<confirm ? FRONTIER_PENDING : NO_FRONTIER;
}

int FastExplorationManager::planGlobalPath(const Eigen::Vector3d &pos,
                                           const Eigen::Vector3d &vel) {
  bool bm_without_topo = false;
  auto estimiateVdirCost = [&](const TopoNode::Ptr &n1,
                               const Eigen::Vector3d &v1,
                               const TopoNode::Ptr &n2) -> double {
    Eigen::Vector3f dir = n2->center_ - n1->center_;
    dir.normalize();
    Eigen::Vector3f v_dir = v1.normalized().cast<float>();
    float yaw1 = atan2(dir.y(), dir.x());
    float yaw2 = atan2(v_dir.y(), v_dir.x());
    float diff = yaw1 - yaw2;
    while (diff > M_PI)
      diff -= 2.0 * M_PI;
    while (diff < -M_PI)
      diff += 2.0 * M_PI;
    return ep_->w_vdir_ *
           (fabs(diff) / planner_manager_->gcopter_config_->yaw_max_vel);
  };
  epic_ros::Time start = epic_ros::Time::now();
  vector<TopoNode::Ptr> viewpoints;
  // Visitation/handoff uses the latest measured body pose. The graph odom
  // anchor follows scan updates and can lag behind a fast moving vehicle.
  // Keep that graph anchor intact for certified native routing.
  Eigen::Vector3f current_body_position=pos.cast<float>();
  Eigen::Vector3d selection_velocity=vel;
  if (corridor_pass_enabled_) {
    pass_.observe(current_body_position);
    preferred_heading_ = pass_.heading();
    selection_velocity = preferred_heading_.cast<double>();
  } else if(directional_frontiers_) {
    if(vel.head<2>().norm()>.75)preferred_heading_=vel.cast<float>().normalized();
    selection_velocity=preferred_heading_.cast<double>();
  }
  end_turn_.reset();
  level_passage_ = false;
  selected_pass_clearance_ = -1.;
  if(floor_by_floor_) {
    const float safe=planner_manager_->topo_graph_->parallel_bubble_astar_->safe_distance_;
    FrontierRoute outlet{current_body_position};
    if(inspectStoreyOutlet(outlet,safe)) return SUCCEED;
    const bool landing_return=floor_trace_.entrance() && floor_trace_.surveyedLevel(current_body_position,safe) &&
        (current_body_position-*floor_trace_.entrance()).head<2>().norm()<7.f*safe;
    if((floor_trace_.climbingAt(current_body_position,safe) && !visibleLevelOutlet(current_body_position)) ||
       (landing_return && readyToPreviewAscent(current_body_position,current_body_position))) {
      const auto rise=planRisingOpening(current_body_position);
      if(rise && *rise==SUCCEED) return SUCCEED;
    }
  }
  if (selectLevelPassage(current_body_position)) return SUCCEED;
  vector<double> visible_areas;
  vector<vector<ScoredViewpoint>> alternatives;
  const bool truncated = frontier_manager_ptr_->generateTSPViewpoints(
      current_body_position, viewpoints, visible_areas, expand_pass_search_,
      corridor_pass_enabled_ ? &alternatives : nullptr);
  if (floor_by_floor_ && floor_model_supported_) {
    const float safe = planner_manager_->topo_graph_->parallel_bubble_astar_->safe_distance_;
    bool local_frontier = false;
    for (const auto &vp : viewpoints) {
      if (std::abs(vp->center_.z()-current_body_position.z())<1.6f*safe &&
          !frontier_manager_ptr_->recentlySurveyed(vp->center_,2.2f*safe,1.5f*safe)) local_frontier=true;
    }
    for (size_t i = viewpoints.size(); i-- > 0;) {
      const auto &p = viewpoints[i]->center_;
      const float dz = p.z()-current_body_position.z();
      if (dz >= -2.f*safe && (!local_frontier || std::abs(dz)<1.6f*safe) &&
          !frontier_manager_ptr_->recentlySurveyed(p,1.6f*safe,1.3f*safe)) continue;
      viewpoints.erase(viewpoints.begin()+i);
      visible_areas.erase(visible_areas.begin()+i);
      alternatives.erase(alternatives.begin()+i);
    }
  }
  FrontierRoute intents;
  for (const auto &node : viewpoints) intents.push_back(node->center_);
  reverse_pass_ = false;
  selected_pass_route_.clear();
  intermediate_observation_ = false;

  if (viewpoints.empty()) {
    if (corridor_pass_enabled_ && !truncated) {
      if (const auto result = planRisingOpening(current_body_position)) return *result;
      if(clearLastStairTread(current_body_position)) return SUCCEED;
    }
    planner_manager_->graph_visualizer_->vizTour({}, VizColor::RED, "global");
    ed_->global_tour_.clear();
    if (corridor_pass_enabled_ && truncated) {
      expand_pass_search_ = true;
      return FRONTIER_PENDING;
    }
    return frontierStatus(floor_by_floor_ && !truncated);
  }

  if (!floor_by_floor_) frontier_empty_since_=-1.;
  epic_ros::Time t1 = epic_ros::Time::now();
  planner_manager_->topo_graph_->insertNodes(viewpoints, false);
  auto inserted_nodes = viewpoints;
  updateGoalNode();
  epic_ros::Time t2 = epic_ros::Time::now();
  cout << "insert viewpoint to graph time: " << (t2 - t1).toSec() * 1000
       << " ms" << endl;
  float curr_yaw = (float)planner_manager_->local_data_.curr_yaw_;
  vector<double> distance_odom2vp(viewpoints.size(), 0);
  vector<double> initial_turn_cost(viewpoints.size(),0.);
  vector<double> distance_lastgoal2vp(viewpoints.size(), 0);
  double dis2last_goal = 5e3;
  if (!corridor_pass_enabled_ && planner_manager_->lidar_map_interface_->getDisToOcc(
          ed_->next_goal_node_->center_) >
      planner_manager_->parallel_path_finder_->safe_distance_ + 0.1) {
    dis2last_goal = getPathCost(planner_manager_->topo_graph_->odom_node_,
                                Eigen::Vector3d::Zero(), curr_yaw,
                                ed_->next_goal_node_, curr_yaw);
  }
  static double last_frame_value = dis2last_goal;
  bool last_goal_reachable = dis2last_goal < 2e3 && !external_reselect_from_current_;
  external_reselect_from_current_=false;
  // last_goal_reachable = false;

  if (last_goal_reachable && (dis2last_goal < 1.5 * last_frame_value)) {
    last_frame_value = dis2last_goal;
  } else {
    last_goal_reachable = false;
  }

  epic_ros::Time t_start_cvp_1 = epic_ros::Time::now();
  vector<FrontierRoute> routes(viewpoints.size());
  vector<bool> intermediate(viewpoints.size(), false);
  vector<double> margins(viewpoints.size(),-1.);
  omp_set_num_threads(epic_ros::planningWorkerCount());
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (int i = 0; i < viewpoints.size(); ++i) {
    if (last_goal_reachable) {
      distance_lastgoal2vp[i] =
          getPathCost(ed_->next_goal_node_, Eigen::Vector3d::Zero(),
                      viewpoints[i]->yaw_, viewpoints[i], viewpoints[i]->yaw_);
      distance_odom2vp[i] =
          getPathCost(planner_manager_->topo_graph_->odom_node_, selection_velocity, curr_yaw,
                      viewpoints[i], viewpoints[i]->yaw_,
                      use_velocity_direction_cost_ ? &initial_turn_cost[i] : nullptr);

    } else {
      distance_lastgoal2vp[i] =
          getPathCost(planner_manager_->topo_graph_->odom_node_, selection_velocity, curr_yaw,
                      viewpoints[i], viewpoints[i]->yaw_,
                      use_velocity_direction_cost_ ? &initial_turn_cost[i] : nullptr,
                      corridor_pass_enabled_ ? &routes[i] : nullptr);
      distance_odom2vp[i] = distance_lastgoal2vp[i];
    }
  }
  epic_ros::Time t_end_cvp_1 = epic_ros::Time::now();
  if (bm_without_topo) {
    omp_set_num_threads(epic_ros::planningWorkerCount());
    // clang-format off
    #pragma omp parallel for
    // clang-format on
    for (int i = 0; i < viewpoints.size(); ++i) {
      if (last_goal_reachable) {
        distance_lastgoal2vp[i] = getPathCostWithoutTopo(
            ed_->next_goal_node_, Eigen::Vector3d::Zero(), viewpoints[i]->yaw_,
            viewpoints[i], viewpoints[i]->yaw_);
        distance_odom2vp[i] = getPathCostWithoutTopo(
            planner_manager_->topo_graph_->odom_node_, selection_velocity, curr_yaw,
            viewpoints[i], viewpoints[i]->yaw_);

      } else {
        distance_lastgoal2vp[i] = getPathCostWithoutTopo(
            planner_manager_->topo_graph_->odom_node_, selection_velocity, curr_yaw,
            viewpoints[i], viewpoints[i]->yaw_);
        distance_odom2vp[i] = distance_lastgoal2vp[i];
      }
    }
    epic_ros::Time t_end_cvp_2 = epic_ros::Time::now();
    double cost_mat_with_topo = (t_end_cvp_1 - t_start_cvp_1).toSec() * 1000;
    double cost_mat_without_topo = (t_end_cvp_2 - t_end_cvp_1).toSec() * 1000;
    cout << "cost mat topo: " << cost_mat_with_topo << "ms" << endl;
    cout << "cost mat point cloud: " << cost_mat_without_topo << "ms" << endl;
  }

  vector<size_t> reachable;
  vector<CorridorPass::Candidate> candidates;
  const auto connected = [this](const Vector3f &a, const Vector3f &b) {
    return connectedAcrossPass(a, b);
  };
  auto add_reachable = [&](size_t i) {
    reachable.push_back(i);
    if (corridor_pass_enabled_)
      candidates.push_back(pass_.assess(routes[i], visible_areas[i], connected));
  };
  for (size_t i = 0; i < viewpoints.size(); ++i)
    if (distance_odom2vp[i] < 2e3) add_reachable(i);

  auto graph = planner_manager_->topo_graph_;
  auto astar = graph->parallel_bubble_astar_;
  const auto forward = [](const auto &c) { return c.leg == CorridorPass::Leg::Forward; };
  if ((corridor_pass_enabled_ && std::none_of(candidates.begin(), candidates.end(), forward)) ||
      (!corridor_pass_enabled_ && reachable.empty())) {
    const FrontierRoute starts = corridor_pass_enabled_ ? passSearchStarts(current_body_position) :
        FrontierRoute{graph->odom_node_->center_};
    auto attach = [&](size_t i, FrontierRoute route, const Vector3f &goal, float yaw, double area) {
      auto &target = viewpoints[i];
      if ((target->center_ - goal).norm() > .001f) {
        target = std::make_shared<TopoNode>();
        target->center_ = goal;
        target->is_viewpoint_ = true;
        inserted_nodes.push_back(target);
      }
      target->yaw_ = yaw;
      visible_areas[i] = area;
      double length;
      astar->calculatePathCost(route, length);
      const auto odom = graph->odom_node_;
      odom->neighbors_.insert(target);
      target->neighbors_.insert(odom);
      odom->paths_[target] = route;
      target->paths_[odom] = FrontierRoute(route.rbegin(), route.rend());
      odom->weight_[target] = target->weight_[odom] = length;
      distance_odom2vp[i] = distance_lastgoal2vp[i] = length / (ep_->v_max_ / 2.);
      if (directional_frontiers_) initial_turn_cost[i] = frontierContinuationCost(route, preferred_heading_);
      routes[i] = std::move(route);
      add_reachable(i);
    };
    if (!corridor_pass_enabled_) {
      size_t nearest = 0;
      if (directional_frontiers_) {
        auto priority = [&](const TopoNode::Ptr &v) {
          return frontierContinuationCost({graph->odom_node_->center_, v->center_}, preferred_heading_) +
                 (v->center_ - graph->odom_node_->center_).norm();
        };
        nearest = std::min_element(viewpoints.begin(), viewpoints.end(),
            [&](const auto &a, const auto &b) { return priority(a) < priority(b); }) - viewpoints.begin();
      }
      FrontierRoute route;
      if (astar->search(starts.front(), viewpoints[nearest]->center_, route, .08, false) != ParallelBubbleAstar::REACH_END ||
          route.size() < 2 || !astar->collisionCheck_shortenPath(route)) route.clear();
      if (!route.empty()) attach(nearest, std::move(route), viewpoints[nearest]->center_, viewpoints[nearest]->yaw_, visible_areas[nearest]);
    } else {
      for (size_t i = 0; i < viewpoints.size(); ++i) {
        if (distance_odom2vp[i] < 2e3 || starts.empty()) continue;
        const Vector3f goal = viewpoints[i]->center_;
        const char *reason = "no departure";
        auto route = connectPassRoute(current_body_position, starts, goal, .025, &reason);
        if (!route.empty()) {
          attach(i, std::move(route), goal, viewpoints[i]->yaw_, visible_areas[i]);
          continue;
        }
        ROS_INFO("EPIC observation (%.2f %.2f %.2f) connection: %s, goal clearance=%.2f, alternate poses=%zu",
            goal.x(), goal.y(), goal.z(), reason, astar->lidar_map_interface_->getDisToOcc(goal), alternatives[i].size());
        // A cluster is one inspection task. Its other visible poses must not
        // become independent TSP visits or mark the surface as observed.
        for (const auto &view : alternatives[i]) {
          if (!frontier_manager_ptr_->observedFree(view.position)) continue;
          route = connectPassRoute(current_body_position, starts, view.position, .012);
          if (route.empty()) continue;
          ROS_INFO("EPIC alternate observation (%.2f %.2f %.2f) for (%.2f %.2f %.2f)",
              view.position.x(), view.position.y(), view.position.z(), goal.x(), goal.y(), goal.z());
          attach(i, std::move(route), view.position, view.yaw, view.visible_area);
          break;
        }
      }
      // Try a measured intermediate position only after testing actual viewpoints.
      // Prefer the closest missing observation and keep normal pass arbitration.
      if (reachable.empty()) {
        vector<size_t> by_distance(viewpoints.size());
        std::iota(by_distance.begin(), by_distance.end(), 0);
        std::stable_sort(by_distance.begin(), by_distance.end(), [&](size_t a, size_t b) {
          auto priority = [&](size_t index) {
            const auto &goal = intents[index];
            const bool same = floor_by_floor_ && intermediate_intent_ &&
                (goal-*intermediate_intent_).norm() < 2.f*astar->safe_distance_;
            return std::make_pair(!same, (goal-current_body_position).squaredNorm());
          };
          return priority(a) < priority(b);
        });
        for (size_t i : by_distance) {
          const Vector3f goal = viewpoints[i]->center_;
          auto step = floor_by_floor_ ? approachPassGoal(current_body_position,starts,goal,margins[i]) :
              observationStep(current_body_position, starts, goal);
          if (step.empty()) continue;
          const Vector3f landing = step.back(), look = goal - landing;
          const float yaw = look.head<2>().norm() > .05f ? std::atan2(look.y(), look.x()) : viewpoints[i]->yaw_;
          ROS_INFO("EPIC intermediate observation (%.2f %.2f %.2f) toward (%.2f %.2f %.2f)",
              landing.x(), landing.y(), landing.z(), goal.x(), goal.y(), goal.z());
          attach(i, std::move(step), landing, yaw, visible_areas[i]);
          intermediate[i] = true;
          break;
        }
      }
      if (reachable.empty()) ROS_WARN_THROTTLE(1., "EPIC pending: body clearance=%.2f, certified starts=%zu, observation clusters=%zu, no complete or measured intermediate route",
          planner_manager_->lidar_map_interface_->getDisToOcc(current_body_position), starts.size(), viewpoints.size());
    }
  }

  if (corridor_pass_enabled_) {
    const bool has_forward = std::any_of(candidates.begin(), candidates.end(), forward);
    if (!has_forward && truncated) {
      // Dense clusters behind the vehicle can fill the nearest-K shortlist.
      // Check the other live clusters on the next update before turning.
      expand_pass_search_ = true;
      reachable.clear();
    } else {
      const bool only_rear = !candidates.empty() && std::all_of(candidates.begin(), candidates.end(),
          [](const auto &c) { return c.leg == CorridorPass::Leg::Return; });
      const bool wall = only_rear && pass_.wallAhead(current_body_position,
          [&](const Vector3f &p) { return planner_manager_->lidar_map_interface_->getDisToOcc(p); });
      const auto choice = pass_.select(candidates, pass_scan_stamp_, wall);
      vector<size_t> selected;
      for (size_t index : choice.indices) selected.push_back(reachable[index]);
      reachable.swap(selected);
      reverse_pass_ = choice.reverses();
      expand_pass_search_ = !has_forward && !reverse_pass_;
      if (reverse_pass_) {
        const char *reason = choice.reason == CorridorPass::Reason::Wall ? "measured wall" :
            choice.reason == CorridorPass::Reason::Gain ? "rear information gain" : "no reachable forward frontier";
        ROS_INFO("EPIC corridor return: %s, eligible=%zu", reason, reachable.size());
      }
    }
  }
  if (reachable.empty()) {
    graph->removeNodes(inserted_nodes);
    if (corridor_pass_enabled_ && !truncated && candidates.empty()) {
      if (const auto result = planRisingOpening(current_body_position)) return *result;
      if(clearLastStairTread(current_body_position)) return SUCCEED;
    }
    planner_manager_->graph_visualizer_->vizTour({}, VizColor::RED, "global");
    ed_->global_tour_.clear();
    return corridor_pass_enabled_ && (truncated || !candidates.empty()) ? FRONTIER_PENDING : frontierStatus(floor_by_floor_);
  }
  vector<TopoNode::Ptr> viewpoint_reachable;
  vector<double> viewpoint_reachable_distance, viewpoint_reachable_distance2, viewpoint_reachable_turn_cost;
  for (size_t i : reachable) {
    viewpoint_reachable.push_back(viewpoints[i]);
    viewpoint_reachable_distance.push_back(last_goal_reachable ? distance_lastgoal2vp[i] : distance_odom2vp[i]);
    viewpoint_reachable_distance2.push_back(distance_odom2vp[i]);
    viewpoint_reachable_turn_cost.push_back(initial_turn_cost[i]);
  }

  if (viewpoint_reachable.size() == 1) {
    frontier_empty_since_=-1.;
    selected_pass_clearance_=margins[reachable.front()];
    intermediate_observation_ = intermediate[reachable.front()];
    intermediate_intent_ = intermediate_observation_ ? std::make_optional(intents[reachable.front()]) : std::nullopt;
    if (corridor_pass_enabled_) selected_pass_route_ = routes[reachable.front()];
    ed_->global_tour_.clear();
    ed_->global_tour_.emplace_back(pos.cast<float>());
    ed_->global_tour_.emplace_back(viewpoint_reachable.front()->center_);
    planner_manager_->local_data_.end_yaw_ = viewpoint_reachable.front()->yaw_;
    updateGoalNode(viewpoint_reachable.front());
    planner_manager_->topo_graph_->removeNodes(inserted_nodes);
    planner_manager_->graph_visualizer_->vizTour(ed_->global_tour_, VizColor::RED, "global");
    return SUCCEED;
  }

  int dim = viewpoint_reachable.size() + 1;
  Eigen::MatrixXd mat;
  mat.resize(dim, dim);
  mat.setZero();
  for (int i = 1; i < dim; ++i) {
    mat(0, i) = viewpoint_reachable_distance[i - 1];
  }

  omp_set_num_threads(epic_ros::planningWorkerCount());
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (int i = 1; i < dim; i++) {
    for (int j = i + 1; j < dim; j++) {
      mat(i, j) = mat(j, i) = getPathCost(
          viewpoint_reachable[i - 1], Eigen::Vector3d(0, 0, 0),
          viewpoint_reachable[i - 1]->yaw_, viewpoint_reachable[j - 1],
          viewpoint_reachable[j - 1]->yaw_);
    }
  }
  // trick 往远走
  for (int i = 1; i < dim; ++i) {
    mat(i, 0) = 2e3 - viewpoint_reachable_distance2[i - 1] * 0.2;
  }
  for (int i = 0; i < dim; ++i) {
    for (int j = 1; j < dim; ++j) {
      for (int k = 1; k < dim; ++k) {
        if (mat(i, j) > mat(i, k) + mat(k, j)) {
          mat(i, j) = mat(i, k) + mat(k, j) + 1e-2;
        }
      }
    }
  }
  if(use_velocity_direction_cost_)for(int i=1;i<dim;++i)
    mat(0,i)+=viewpoint_reachable_turn_cost[i-1];
  vector<int> indices;
  indices.reserve(dim);
  epic_ros::Time start_tsp = epic_ros::Time::now();
  cout << "calculate tsp cost matrix cost " << (start_tsp - t2).toSec() * 1000
       << "ms" << endl;
  solveLHK(mat, indices);
  epic_ros::Time end_tsp = epic_ros::Time::now();
  cout << "lkh solver cost: " << (end_tsp - start_tsp).toSec() * 1000 << "ms"
       << endl;
  // if ((end_tsp - start_tsp).toSec() * 1000 > 100)
  //   exit(0);
  ed_->global_tour_.clear();
  for (auto &i : indices) {
    if (i == 0) {
      ed_->global_tour_.push_back(
          planner_manager_->topo_graph_->odom_node_->center_);
    } else {
      ed_->global_tour_.emplace_back(viewpoint_reachable[i - 1]->center_);
    }
  }
  if (!last_goal_reachable)
    last_frame_value = viewpoint_reachable_distance[indices[1] - 1];

  epic_ros::Time end = epic_ros::Time::now();
  // Transfer the selected viewpoint's certified native connections before its
  // temporary node is removed. A bounded reconnect may not find the same edge.
  const auto selected_viewpoint=viewpoint_reachable[indices[1]-1];
  intermediate_observation_ = intermediate[reachable[indices[1]-1]];
  selected_pass_clearance_=margins[reachable[indices[1]-1]];
  frontier_empty_since_=-1.;
  intermediate_intent_ = intermediate_observation_ ?
      std::make_optional(intents[reachable[indices[1]-1]]) : std::nullopt;
  if (corridor_pass_enabled_) selected_pass_route_ = routes[reachable[indices[1]-1]];
  updateGoalNode(selected_viewpoint);
  planner_manager_->topo_graph_->removeNodes(inserted_nodes);
  planner_manager_->graph_visualizer_->vizTour(ed_->global_tour_, VizColor::RED,
                                               "global");

  planner_manager_->local_data_.end_yaw_ =
      viewpoint_reachable[indices[1] - 1]->yaw_;
  return SUCCEED;
}

void FastExplorationManager::solveLHK(Eigen::MatrixXd &cost_mat,
                                      vector<int> &indices) {
  // Solve linear homogeneous kernel
  // Write params and cost matrix to problem file
  int dimension = cost_mat.rows();
  if (dimension < 3)
    return;
  ofstream prob_file(ep_->tsp_dir_ + "/single.tsp");
  // Problem specification part, follow the format of TSPLIB

  string prob_spec =
      "NAME : single\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
      "\nEDGE_WEIGHT_TYPE : "
      "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";

  // string prob_spec = "NAME : single\nTYPE : TSP\nDIMENSION : " +
  // to_string(dimension) +
  //     "\nEDGE_WEIGHT_TYPE : "
  //     "EXPLICIT\nEDGE_WEIGHT_FORMAT : LOWER_ROW\nEDGE_WEIGHT_SECTION\n";

  prob_file << prob_spec;
  // prob_file << "TYPE : TSP\n";
  // prob_file << "EDGE_WEIGHT_FORMAT : LOWER_ROW\n";
  // Problem data part
  const int scale = 100;

  // Use Asymmetric TSP
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = cost_mat(i, j) * scale;
      prob_file << int_cost << " ";
    }
    prob_file << "\n";
  }

  prob_file << "EOF";
  prob_file.close();

  // Call LKH TSP solver
  solveTSPLKH((ep_->tsp_dir_ + "/single.par").c_str());

  // Read optimal tour from the tour section of result file
  ifstream res_file(ep_->tsp_dir_ + "/single.txt");
  string res;
  while (getline(res_file, res)) {
    // Go to tour section
    if (res.compare("TOUR_SECTION") == 0)
      break;
  }

  // Read path for ATSP formulation
  while (getline(res_file, res)) {

    // Read indices of frontiers in optimal tour
    int id = stoi(res);
    if (id == -1)
      break;
    indices.push_back(id - 1); // Idx of solver-2 == Idx of frontier
  }

  res_file.close();
}

bool FastExplorationManager::repairHeightExcursions(FrontierRoute &path, const Vector3f &body, float margin) const {
  if (!corridor_pass_enabled_ || path.size()<2 || !frontier_manager_ptr_) return false;
  const auto astar=planner_manager_->topo_graph_->parallel_bubble_astar_;
  const Vector3f goal=path.back();
  const float low=std::min(body.z(),goal.z())-margin, high=std::max(body.z(),goal.z())+margin;
  const auto in_band=[&](const Vector3f &p){return p.allFinite() && p.z()>=low && p.z()<=high;};
  if (!in_band(goal)) return false;
  const auto certified=[&](const FrontierRoute &route) {
    if (!observedPass(route)) return false;
    for (size_t i=1;i<route.size();++i) {
      FrontierRoute edge{route[i-1],route[i]};
      if (!astar->collisionCheck_shortenPath(edge)) return false;
    }
    return true;
  };
  const Vector3f box_min(-std::numeric_limits<float>::max(),-std::numeric_limits<float>::max(),low);
  const Vector3f box_max(std::numeric_limits<float>::max(),std::numeric_limits<float>::max(),high);
  FrontierRoute repaired{body};
  for (size_t i=0;i<path.size();) {
    if (in_band(path[i])) {
      if ((repaired.back()-path[i]).norm()>.005f) repaired.push_back(path[i]);
      ++i;
      continue;
    }
    FrontierRoute patch{repaired.back()};
    while (i<path.size() && !in_band(path[i])) {
      if (!path[i].allFinite()) return false;
      Vector3f p=path[i++];
      p.z()=std::clamp(p.z(),low,high);
      if ((patch.back()-p).norm()>.005f) patch.push_back(p);
    }
    if (i==path.size()) return false;
    const Vector3f next=path[i++];
    if ((patch.front()-next).norm()<.005f) continue;
    patch.push_back(next);
    if (!certified(patch)) {
      patch.clear();
      if (astar->search(repaired.back(),next,patch,.035,false,false,box_min,box_max)!=ParallelBubbleAstar::REACH_END ||
          patch.size()<2 || !certified(patch) ||
          !withinEndpointHeights(patch,body,goal,margin)) return false;
    }
    repaired.insert(repaired.end(),patch.begin()+1,patch.end());
  }
  if(repaired.size()<2 || !withinEndpointHeights(repaired,body,goal,margin)) return false;
  path.swap(repaired);
  ROS_INFO("EPIC reconnected height-limited section between observed anchors");
  return true;
}

bool FastExplorationManager::reanchorLocalPath(vector<Eigen::Vector3f> &path,
    const Eigen::Vector3f &boundary,const Eigen::Vector3f &velocity) {
  if(path.empty() || !boundary.allFinite() || !velocity.allFinite())return false;
  auto astar=planner_manager_->topo_graph_->parallel_bubble_astar_;
  auto lio=planner_manager_->lidar_map_interface_;
  const auto goal=path.back();
  epic_ros::NodeHandle height_nh;double height_margin;
  height_nh.param("bubble_astar/endpoint_height_margin",height_margin,-1.);
  if(height_margin>=0.) {
    if(corridor_pass_enabled_ && !withinEndpointHeights(path,boundary,goal,height_margin)) {
      // A cached departure behind the current body is no longer flown. Try
      // the same connected suffix before repairing an actual height excursion.
      double progress=0.;
      auto suffix=remainingFrontierRoute(path,boundary,path.front(),progress);
      if(progress>.15 && withinEndpointHeights(suffix,boundary,goal,height_margin) &&
         certifyPassGuide(suffix,nullptr,-1.,true)) {
        path.swap(suffix);
        ROS_INFO("EPIC trimmed past travel below the current guide height band");
        return true;
      }
    }
    if(!withinEndpointHeights(path, boundary, goal, height_margin) &&
       !repairHeightExcursions(path,boundary,float(height_margin))) {
      vector<Eigen::Vector3f> repaired;
      if(astar->search(boundary,goal,repaired,.08,false)!=ParallelBubbleAstar::REACH_END) {
        ROS_WARN("Reject cached guide outside endpoint height band; waiting for native bounded reconnect");
        path.clear();return false;
      }
      path.swap(repaired);
      ROS_INFO("Repaired native cached guide within measured endpoint height band");
    }
  }
  if(!goal.allFinite() || goal.z()<astar->flight_min_z_-1.e-5 ||
      goal.z()>astar->flight_max_z_+1.e-5 ||
      lio->getDisToOcc(goal)<=astar->safe_distance_+.011) {
    path.clear();return false;
  }
  auto attach=[&](size_t join,vector<Eigen::Vector3f> connector) {
    connector.insert(connector.end(),path.begin()+join+1,path.end());
    path.swap(connector);
  };
  Eigen::Vector3f direction=velocity;
  if(direction.norm()<.1f)direction=goal-boundary;
  if(direction.norm()>.1f) {
    direction.normalize();
    // Prefer the latest safe point near the moving boundary, discarding OLD
    // departure loops rather than picking their earliest nearby occurrence.
    for(size_t i=path.size();i-->0;) {
      const Eigen::Vector3f delta=path[i]-boundary;
      if(delta.norm()>3.f || delta.dot(direction)<.05f)continue;
      vector<Eigen::Vector3f> connector;
      if(astar->search(boundary,path[i],connector,.002,false,true)==ParallelBubbleAstar::REACH_END) {
        attach(i,std::move(connector));return true;
      }
    }
  }
  // Repair only the departure into the existing native graph guide. Rebuilding
  // the distant global goal here turns a local reconnect into a warehouse-wide
  // grid search. The suffix is guidance; complete candidate checks still apply.
  int attempts=0;
  for(size_t i=path.size();i-->0 && attempts<2;) {
    const float distance=(path[i]-boundary).norm();
    if(distance<.05f || distance>3.f ||
        lio->getDisToOcc(path[i])<=astar->safe_distance_+.011)continue;
    ++attempts;vector<Eigen::Vector3f> connector;
    if(astar->search(boundary,path[i],connector,.03,false)==ParallelBubbleAstar::REACH_END) {
      attach(i,std::move(connector));
      ROS_INFO("Native A* repaired local departure; cached global guide/goal retained");return true;
    }
  }
  const auto &config=planner_manager_->gcopter_config_;
  if(config && config->startCorridorRecovery) {
    const float margin=config->dilateRadiusHard+config->startRecoveryMargin;
    vector<size_t> candidates;
    for(size_t i=0;i<path.size();++i)
      if((path[i]-boundary).norm()<=3.f &&
          lio->getDisToOcc(path[i])>astar->safe_distance_+.011)candidates.push_back(i);
    std::stable_sort(candidates.begin(),candidates.end(),[&](size_t a,size_t b) {
      const float da=(path[a]-boundary).squaredNorm(),db=(path[b]-boundary).squaredNorm();
      return da==db ? a>b : da<db;
    });
    for(size_t i=0;i<std::min(size_t(8),candidates.size());++i) {
      const size_t join=candidates[i];const auto p=path[join];
      Eigen::Vector3f current=boundary;bool certified=false;
      for(int step=0;step<64;++step) {
        const float radius=lio->getDisToOcc(current)-margin;
        const Eigen::Vector3f delta=p-current;const float distance=delta.norm();
        if(!std::isfinite(radius) || radius<=1.e-4f)break;
        if(distance<=radius){certified=true;break;}
        current+=delta*(.9f*radius/distance);
      }
      if(!certified)continue;
      attach(join,{boundary,p});
      ROS_INFO("Local departure recovered with certified %.3fm clearance; cached native guide retained",margin);
      return true;
    }
  }
  // Sparse cached guides may have no nearby point. Preserve the bounded full
  // native fallback, never prepend an unchecked leg to an old departure.
  vector<Eigen::Vector3f> rebuilt;
  const int status=astar->search(boundary,goal,rebuilt,.08,false);
  if(status!=ParallelBubbleAstar::REACH_END || rebuilt.size()<2 ||
      (rebuilt.back()-goal).norm()>.2f) {
    ROS_WARN("Cannot safely reanchor cached local guide; native A* status %d, start=(%.2f %.2f %.2f) clearance=%.3f, goal=(%.2f %.2f %.2f) clearance=%.3f",
        status,boundary.x(),boundary.y(),boundary.z(),lio->getDisToOcc(boundary),
        goal.x(),goal.y(),goal.z(),lio->getDisToOcc(goal));
    path.clear();return false;
  }
  path.swap(rebuilt);return true;
}

void FastExplorationManager::commitObservedOutlet(const Vector3f &body,const Vector3f &goal,float clearance) {
  if(floor_trace_.surveyedLevel(body,clearance) || floor_trace_.inAscentFootprint(goal,clearance) ||
     std::abs(goal.z()-body.z())>1.8f*clearance || (goal-body).head<2>().norm()<1.7f*clearance) return;
  if(deferred_outlet_ && deferred_outlet_->second>pass_scan_stamp_ &&
     (goal-deferred_outlet_->first).norm()<2.5f*clearance) return;
  if(!outlet_commitment_ || (goal-outlet_commitment_->goal).norm()>2.f*clearance)
    outlet_commitment_=OutletCommitment{goal,(goal-body).norm(),pass_scan_stamp_};
  landing_intent_.reset();
}

FrontierRoute FastExplorationManager::measuredRises(const Vector3f &tip,float clearance) const {
  for(const auto &q:rising_queries_)
    if(q.clearance==clearance && q.tip==tip) return q.openings;
  auto found=frontier_manager_ptr_->risingOpenings(tip,clearance);
  if(rising_queries_.size()<16) rising_queries_.push_back({tip,clearance,found});
  return found;
}

void FastExplorationManager::observePass(const Vector3f &position, double scan_stamp) {
  rising_queries_.clear();
  if (!corridor_pass_enabled_ || scan_stamp <= pass_scan_stamp_) return;
  pass_scan_stamp_ = scan_stamp;
  blocked_points_.erase(std::remove_if(blocked_points_.begin(), blocked_points_.end(),
      [&](const auto &entry) { return entry.second <= scan_stamp; }), blocked_points_.end());
  pass_.observe(position);
  if (!floor_by_floor_) return;
  const float safe = planner_manager_->topo_graph_->parallel_bubble_astar_->safe_distance_;
  const Vector3f sensor=planner_manager_->lidar_map_interface_->ld_->lidar_pose_;
  floor_trace_.observe(sensor,safe);
  if(scan_stamp-floor_support_stamp_>.8) {
    floor_support_stamp_=scan_stamp;
    const auto map=planner_manager_->lidar_map_interface_;
    floor_model_supported_=hasSupportingFloor(sensor,safe,
        [&](const Vector3f &p){return frontier_manager_ptr_->observedFree(p);},
        [&](const Vector3f &p){return map->IsInBox(p)?float(map->getDisToOcc(p)):std::numeric_limits<float>::infinity();});
  }
  if(outlet_commitment_) {
    auto &outlet=*outlet_commitment_;
    const float distance=(position-outlet.goal).norm();
    if(distance+.25f*safe<outlet.distance) {outlet.distance=distance;outlet.progress=scan_stamp;}
    if(distance<1.3f*safe || floor_trace_.surveyedLevel(position,safe) ||
       std::abs(position.z()-outlet.goal.z())>2.3f*safe) outlet_commitment_.reset();
    else if(scan_stamp-outlet.progress>6.) {
      deferred_outlet_=std::make_pair(outlet.goal,scan_stamp+8.);
      outlet_commitment_.reset();
    }
  }
  if (end_turn_) {
    const Vector3f offset=position-end_turn_->tip;
    if (offset.norm()<1.25f*safe) end_turn_->reached=true;
    if (end_turn_->reached && offset.dot(end_turn_->forward)<-2.0f*safe &&
        planner_manager_->local_data_.curr_vel_.cast<float>().dot(end_turn_->forward)<-.2f) end_turn_.reset();
  }
  if (last_branch_scan_position_ && (sensor-*last_branch_scan_position_).norm()<2.f*safe) return;
  last_branch_scan_position_=sensor;
  // A long guide may cross a junction between planning callbacks. Preserve
  // side passages as soon as their actual rays arrive, even while the current
  // guide remains valid and owns the direction of travel.
  const auto map=planner_manager_->lidar_map_interface_;
  const float step=std::min(.4f,safe*.5f);
  for (int i=0;i<24;++i) {
    const double angle=i*M_PI/12.;
    const Vector3f axis(std::cos(angle),std::sin(angle),0.);
    Vector3f last=sensor;
    for (float range=step;range<=map->lp_->max_ray_length_;range+=step) {
      const Vector3f p=sensor+range*axis;
      if (!frontier_manager_ptr_->observedFree(p) || map->getDisToOcc(p)<safe+.02f) break;
      last=p;
    }
    if ((last-sensor).norm()>4.5f*safe && !frontier_manager_ptr_->recentlySurveyed(last,3.6f*safe,1.5f*safe,.4f*safe))
      rememberPassage(last);
  }
}

void FastExplorationManager::rememberPassage(const Vector3f &goal) {
  const float safe=planner_manager_->topo_graph_->parallel_bubble_astar_->safe_distance_;
  if (frontier_manager_ptr_->inspectedPassage(goal,safe)) return;
  if (std::none_of(pending_passages_.begin(),pending_passages_.end(),[&](const auto &branch){
    return (branch.goal-goal).norm()<2.5f*safe;
  })) pending_passages_.push_back({goal,0.});
}

bool FastExplorationManager::connectedAcrossPass(const Vector3f &a, const Vector3f &b) const {
  // The bridge tests whether two lanes occupy the same open passage. It is
  // never emitted as a flight segment; native flight clearance still applies.
  auto astar = planner_manager_->topo_graph_->parallel_bubble_astar_;
  FrontierRoute bridge{a, b};
  return astar->collisionCheck_shortenPath(bridge, .25 * astar->safe_distance_);
}

bool FastExplorationManager::observedPass(const FrontierRoute &path) const {
  if (path.size() < 2) return false;
  const auto astar = planner_manager_->topo_graph_->parallel_bubble_astar_;
  const float step = astar->resolution_;
  for (size_t i = 1; i < path.size(); ++i) {
    const Vector3f delta = path[i] - path[i - 1];
    const int count = std::max(1, int(std::ceil(delta.norm() / step)));
    for (int j = 1; j <= count; ++j) {
      const Vector3f p = path[i - 1] + delta * (float(j) / count);
      if (!frontier_manager_ptr_->observedFree(p) || !astar->lidar_map_interface_->IsInBox(p)) return false;
      for (const auto &entry : blocked_points_) {
        const float limit = std::min(float(astar->safe_distance_ * .7), (path.front() - entry.first).norm());
        if ((p - entry.first).norm() + .001f < limit) return false;
      }
    }
  }
  return true;
}

void FastExplorationManager::avoidBlockedPoint(const Vector3f &point, bool stalled) {
  end_turn_.reset();
  // A fresh scan already constrains every replacement segment. Inflating
  // the rejected *centreline* again can close the only safe lane beside it.
  // Reserve this additional exclusion for an actually stalled departure.
  if (floor_by_floor_ && !stalled) return;
  if (blocked_points_.size() == 16) blocked_points_.erase(blocked_points_.begin());
  blocked_points_.emplace_back(point, pass_scan_stamp_ + 15.);
}

std::optional<Vector3f> FastExplorationManager::blockedAhead(const FrontierRoute &remaining, double clearance) const {
  const auto astar = planner_manager_->topo_graph_->parallel_bubble_astar_;
  const auto &config = planner_manager_->gcopter_config_;
  if (remaining.empty()) return std::nullopt;
  const double normal = clearance < 0. ? astar->safe_distance_ : clearance;
  const auto map = astar->lidar_map_interface_;
  const double recovery = config->dilateRadiusHard + config->startRecoveryMargin;
  bool recovering = config->startCorridorRecovery && map->getDisToOcc(remaining.front()) <= normal + .011;
  double travelled = 0.;
  for (size_t i = 1; i < remaining.size(); ++i) {
    const Vector3f delta = remaining[i] - remaining[i - 1];
    const float length = delta.norm();
    const int count = std::max(1, int(std::ceil(length / astar->resolution_)));
    for (int j = 1; j <= count; ++j) {
      travelled += length / count;
      if (travelled > 6. * normal) return std::nullopt;
      const Vector3f p = remaining[i - 1] + delta * (float(j) / count);
      const double distance = map->getDisToOcc(p);
      if (distance >= normal + .011 || travelled > 2. * normal) recovering = false;
      if (distance <= (recovering ? recovery : normal) || !frontier_manager_ptr_->observedFree(p)) return p;
    }
  }
  return std::nullopt;
}

FrontierRoute FastExplorationManager::passSearchStarts(const Vector3f &position,double margin) const {
  const auto map = planner_manager_->lidar_map_interface_;
  const auto astar = planner_manager_->topo_graph_->parallel_bubble_astar_;
  const auto &config = planner_manager_->gcopter_config_;
  const double normal=margin<0. ? double(astar->safe_distance_) : std::min(margin,double(astar->safe_distance_));
  const double current = map->getDisToOcc(position);
  if (current > normal + .011) return {position};
  const double recovery = config->dilateRadiusHard + config->startRecoveryMargin;
  if (!config->startCorridorRecovery || current <= recovery + .01) return {};
  FrontierRoute directions;
  PointVector nearest;
  vector<float> distances;
  map->KNN(PointType(position.x(), position.y(), position.z()), 1, nearest, distances);
  if (!nearest.empty()) {
    const Vector3f away = position - nearest.front().getVector3fMap();
    if (away.norm() > .001f) directions.emplace_back(away.normalized());
  }
  for (int elevation : {-60, -30, 0, 30, 60}) {
    const float pitch = elevation * float(M_PI / 180.);
    for (int i = 0; i < 16; ++i) {
      const float yaw = i * float(M_PI / 8.);
      directions.emplace_back(std::cos(pitch)*std::cos(yaw), std::cos(pitch)*std::sin(yaw), std::sin(pitch));
    }
  }
  directions.push_back(Vector3f::UnitZ());
  directions.push_back(-Vector3f::UnitZ());
  vector<std::pair<double, Vector3f>> scored;
  for (const auto &direction : directions) {
    for (double step = .25 * normal; step <= 1.5 * normal + 1.e-5; step += .25 * normal) {
      const Vector3f p = position + float(step) * direction;
      const double clearance = map->getDisToOcc(p);
      if (clearance <= normal + .05 || p.z() < astar->flight_min_z_ || p.z() > astar->flight_max_z_) continue;
      if(margin>=0.) {
        bool away=true;
        for(float d=.5f*astar->resolution_;d<step && away;d+=.5f*astar->resolution_) {
          const Vector3f q=position+d*direction;
          away=map->getDisToOcc(q)+.012f>=current;
        }
        if(!away) continue;
      }
      FrontierRoute edge{position, p};
      if (observedPass(edge) && astar->collisionCheck_shortenPath(edge, recovery))
        scored.emplace_back(clearance - .12 * step, p);
    }
  }
  std::stable_sort(scored.begin(), scored.end(), [](const auto &a, const auto &b){return a.first > b.first;});
  FrontierRoute result;
  for (const auto &entry : scored) {
    if (std::none_of(result.begin(), result.end(), [&](const Vector3f &p) {
          return (p - entry.second).norm() < .6 * normal;
        })) result.push_back(entry.second);
    if (result.size() == 4) break;
  }
  return result;
}

FrontierRoute FastExplorationManager::connectPassRoute(const Vector3f &body, const FrontierRoute &starts,
                                                       const Vector3f &goal, double budget, const char **reason) const {
  const auto astar = planner_manager_->topo_graph_->parallel_bubble_astar_;
  if (reason) *reason = "no certified departure";
  const double end_room=std::max(double(astar->safe_distance_),double(planner_manager_->topo_graph_->bubble_min_radius_));
  if (astar->lidar_map_interface_->getDisToOcc(goal) <= end_room + .011) {
    if (reason) *reason = "goal clearance";
    return {};
  }
  for (const auto &start : starts) {
    FrontierRoute route;
    const int result = astar->search(start, goal, route, budget / starts.size(), false);
    if (result != ParallelBubbleAstar::REACH_END || route.size() < 2) {
      if (reason) *reason = result == ParallelBubbleAstar::TIME_OUT ? "A* timeout" : "no native connection";
      continue;
    }
    if ((start - body).norm() > .01f) route.insert(route.begin(), body);
    if (certifyPassGuide(route, reason)) return route;
  }
  return {};
}

FrontierRoute FastExplorationManager::measuredClimb(const Vector3f &body,const Vector3f &heading,
    const FrontierRoute &openings,double &clearance) const {
  const auto graph=planner_manager_->topo_graph_;
  const auto astar=graph->parallel_bubble_astar_;
  const auto map=astar->lidar_map_interface_;
  const float safe=astar->safe_distance_,minimum=narrowPassClearance(true);
  FrontierRoute local;
  for(const auto &p:openings)
    if((p-body).head<2>().norm()<12.f*safe && p.z()>body.z()+.75f*safe) local.push_back(p);
  if(local.empty()) return {};
  const float cell=.5f*astar->resolution_;
  std::unordered_map<Eigen::Vector3i,float,v3i_hash> distances;
  auto room=[&](const Vector3f &p) -> float {
    if(!map->IsInBox(p) || !frontier_manager_ptr_->observedFree(p) || p.z()<astar->flight_min_z_ ||
       p.z()>astar->flight_max_z_) return 0.f;
    const Eigen::Vector3i k=(p/cell).array().floor().cast<int>();
    const Vector3f center=(k.cast<float>().array()+.5f)*cell;
    auto [it,fresh]=distances.try_emplace(k,0.f);
    if(fresh) it->second=map->getDisToOcc(center);
    // Subtract the centre offset to get a conservative pointwise lower bound.
    return it->second-(p-center).norm();
  };
  auto climb=observedClimb(body,heading,local,safe,minimum,room,[&](const Vector3f &p) {
    return !frontier_manager_ptr_->recentlySurveyed(p,1.5f*safe,.7f*safe,.4f*safe);
  });
  if(climb.size()<2 || map->getDisToOcc(climb.back())<graph->bubble_min_radius_+.05f) return {};
  for(float margin:{safe,narrowPassClearance(),minimum}) {
    auto route=climb;
    if(!certifyPassGuide(route,nullptr,margin)) continue;
    clearance=margin;
    ROS_INFO("EPIC joined successive measured slope segments, horizontal %.2fm, rise %.2fm",
             double((route.back()-body).head<2>().norm()),double(route.back().z()-body.z()));
    return route;
  }
  return {};
}

FrontierRoute FastExplorationManager::chooseMeasuredAscent(const Vector3f &body,const Vector3f &velocity,
                                                           const FrontierRoute &openings,double &margin) const {
  auto previous=measuredPreviousAscent(body,margin);
  if(previous.empty()) {
    auto local=measuredClimb(body,velocity,openings,margin);
    if(!local.empty() || openings.empty()) return local;
    const auto astar=planner_manager_->topo_graph_->parallel_bubble_astar_;
    const float tier=narrowPassClearance(true);
    if(astar->lidar_map_interface_->getDisToOcc(body)>tier+.012f) return {};
    auto starts=passSearchStarts(body,tier);
    std::stable_sort(starts.begin(),starts.end(),[&](const Vector3f &a,const Vector3f &b) {
      return (a-body).normalized().dot(velocity)>(b-body).normalized().dot(velocity);
    });
    for(const auto &start:starts) {
      if((start-body).norm()<.02f) continue;
      double recovered=-1.;
      auto route=measuredClimb(start,velocity,openings,recovered);
      if(route.size()<2) continue;
      route.insert(route.begin(),body);
      const float certified=std::min(tier,recovered<0. ? float(astar->safe_distance_) : float(recovered));
      if(!certifyPassGuide(route,nullptr,certified,true)) continue;
      ROS_INFO("EPIC joined the measured ascent from a verified clearance-recovery departure");
      margin=certified;
      return route;
    }
    return {};
  }
  if(openings.empty()) return previous;
  const float safe=planner_manager_->topo_graph_->parallel_bubble_astar_->safe_distance_;
  float length=0.f;
  for(size_t i=1;i<previous.size();++i) length+=(previous[i]-previous[i-1]).norm();
  const Vector3f direct=previous.back()-body,departure=previous[1]-body;
  const bool aligned=velocity.norm()<.55f || velocity.dot(departure)>.50f*velocity.norm()*departure.norm();
  if(aligned && length<1.45f*direct.norm() && direct.z()>1.6f*safe) return previous;
  double changed_margin=-1.;
  auto alternate=measuredClimb(body,velocity,openings,changed_margin);
  if(alternate.size()<2 || alternate.back().z()-body.z()<.75f*direct.z()) return previous;
  const auto &limits=planner_manager_->gcopter_config_;
  auto rate=[&](const FrontierRoute &route,double clearance) {
    const double duration=estimateTransitTime(route,velocity,.8f*safe,limits->maxVelMag,limits->maxAccMag,limits->maxJerkMag);
    return (route.back().z()-body.z())/(duration+.7)+.07*std::min(double(safe),clearance<0.?double(safe):clearance);
  };
  if(rate(alternate,changed_margin)>1.08*rate(previous,margin)) {
    margin=changed_margin;
    ROS_INFO("EPIC selected a faster measured ascent around the current stair bend");
    return alternate;
  }
  return previous;
}

FrontierRoute FastExplorationManager::measuredPreviousAscent(const Vector3f &body,double &clearance) const {
  const auto graph=planner_manager_->topo_graph_;
  const auto astar=graph->parallel_bubble_astar_;
  const auto map=astar->lidar_map_interface_;
  const float safe=astar->safe_distance_;
  const auto guides=floor_trace_.ascentCandidates(body,planner_manager_->local_data_.curr_vel_.cast<float>(),
                                                  16.f*safe,safe);
  FrontierRoute best;
  float best_score=0.f;
  for(const auto &guide:guides) {
    FrontierRoute centers{body};
    std::vector<float> arc{0.f};
    for(size_t i=1;i<guide.size();++i) {
      Vector3f tangent=guide[std::min(i+1,guide.size()-1)]-guide[i-1];
      tangent.z()=0.f;
      const Vector3f side=tangent.norm()>.05f ? Vector3f(-tangent.y(),tangent.x(),0.f).normalized() : Vector3f::UnitX();
      Vector3f chosen=guide[i];
      float score=-std::numeric_limits<float>::infinity();
      for(int lateral=-2;lateral<=2;++lateral) for(int height=-1;height<=1;++height) {
        const Vector3f offset=(.4f*safe*lateral)*side+Vector3f(0,0,.35f*safe*height);
        const Vector3f p=guide[i]+offset;
        if(!map->IsInBox(p) || !frontier_manager_ptr_->observedFree(p) ||
            p.z()<astar->flight_min_z_ || p.z()>astar->flight_max_z_) continue;
        const float room=map->getDisToOcc(p);
        const float rank=std::min(room,1.7f*safe)-.3f*offset.norm();
        if(rank>score){score=rank;chosen=p;}
      }
      if(!std::isfinite(score)) break;
      centers.push_back(chosen);
      arc.push_back(arc.back()+(guide[i]-guide[i-1]).norm());
    }
    for(float margin:{safe,narrowPassClearance(),narrowPassClearance(true)}) {
      FrontierRoute actual{body};
      size_t end=0;
      for(size_t i=0;i+1<centers.size();) {
        size_t furthest=i;
        for(size_t j=i+1;j<centers.size() && arc[j]-arc[i]<=7.f*safe;++j) {
          if((centers[j]-actual.back()).norm()<.1f*safe) continue;
          FrontierRoute edge{actual.back(),centers[j]};
          if(observedPass(edge) && astar->collisionCheck_shortenPath(edge,margin)) furthest=j;
        }
        if(furthest==i) break;
        i=furthest;
        actual.push_back(centers[i]);
        if(map->getDisToOcc(centers[i])>graph->bubble_min_radius_+.04f &&
            centers[i].z()>body.z()+.55f*safe) end=actual.size();
      }
      if(!end) continue;
      actual.resize(end);
      const auto &tip=actual.back();
      if(frontier_manager_ptr_->recentlySurveyed(tip,1.4f*safe,.65f*safe,.4f*safe) ||
          !certifyPassGuide(actual,nullptr,margin)) continue;
      const float score=2.f*(actual.back().z()-body.z())+.25f*(actual.back()-body).norm()+margin;
      if(score>best_score+.05f*safe) {best_score=score;best=std::move(actual);clearance=margin;}
    }
  }
  const float achieved=best.empty() ? 0.f : best.back().z()-body.z();
  float arc=0.f;
  for(size_t i=1;i<best.size();++i) arc+=(best[i]-best[i-1]).norm();
  if(!guides.empty() && (achieved<1.65f*safe || arc<4.5f*safe)) {
    // Selecting each station's point of maximum clearance independently can
    // alternate between both sides of the handrail. Look for one connected
    // corridor through the observed cross sections before sending a short tip.
    auto room=[&](const Vector3f &p) {
      if(!map->IsInBox(p) || p.z()<astar->flight_min_z_ || p.z()>astar->flight_max_z_ ||
         !frontier_manager_ptr_->observedFree(p)) return 0.f;
      return float(map->getDisToOcc(p));
    };
    struct Tip {size_t guide,index;Vector3f point;float score;};
    std::vector<Tip> tips;
    const float required=graph->bubble_min_radius_+.055f;
    for(size_t g=0;g<guides.size();++g) {
      const auto &guide=guides[g];
      for(size_t i=2;i<guide.size();++i) {
        if(guide[i].z()<body.z()+achieved+.55f*safe) continue;
        Vector3f tangent=guide[i]-guide[i-1];
        Vector3f side(-tangent.y(),tangent.x(),0.f);
        if(side.norm()>.05f) side.normalize();else side=Vector3f::UnitX();
        float best_room=-1.f;Vector3f tip=guide[i];
        for(float y:{0.f,.5f,-.5f}) for(float z:{0.f,.38f,-.38f}) {
          const Vector3f offset=safe*(y*side+z*Vector3f::UnitZ()),p=guide[i]+offset;
          const float free=room(p);
          const float score=std::min(free,1.6f*safe)-.26f*offset.norm();
          if(free>required && score>best_room) {best_room=score;tip=p;}
        }
        if(best_room<0.f || tip.z()<body.z()+achieved+.55f*safe ||
           frontier_manager_ptr_->recentlySurveyed(tip,1.4f*safe,.65f*safe,.4f*safe)) continue;
        tips.push_back({g,i,tip,2.f*(tip.z()-body.z())+.22f*(tip-body).norm()+.25f*best_room});
      }
    }
    std::stable_sort(tips.begin(),tips.end(),[](const Tip &a,const Tip &b){return a.score>b.score;});
    std::vector<Vector3f> attempted;
    for(const auto &tip:tips) {
      if(std::any_of(attempted.begin(),attempted.end(),[&](const Vector3f &p){return (p-tip.point).norm()<1.55f*safe;})) continue;
      attempted.push_back(tip.point);
      const auto &guide=guides[tip.guide];
      bool reached=false;
      for(float margin:{safe,narrowPassClearance(),narrowPassClearance(true)}) {
        if(room(tip.point)<margin+.015f) continue;
        auto starts=passSearchStarts(body,margin);
        auto clear=[&](const Vector3f &a,const Vector3f &b) {
          FrontierRoute segment{a,b};return observedPass(segment) && astar->collisionCheck_shortenPath(segment,margin);
        };
        for(size_t s=0;s<std::min<size_t>(2,starts.size());++s) {
          FrontierRoute outline(guide.begin(),guide.begin()+tip.index+1);
          outline.front()=starts[s];outline.back()=tip.point;
          FrontierRoute lane;
          if(clear(starts[s],tip.point)) lane={starts[s],tip.point};
          else if((starts[s]-body).norm()<.02f || (starts[s]-outline[1]).norm()<2.5f*safe)
            lane=repairObservedBend(outline,margin,1.05f*safe,room,clear,20.f*safe);
          if(lane.size()<2) continue;
          if((starts[s]-body).norm()>.02f) lane.insert(lane.begin(),body);
          if(!certifyPassGuide(lane,nullptr,margin,true)) continue;
          best.swap(lane);clearance=margin;reached=true;break;
        }
        if(reached) break;
      }
      if(reached) {
        ROS_INFO("EPIC connected an observed stair band %.2fm above the shorter guide",double(best.back().z()-body.z()-achieved));
        break;
      }
      if(attempted.size()>=3) break;
    }
  }
  if(!best.empty())
    ROS_INFO("EPIC confirmed previous stair shape with current measured space, rise %.2fm",double(best.back().z()-body.z()));
  return best;
}

std::optional<int> FastExplorationManager::recoverFlownTrack(const Vector3f &body) {
  const auto graph=planner_manager_->topo_graph_;
  const auto astar=graph->parallel_bubble_astar_;
  const auto map=astar->lidar_map_interface_;
  const float normal=astar->safe_distance_,current=map->getDisToOcc(body);
  if(!floor_by_floor_ || current>=narrowPassClearance(true)+.025f) return std::nullopt;
  auto track=floor_trace_.recentReturn(body,10.f*normal,normal);
  FrontierRoute exits;
  for(auto it=track.rbegin();it!=track.rend();++it) {
    if((*it-body).norm()<1.5f*normal || map->getDisToOcc(*it)<std::max(normal,float(graph->bubble_min_radius_))+.04f ||
       !frontier_manager_ptr_->observedFree(*it)) continue;
    if(exits.empty() || (*it-exits.back()).norm()>1.3f*normal) exits.push_back(*it);
    if(exits.size()==4) break;
  }
  if(exits.empty()) return std::nullopt;
  for(float margin:{narrowPassClearance(),narrowPassClearance(true)}) {
    const auto starts=passSearchStarts(body,margin);
    if(starts.empty()) continue;
    for(const auto &goal:exits) {
      auto route=observationStep(body,starts,goal,margin);
      if(route.size()<2 || (route.back()-goal).norm()>1.25f*normal ||
         map->getDisToOcc(route.back())<std::max(normal,float(graph->bubble_min_radius_))+.02f ||
         !certifyPassGuide(route,nullptr,margin)) continue;
      const auto leg=pass_.assess(route,0.,[this](const Vector3f &a,const Vector3f &b){return connectedAcrossPass(a,b);});
      reverse_pass_=leg.leg==CorridorPass::Leg::Return;
      level_passage_=false;survey_beyond_level_goal_=false;
      ROS_INFO("EPIC recovered from %.2fm clearance towards a wider point on the actually flown route",current);
      return setMeasuredGoal(body,route.back()-body,route,margin);
    }
  }
  // A known wider pose and an unresolved measured rising branch still
  // exist. Losing the departure in a fresh scan cannot complete exploration.
  ed_->global_tour_.clear();frontier_empty_since_=-1.;
  planner_manager_->graph_visualizer_->vizTour({},VizColor::RED,"global");
  ROS_WARN_THROTTLE(2.,"EPIC pending a certified return from %.2fm clearance to its previously flown track",current);
  return FRONTIER_PENDING;
}

std::optional<int> FastExplorationManager::planRisingOpening(const Vector3f &body) {
  if(outlet_commitment_) return std::nullopt;
  const auto graph = planner_manager_->topo_graph_;
  const auto astar = graph->parallel_bubble_astar_;
  const auto openings = measuredRises(body, astar->safe_distance_);
  if (openings.empty() && !floor_by_floor_) return std::nullopt;
  const auto starts = passSearchStarts(body);
  FrontierRoute route;
  double route_clearance=-1.;
  Vector3f opening;
  if (floor_by_floor_ && !openings.empty()) {
    route=chooseMeasuredAscent(body,planner_manager_->local_data_.curr_vel_.cast<float>(),openings,route_clearance);
    if(!route.empty()) opening=route.back();
  }
  for (const auto &p : openings) {
    if (!route.empty()) break;
    route = connectPassRoute(body, starts, p, .035);
    if (!route.empty()) { opening = p; break; }
  }
  if (route.empty() && !starts.empty() && !openings.empty()) {
    // A remote branch can require an initial detour. Reuse the witnessed
    // odometry/skeleton graph before choosing a local observation step.
    vector<TopoNode::Ptr> goals;
    for (const auto &p : openings) {
      Eigen::Vector3i index;
      graph->getIndex(p, index);
      if (!graph->getRegionNode(index)) continue;
      auto node = std::make_shared<TopoNode>();
      node->center_ = p;
      node->yaw_ = graph->odom_node_->yaw_;
      node->is_viewpoint_ = true;
      goals.push_back(node);
    }
    graph->insertNodes(goals, false);
    for (auto &goal : goals) {
      float yaw = graph->odom_node_->yaw_;
      if (getPathCost(graph->odom_node_, Vector3d::Zero(), yaw, goal, goal->yaw_, nullptr, &route) < 2e3 &&
          !route.empty()) { opening = goal->center_; break; }
      route.clear();
    }
    graph->removeNodes(goals);
  }
  if (route.empty()) {
    if (floor_by_floor_) for (const auto &start : starts) {
      route = frontier_manager_ptr_->levelPassage(start,pass_.heading(),astar->safe_distance_,
          [&](const Vector3f &p) {
            return std::any_of(blocked_points_.begin(),blocked_points_.end(),[&](const auto &entry){
              return (p-entry.first).norm()<.7f*astar->safe_distance_;
            });
          },float(graph->bubble_min_radius_)+.06f,true);
      if (route.size()<2) continue;
      if ((start-body).norm()>.01f) route.insert(route.begin(),body);
      opening=route.back();
      if (certifyStairStep(route)) break;
      route.clear();
    }
  }
  if (route.empty() && floor_by_floor_) {
    route=measuredNarrowPass(body,true,route_clearance);
    if (!route.empty()) opening=route.back();
  }
  if (route.empty()) {
    double best=-std::numeric_limits<double>::infinity();
    for (const auto &p : openings) {
      double margin=-1.;
      auto step = floor_by_floor_ ? approachPassGoal(body,starts,p,margin) : observationStep(body,starts,p);
      if (step.size()<2) continue;
      const auto &end=step.back();
      const double gain=(body-p).norm()-(end-p).norm();
      const double score=2.*(end.z()-body.z())+gain;
      if (score>best) {best=score;route=std::move(step);opening=p;route_clearance=margin;}
      if (!floor_by_floor_) break;
    }
  }
  if (route.empty()) {
    if (openings.empty()) return std::nullopt;
    if (floor_by_floor_ && starts.empty()) {
      const auto recovery=recoverFlownTrack(body);
      if(recovery) return recovery;
    }
    if (floor_by_floor_) {
      const float safe=astar->safe_distance_;
      const size_t evidence=frontier_manager_ptr_->nearbyFreeCells(body,4.f*safe);
      if (!unresolved_rise_origin_ || (body-*unresolved_rise_origin_).norm()>safe ||
          evidence>unresolved_rise_evidence_+12) {
        unresolved_rise_origin_=body;
        unresolved_rise_evidence_=evidence;
        unresolved_rise_since_=pass_scan_stamp_;
      }
      // The native graph, full-band grid, and intermediate routes have all
      // failed across fresh stationary scans. A visible but disconnected
      // ceiling pocket cannot keep an otherwise finished mission in WAIT_MAP.
      if (pass_scan_stamp_-unresolved_rise_since_>12.) return std::nullopt;
    }
    ed_->global_tour_.clear();
    frontier_empty_since_ = -1.;
    planner_manager_->graph_visualizer_->vizTour({}, VizColor::RED, "global");
    ROS_WARN_THROTTLE(2., "EPIC pending rising branch: %zu witnessed openings, %zu certified starts, body clearance %.2f",
        openings.size(), starts.size(), astar->lidar_map_interface_->getDisToOcc(body));
    return FRONTIER_PENDING;
  }
  const auto leg = pass_.assess(route, 0., [this](const Vector3f &a, const Vector3f &b) {
    return connectedAcrossPass(a, b);
  });
  const auto choice = pass_.select({leg}, pass_scan_stamp_, false);
  if (choice.indices.empty()) {
    ed_->global_tour_.clear();
    planner_manager_->graph_visualizer_->vizTour({}, VizColor::RED, "global");
    return FRONTIER_PENDING;
  }
  reverse_pass_ = choice.reverses();
  ROS_INFO("EPIC measured rising opening (%.2f %.2f %.2f), observation step (%.2f %.2f %.2f)",
      opening.x(), opening.y(), opening.z(), route.back().x(), route.back().y(), route.back().z());
  if (floor_by_floor_ && opening.z()>body.z()+.5f*astar->safe_distance_) {
    floor_trace_.startingAscent(body,astar->safe_distance_);
    if ((route.back()-opening).norm()<astar->safe_distance_) {
      landing_intent_=opening;
      landing_closest_=(body-opening).norm();
      landing_progress_stamp_=pass_scan_stamp_;
    }
  }
  return setMeasuredGoal(body, opening - body, route,route_clearance);
}

bool FastExplorationManager::readyToPreviewAscent(const Vector3f &body,const Vector3f &tip,const FrontierRoute *approach) const {
  if(outlet_commitment_) return false;
  const float safe=planner_manager_->topo_graph_->parallel_bubble_astar_->safe_distance_;
  if (visibleLevelOutlet(body,approach)) return false;
  if (floor_trace_.climbingAt(body,safe)) return true;
  // A sloped fringe visible at takeoff is not permission to leave the
  // current floor. For a small room the ordinary frontier selector decides
  // when to climb. Rolling extension only anticipates an established return.
  if (!floor_trace_.surveyedLevel(body,safe)) return false;
  for (const auto &branch:pending_passages_) {
    if (std::abs(branch.goal.z()-body.z())>1.6f*safe ||
        (branch.goal-tip).head<2>().norm()<5.f*safe || branch.next_attempt>pass_scan_stamp_ ||
        frontier_manager_ptr_->recentlySurveyed(branch.goal,3.6f*safe,1.5f*safe,.4f*safe) ||
        frontier_manager_ptr_->inspectedPassage(branch.goal,safe)) continue;
    if (branch.hasNewEvidence(frontier_manager_ptr_->nearbyFreeCells(branch.goal,3.f*safe)) &&
        (!floor_trace_.inAscentFootprint(branch.goal,safe) || visibleLevelOutlet(branch.goal))) return false;
  }
  return true;
}

bool FastExplorationManager::clearLastStairTread(const Vector3f &body) {
  if(!floor_by_floor_ || outlet_commitment_) return false;
  const auto graph=planner_manager_->topo_graph_;
  const auto astar=graph->parallel_bubble_astar_;
  const auto map=astar->lidar_map_interface_;
  const float safe=astar->safe_distance_;
  const auto heading=floor_trace_.landingArrival(body,safe);
  if(!heading || (sampled_rise_landing_ && (*sampled_rise_landing_-body).norm()<7.f*safe)) return false;
  const Vector3f side(-heading->y(),heading->x(),0.f);
  struct View {Vector3f point; float value;};
  std::vector<View> views;
  // The floor past a handrail can be visible but require an indirect approach.
  // Rank observed stand-off poses locally, then search the connected volume.
  for(int a=-7;a<=7;++a) {
    const float angle=.2f*a;
    const Vector3f axis=std::cos(angle)*(*heading)+std::sin(angle)*side;
    for(float distance:{1.8f,2.5f,3.3f,4.f}) for(float dz:{0.f,-.35f,.35f,-.6f,.6f}) {
      const Vector3f p=body+safe*(distance*axis+dz*Vector3f::UnitZ());
      if(!map->IsInBox(p) || !frontier_manager_ptr_->observedFree(p) ||
         p.z()<astar->flight_min_z_ || p.z()>astar->flight_max_z_) continue;
      const float room=map->getDisToOcc(p);
      if(room<graph->bubble_min_radius_+.055f ||
         frontier_manager_ptr_->recentlySurveyed(p,1.25f*safe,.8f*safe,.35f*safe)) continue;
      const float rank=.9f*std::min(2.f*safe,room)+.3f*distance*safe+.2f*(p-body).dot(*heading);
      views.push_back({p,rank});
    }
  }
  std::stable_sort(views.begin(),views.end(),[](const View &a,const View &b){return a.value>b.value;});
  FrontierRoute best,tried;
  float best_margin=safe;
  for(const auto &view:views) {
    if(tried.size()>=6) break;
    if(std::any_of(tried.begin(),tried.end(),[&](const Vector3f &p){return (p-view.point).norm()<.65f*safe;})) continue;
    tried.push_back(view.point);
    for(float margin:{safe,narrowPassClearance(),narrowPassClearance(true)}) {
      auto route=observationStep(body,passSearchStarts(body,margin),view.point,margin);
      if(route.size()<2 || (route.back()-view.point).norm()>.7f*safe ||
         (route.back()-body).head<2>().norm()<1.65f*safe ||
         map->getDisToOcc(route.back())<graph->bubble_min_radius_+.025f ||
         std::any_of(route.begin(),route.end(),[&](const Vector3f &p){return std::abs(p.z()-body.z())>.85f*safe;}) ||
         !certifyPassGuide(route,nullptr,margin,true)) continue;
      best=std::move(route);best_margin=margin;break;
    }
    if(!best.empty()) break;
  }
  if(best.empty()) return false;
  const Vector3f goal=best.back();
  sampled_rise_landing_=body;
  level_passage_=true;reverse_pass_=false;survey_beyond_level_goal_=false;
  level_guide_intent_=goal;
  landing_intent_=goal;landing_closest_=(goal-body).norm();landing_progress_stamp_=pass_scan_stamp_;
  setMeasuredGoal(body,goal-body,best,best_margin);
  ROS_INFO("EPIC surveyed the connected landing beyond the last rising tread, %.2fm",double((goal-body).norm()));
  return true;
}

bool FastExplorationManager::visibleLevelOutlet(const Vector3f &body,const FrontierRoute *approach) const {
  const auto astar=planner_manager_->topo_graph_->parallel_bubble_astar_;
  const auto map=astar->lidar_map_interface_;
  const float safe=astar->safe_distance_, room=narrowPassClearance(true);
  return hasHorizontalOutlet(body,safe,[&](const Vector3f &p) {
    return map->IsInBox(p) && frontier_manager_ptr_->observedFree(p) && map->getDisToOcc(p)>room;
  },[&](const Vector3f &p) {
    if(frontier_manager_ptr_->recentlySurveyed(p,3.f*safe,1.4f*safe,.4f*safe) ||
       frontier_manager_ptr_->inspectedPassage(p,safe)) return false;
    // A cell on the still-active staircase approach will be observed during
    // the same pass. Check a live unobstructed line before deferring it.
    // This is a local choice of direction; no cell is marked as visited.
    if(approach) for(size_t i=1;i<approach->size();++i) {
      const Vector3f a=(*approach)[i-1],edge=(*approach)[i]-a;
      const float t=std::clamp((p-a).dot(edge)/std::max(.00001f,edge.squaredNorm()),0.f,1.f);
      const Vector3f eye=a+t*edge,delta=p-eye;
      if(std::abs(delta.z())>1.5f*safe || delta.head<2>().norm()>4.6f*safe) continue;
      const int count=std::max(1,int(std::ceil(delta.norm()/(.45f*safe))));
      bool visible=true;
      for(int j=0;j<=count && visible;++j) {
        const Vector3f sample=eye+(float(j)/count)*delta;
        visible=map->IsInBox(sample) && frontier_manager_ptr_->observedFree(sample) &&
                map->getDisToOcc(sample)>.4f*safe;
      }
      if(visible) return false;
    }
    return true;
  });
}

bool FastExplorationManager::selectLevelPassage(const Vector3f &body, bool interrupting_ascent) {
  if (!floor_by_floor_) return false;
  const float safe = planner_manager_->topo_graph_->parallel_bubble_astar_->safe_distance_;
  // An ascent starts at the old floor's height. Only a newly observed
  // horizontal opening may interrupt it, even before the first altitude gain.
  if(interrupting_ascent && !outlet_commitment_) {
    FrontierRoute approach;
    const auto &active=ed_->path_next_goal_;
    if(floor_trace_.surveyedLevel(body,safe) && active.size()>1 && active.back().z()>body.z()+.65f*safe) {
      // Keep an already witnessed rise past the old floor's surveyed
      // doorway. Any reachable lateral space outside its visible approach
      // still owns the normal landing priority.
      approach=active;
      splitFrontierRoute(approach,16.f*safe);
    }
    if(!visibleLevelOutlet(body,approach.empty()?nullptr:&approach)) return false;
  }
  const double now = pass_scan_stamp_;
  auto excluded = [&](const Vector3f &p) {
    return std::any_of(blocked_points_.begin(), blocked_points_.end(), [&](const auto &entry) {
      return entry.second > now && (entry.first-p).squaredNorm() < safe*safe;
    });
  };
  const FrontierRoute starts = passSearchStarts(body);
  double margin=-1.;
  enum class Target { Passage, LandingPose, ObservedOutlet };
  auto publish = [&](FrontierRoute &route,const Vector3f &intent, Target kind,double clearance=-1.) {
    const bool landing=kind!=Target::Passage;
    if(kind==Target::ObservedOutlet) commitObservedOutlet(body,intent,safe);
    const auto leg = pass_.assess(route, 0., [this](const Vector3f &a, const Vector3f &b) {
      return connectedAcrossPass(a,b);
    });
    reverse_pass_ = leg.leg == CorridorPass::Leg::Return;
    level_passage_ = true;
    level_guide_intent_ = intent;
    // An outlet detected using an earlier floor is a horizontal survey
    // target. Its sampled coordinate only identifies the passage. A required
    // landing pose and the actual return-to-entrance still have endpoints.
    survey_beyond_level_goal_ = kind!=Target::LandingPose &&
        (!floor_trace_.entrance() || (intent-*floor_trace_.entrance()).norm()>safe);
    if (!landing || !floor_trace_.climbingAt(body,safe)) {
      FrontierRoute forward_branch;
      if (continueAcrossJunction(route,planner_manager_->local_data_.curr_vel_.cast<float>(),forward_branch))
        return true;
    }
    const Vector3f direction = route.back() - route[route.size()-2];
    ROS_INFO("EPIC level passage (%.2f %.2f %.2f) from (%.2f %.2f %.2f), intent (%.2f %.2f %.2f), landing=%d",
        route.back().x(),route.back().y(),route.back().z(),body.x(),body.y(),body.z(),intent.x(),intent.y(),intent.z(),int(landing));
    setMeasuredGoal(body,direction,route,clearance);
    return true;
  };
  auto remember = [&](const Vector3f &goal) {
    passage_intent_ = goal;
    passage_closest_ = (goal-body).norm();
    passage_progress_stamp_ = now;
  };
  // Reaching the altitude of a landing does not complete the crossing.
  // After the split climbing guide, new scan points may rule out the
  // original last pose while leaving a safe nearby observation pose.
  if(outlet_commitment_) {
    const Vector3f goal=outlet_commitment_->goal;
    auto route=approachPassGoal(body,starts,goal,margin);
    if(route.size()>1 && (goal-route.back()).norm()+.45f*safe<(goal-body).norm() &&
       certifyPassGuide(route,nullptr,margin)) {
      remember(goal);
      return publish(route,goal,Target::ObservedOutlet,margin);
    }
    // Leave the branch available for a later scan when a fresh obstruction
    // prevents further progress. A stale commitment must not starve the graph.
    if(interrupting_ascent) return false;
    deferred_outlet_=std::make_pair(goal,now+6.);
    outlet_commitment_.reset();
    if(passage_intent_ && (*passage_intent_-goal).norm()<safe) passage_intent_.reset();
  }
  if (landing_intent_) {
    const Vector3f to=*landing_intent_-body;
    const float gap=to.norm();
    if (gap+.25f<landing_closest_) {landing_closest_=gap;landing_progress_stamp_=now;}
    if (gap<safe || to.z()<-1.5f*safe || now-landing_progress_stamp_>10.) {
      landing_intent_.reset();
    } else if (std::abs(to.z())<safe && gap<6.f*safe) {
      auto step=approachPassGoal(body,starts,*landing_intent_,margin);
      if (step.size()>1 && (step.back()-*landing_intent_).norm()+.35f<gap)
        return publish(step,*landing_intent_,Target::LandingPose,margin);
    }
  }
  auto defer = [&](const Vector3f &p) {
    for (auto &branch : pending_passages_) if ((p-branch.goal).norm()<3.f*safe) branch.next_attempt=now+15.;
  };
  // The return to the entrance is a navigation leg with a real endpoint.
  // Ordinary observation goals can be retired after close measured sight.
  if(passage_intent_ && frontier_manager_ptr_->inspectedPassage(*passage_intent_,safe) &&
     (!floor_trace_.entrance() || (*passage_intent_-*floor_trace_.entrance()).norm()>safe)) {
    passage_intent_.reset();
  }
  if (passage_intent_) {
    const float gap = (*passage_intent_-body).norm();
    if (gap < 2.8f*safe || std::abs(passage_intent_->z()-body.z())>2.5f*safe) {
      if (gap < 2.8f*safe && (!floor_trace_.entrance() ||
          (*passage_intent_ - *floor_trace_.entrance()).norm() > safe)) {
        auto branch = std::find_if(pending_passages_.begin(), pending_passages_.end(),
            [&](const auto &b) { return (b.goal - *passage_intent_).norm() < .5f*safe; });
        if (branch == pending_passages_.end()) {
          pending_passages_.push_back({*passage_intent_, 0., std::nullopt});
          branch = std::prev(pending_passages_.end());
        }
        branch->free_cells_at_approach = frontier_manager_ptr_->nearbyFreeCells(branch->goal, 3.f*safe);
      }
      passage_intent_.reset();
    } else {
      if (gap+.45f < passage_closest_) {passage_closest_=gap; passage_progress_stamp_=now;}
      if (now-passage_progress_stamp_ < 10.) {
        auto route = approachPassGoal(body,starts,*passage_intent_,margin);
        if (!route.empty() && (route.back()-*passage_intent_).norm() < gap-.45f)
          return publish(route,*passage_intent_,Target::Passage,margin);
      } else {defer(*passage_intent_); passage_intent_.reset();}
    }
  }
  pending_passages_.erase(std::remove_if(pending_passages_.begin(),pending_passages_.end(),[&](const auto &branch) {
    return frontier_manager_ptr_->inspectedPassage(branch.goal,safe) || (!branch.free_cells_at_approach &&
        frontier_manager_ptr_->recentlySurveyed(branch.goal,3.6f*safe,1.5f*safe,.4f*safe));
  }),pending_passages_.end());
  auto awaiting_evidence = [&](const PassageBranch &branch) {
    return branch.free_cells_at_approach &&
        !branch.hasNewEvidence(frontier_manager_ptr_->nearbyFreeCells(branch.goal, 3.f*safe));
  };
  if(!chose_initial_opening_ && !interrupting_ascent) {
    chose_initial_opening_=true;
    const auto graph=planner_manager_->topo_graph_;
    const auto map=graph->parallel_bubble_astar_->lidar_map_interface_;
    const float end_room=graph->bubble_min_radius_+.06f;
    struct Opening {Vector3f point;float score,margin; FrontierRoute route;};
    std::vector<Opening> choices;
    for(float clearance:{safe,narrowPassClearance(),narrowPassClearance(true)}) {
      // Use the same fine connected search used to navigate a narrow doorway.
      // The coarse grid for general frontiers can miss the outlet at takeoff.
      const bool fine=clearance<safe-.01f;
      const auto origins=map->getDisToOcc(body)>clearance+.018f ? FrontierRoute{body} : passSearchStarts(body,clearance);
      if(origins.empty()) continue;
      const auto &origin=origins.front();
      FrontierRoute branches;
      auto local=frontier_manager_ptr_->levelPassage(origin,Vector3f::Zero(),clearance,excluded,
                                                    std::max(end_room,clearance+.025f),false,&branches,fine);
      if(local.size()<2) continue;
      if((local.front()-body).norm()>.02f) local.insert(local.begin(),body);
      branches.insert(branches.begin(),local.back());
      for(const auto &p:branches) {
        const Vector3f delta=p-body;
        if(delta.head<2>().norm()<2.5f*safe) continue;
        const float gain=horizontalObservationGain(p,safe,
            [&](const Vector3f &q){return frontier_manager_ptr_->observedFree(q);},
            [&](const Vector3f &q){return map->IsInBox(q)?float(map->getDisToOcc(q)):0.f;});
        if(gain<.035f) continue;
        const float reserve=(clearance-narrowPassClearance(true))/std::max(.05f,safe-narrowPassClearance(true));
        const float score=6.f*safe*gain+1.8f*safe*reserve+.15f*delta.head<2>().norm()-2.f*std::abs(delta.z());
        Opening option{p,score,clearance,(p-local.back()).norm()<.02f ? local : FrontierRoute{}};
        const auto duplicate=std::find_if(choices.begin(),choices.end(),[&](const Opening &o) {
          return std::abs(o.margin-clearance)<.001f && (o.point-p).norm()<1.1f*safe;
        });
        if(duplicate==choices.end()) choices.push_back(std::move(option));
        else if(score>duplicate->score) *duplicate=std::move(option);
      }
    }
    std::stable_sort(choices.begin(),choices.end(),[](const Opening &a,const Opening &b){return a.score>b.score;});
    for(size_t i=0;i<std::min<size_t>(8,choices.size());++i) {
      auto &option=choices[i];
      if(option.route.size()<2 || !certifyPassGuide(option.route,nullptr,option.margin)) {
        auto origins=passSearchStarts(body,option.margin);
        option.route=observationStep(body,origins,option.point,option.margin);
      }
      if(option.route.size()<2) continue;
      const float range=(option.point-body).norm(),advance=range-(option.point-option.route.back()).norm();
      if(advance<std::max(2.2f*safe,.45f*range) || !certifyPassGuide(option.route,nullptr,option.margin)) continue;
      // The connected frontier is the intent; hand off only the measured
      // part available from the current side of the opening.
      remember(option.point);
      ROS_INFO("EPIC selected a connected initial observation, %.2fm of %.2fm confirmed with %.2fm clearance",
               double(advance),double(range),double(option.margin));
      // The first connected aperture has the same crossing semantics as
      // one found on a higher floor. Extend ahead while the vehicle enters
      // it; examine unrelated side branches from its observed opening.
      return publish(option.route,option.point,Target::ObservedOutlet,option.margin);
    }
    ROS_INFO("EPIC inspected %zu initial horizontal apertures before selecting a native passage",choices.size());
  }
  FrontierRoute local;
  for (const auto &start : starts) {
    const float endpoint = std::max(safe + .1f, float(planner_manager_->topo_graph_->bubble_min_radius_) + .05f);
    FrontierRoute branches;
    auto route = frontier_manager_ptr_->levelPassage(start, pass_.heading(), safe, excluded, endpoint,false,&branches);
    for (const auto &p : branches) rememberPassage(p);
    if (route.size() < 2) continue;
    if ((start-body).norm() > .01f) route.insert(route.begin(), body);
    if (!certifyPassGuide(route)) continue;
    local=std::move(route);
    break;
  }
  Vector3f forward=pass_.heading(); forward.z()=0.f;
  const bool moving_along = forward.norm()>.75f;
  if (moving_along) forward.normalize();
  auto advances = [&](const Vector3f &target) {
    const Vector3f d=target-body;
    return moving_along && forward.dot(d) > .72f*d.norm() && std::abs(d.z())<1.5f*safe;
  };
  // At takeoff there is no previous corridor heading. The connected
  // horizontal flood already ranks the measured openings by reachable
  // space; a nearer remembered ray endpoint is not a separate branch.
  if (!local.empty() && (!moving_along || advances(local.back()))) {
    if(!moving_along) ROS_INFO("EPIC selected the first connected horizontal passage");
    remember(local.back());
    return publish(local,*passage_intent_,Target::Passage);
  }
  if (moving_along && frontier_manager_ptr_->recentlySurveyed(body-2.8f*safe*forward,safe,.9f*safe)) {
    auto step=approachPassGoal(body,starts,body+10.f*safe*forward,margin);
    if (!step.empty() && advances(step.back()) &&
        !frontier_manager_ptr_->recentlySurveyed(step.back(),2.8f*safe,safe,.4f*safe) &&
        frontier_manager_ptr_->unobservedViewGain(step.back(),safe)>0.f) {
      remember(step.back());
      return publish(step,*passage_intent_,Target::Passage,margin);
    }
  }
  const auto outlets = frontier_manager_ptr_->landingPassages(body,safe);
  // The current storey's measured openings backed by an earlier floor
  // track are still outstanding when an individual ray or pose expires.
  // Prefer their continuation over turning back towards the staircase.
  for (const auto &outlet : outlets) {
    if (!local.empty() && !advances(outlet) && (outlet-body).head<2>().norm()<8.f*safe) continue;
    if (std::any_of(pending_passages_.begin(), pending_passages_.end(), [&](const auto &branch) {
          return (branch.goal-outlet).norm() < .5f*safe && awaiting_evidence(branch);
        })) continue;
    auto step=approachPassGoal(body,starts,outlet,margin);
    if (step.size()<2 || (step.back()-outlet).norm()+safe>(body-outlet).norm()) continue;
    remember(outlet);
    return publish(step,outlet,Target::ObservedOutlet,margin);
  }
  // When a new local winner would leave this passage, service a witnessed
  // branch still outstanding on the landing. Visiting a previous floor is
  // never inferred from reaching the same XY upstairs.
  std::sort(pending_passages_.begin(),pending_passages_.end(),[&](const auto &a,const auto &b){
    auto cost=[&](const Vector3f &p) {
      return (p-body).norm() + 4.f*std::abs(p.z()-body.z()) - (advances(p) ? 8.f*safe : 0.f);
    };
    return cost(a.goal)<cost(b.goal);
  });
  size_t tried=0;
  for (auto &b : pending_passages_) {
    if (frontier_manager_ptr_->inspectedPassage(b.goal,safe) || now < b.next_attempt || std::abs(b.goal.z()-body.z())>1.8f*safe ||
        (!local.empty() && !advances(b.goal) && std::abs(local.back().z()-body.z())<.7f*safe)) continue;
    if (awaiting_evidence(b) || frontier_manager_ptr_->unobservedViewGain(b.goal,safe)==0.f ||
        frontier_manager_ptr_->recentlySurveyed(b.goal,3.6f*safe,1.5f*safe,.4f*safe))
      continue;
    auto route=approachPassGoal(body,starts,b.goal,margin);
    if (!route.empty() && (route.back()-b.goal).norm()+.5f < (body-b.goal).norm()) {
      ROS_INFO("EPIC approaching a stored horizontal observation %.2fm away",double((b.goal-body).norm()));
      remember(b.goal);
      return publish(route,*passage_intent_,Target::Passage,margin);
    }
    b.next_attempt=now+3.;
    if (++tried==4) break;
  }
  if (!local.empty()) {
    remember(local.back());
    return publish(local,*passage_intent_,Target::Passage);
  }
  if (moving_along || starts.empty()) {
    auto route = measuredNarrowPass(body,false,margin);
    if (!route.empty()) {
      remember(route.back());
      return publish(route,*passage_intent_,Target::Passage,margin);
    }
  }
  // A previously flown return to the old entrance is not a new landing
  // observation. Keep a progressing measured ascent in that case.
  if (!interrupting_ascent && hasReturnWork(body,safe)) {
    auto back=floor_trace_.returnPath(body,std::max(12.f*safe,float(level_preview_distance_)),safe);
    const float room=planner_manager_->topo_graph_->bubble_min_radius_+.05f;
    const auto target=back.size()>1 ? back.back() : *floor_trace_.entrance();
    FrontierRoute route=repairFloorReturn(back,margin);
    // Give the follower the longest still-clear prefix of the measured trail.
    // A newly obstructed distant sample need not discard the approach to it.
    while (route.empty() && back.size()>1) {
      if (planner_manager_->lidar_map_interface_->getDisToOcc(back.back())>=room) {
        route=back;
        margin=-1.;
        if (certifyPassGuide(route)) break;
        route=back;
        margin=narrowPassClearance();
        if (certifyPassGuide(route,nullptr,margin)) break;
        route=back;
        margin=narrowPassClearance(true);
        if (certifyPassGuide(route,nullptr,margin)) break;
        route.clear();
      }
      back.pop_back();
    }
    if (back.size()<2) route=approachPassGoal(body,starts,target,margin);
    if (route.size()<2 || (route.back()-body).norm()<safe)
      route=approachPassGoal(body,starts,*floor_trace_.entrance(),margin);
    if (route.size()>1 && (route.back()-body).norm()>safe) {
      remember(*floor_trace_.entrance());
      ROS_INFO("EPIC returning along measured floor track to entrance (%.2f %.2f %.2f), clearance %.2f",
          floor_trace_.entrance()->x(),floor_trace_.entrance()->y(),floor_trace_.entrance()->z(),margin<0.?safe:margin);
      return publish(route,*floor_trace_.entrance(),Target::Passage,margin);
    }
  }
  return false;
}

bool FastExplorationManager::extendThroughPassage(FrontierRoute &guide,double clearance) {
  if (!floor_by_floor_ || !level_passage_ || end_turn_ || guide.size()<2) return false;
  const auto graph=planner_manager_->topo_graph_;
  const auto astar=graph->parallel_bubble_astar_;
  const float safe=astar->safe_distance_;
  const Vector3f body=guide.front(), tip=guide.back();
  if((body-tip).norm()>10.f*safe || (body-tip).norm()<.2f*safe || !frontier_manager_ptr_->observedFree(tip)) return false;
  size_t vertex=guide.size()-1;
  while(vertex && (tip-guide[vertex]).norm()<2.f*safe) --vertex;
  const Vector3f approach=tip-guide[vertex];
  if(approach.head<2>().norm()<.45f*safe) return false;
  const Vector3f forward=approach.normalized();
  const auto starts=passSearchStarts(tip);
  const double old_margin=clearance<0. ? safe : clearance;
  auto join=[&](FrontierRoute extra, const Vector3f &intent,double margin,bool rising,bool surveyed) {
    if(extra.size()<2) return false;
    const Vector3f delta=extra.back()-tip;
    if(delta.norm()<2.2f*safe || (extra.back()-body).norm()<2.f*safe ||
       (delta.normalized().dot(forward)<-.45f && !rising)) return false;
    // A remote endpoint can be ahead even though reaching it would
    // immediately undo the passage we are still traversing. Apply the
    // same connected, ordered test used for native tour extensions.
    if(!rising && !surveyed && !pass_.allowsPreview(guide,extra,reverse_pass_,
        [this](const Vector3f &a,const Vector3f &b){return connectedAcrossPass(a,b);})) return false;
    if(!certifyPassGuide(extra,nullptr,margin)) return false;
    const float gain=delta.norm();
    const double next_margin=margin<0. ? safe : margin;
    auto joined=guide;
    if(next_margin<old_margin && !certifyPassGuide(joined,nullptr,old_margin)) return false;
    joined.insert(joined.end(),extra.begin()+1,extra.end());
    const double common=std::min(old_margin,next_margin);
    if(!certifyPassGuide(joined,nullptr,common)) return false;
    level_passage_=!rising;
    reverse_pass_=false;
    if(rising) {
      passage_intent_.reset();landing_intent_=intent;
      landing_closest_=(body-intent).norm();landing_progress_stamp_=pass_scan_stamp_;
      floor_trace_.startingAscent(body,safe);
    } else {
      level_guide_intent_=intent;passage_intent_=intent;
      passage_closest_=(body-intent).norm();passage_progress_stamp_=pass_scan_stamp_;
      survey_beyond_level_goal_=!surveyed;
    }
    setMeasuredGoal(body,delta,joined,common);
    guide.swap(joined);
    ROS_INFO("EPIC connected the next observed %s before the passage endpoint, gain %.2fm",rising?"ascent":"bend",gain);
    return true;
  };
  // Existing intent can be around a corner beyond a truncated measured guide.
  // A pure map query at that future location leaves actual visitation intact.
  const bool returning=floor_trace_.entrance() && (level_guide_intent_-*floor_trace_.entrance()).norm()<2.5f*safe;
  const Vector3f to_intent=level_guide_intent_-tip;
  if(to_intent.norm()>2.4f*safe && to_intent.norm()<level_preview_distance_ &&
     to_intent.normalized().dot(forward)>-.45f) {
    double margin=-1.;
    auto tail=approachPassGoal(tip,starts,level_guide_intent_,margin);
    if(tail.size()>=2 && (tail.back()-level_guide_intent_).norm()+1.2f*safe<to_intent.norm() &&
        join(tail,level_guide_intent_,margin,false,returning)) return true;
  }
  // The next horizontal junction is selected after physically reaching the
  // observed doorway. Straight preview towards and beyond it stays available.
  if(outlet_commitment_ || (returning && to_intent.norm()>3.f*safe)) return false;
  if(!returning || visibleLevelOutlet(tip)) {
    const float endpoint=std::max(safe+.1f,float(graph->bubble_min_radius_)+.05f);
    for(float margin:{safe,narrowPassClearance()}) {
      auto tail=frontier_manager_ptr_->levelPassage(tip,forward,margin,[](const Vector3f &){return false;},endpoint);
      if(tail.size()<2 || (tail.back()-tip).head<2>().norm()<3.5f*safe) continue;
      const Vector3f goal=tail.back();
      if(join(tail,goal,margin,false,false)) return true;
    }
  }
  if(!returning || !readyToPreviewAscent(body,tip)) return false;
  const auto openings=measuredRises(tip,safe);
  if(openings.empty()) return false;
  double margin=-1.;
  auto climb=measuredPreviousAscent(tip,margin);
  if(climb.empty()) climb=measuredClimb(tip,forward,openings,margin);
  if(climb.size()>1) {
    const Vector3f goal=climb.back();
    if(join(climb,goal,margin,true,false)) return true;
  }
  return false;
}

bool FastExplorationManager::interceptReturnAscent(FrontierRoute &guide,const Vector3f &velocity,double clearance) {
  if(!floor_by_floor_ || !level_passage_ || outlet_commitment_ || end_turn_ || guide.size()<2 ||
     !floor_trace_.entrance()) return false;
  const auto graph=planner_manager_->topo_graph_;
  const auto astar=graph->parallel_bubble_astar_;
  const float safe=astar->safe_distance_;
  const Vector3f body=guide.front(), entrance=*floor_trace_.entrance();
  if(!floor_trace_.surveyedLevel(body,safe) || (body-entrance).head<2>().norm()>12.f*safe) return false;
  const bool formal_return=(level_guide_intent_-entrance).norm()<=2.5f*safe;
  const bool through_door=floor_trace_.returningThroughEntrance(guide,safe);
  if(!formal_return && !through_door) {
    const Vector3f &tip=guide.back();
    // The centre of the stairwell and its doorway can be different
    // frontier goals. While returning along an already inspected corridor,
    // a reachable rise next to this path matters even after the doorway
    // itself has been passed. An unvisited side opening still owns priority.
    if((tip-entrance).head<2>().norm()>9.f*safe || std::abs(tip.z()-entrance.z())>1.5f*safe ||
       !frontier_manager_ptr_->recentlySurveyed(tip,3.f*safe,1.4f*safe,.4f*safe) ||
       visibleLevelOutlet(tip)) return false;
  }
  if(!through_door && !readyToPreviewAscent(body,entrance)) return false;
  const double active=clearance<0. ? safe : clearance;
  FrontierRoute best;
  double best_rank=-std::numeric_limits<double>::infinity(),best_margin=-1.;
  // The common entrance on the previous ascent describes where the floor
  // connects. It is not a waypoint to reach before turning upwards. Compare
  // observed turns off the *current* approach while they are still ahead.
  float arc=0.f;
  for(size_t i=1;i<guide.size();++i) arc+=(guide[i]-guide[i-1]).norm();
  float last=-1.f;
  for(float requested:{0.f,2.4f*safe,5.f*safe,8.f*safe}) {
    const float ahead=std::min(arc,requested);
    if(last>=0.f && ahead-last<.35f*safe) continue;
    last=ahead;
    FrontierRoute prefix=guide;
    if(ahead>0.f) splitFrontierRoute(prefix,ahead);
    else prefix.resize(1);
    const Vector3f anchor=prefix.back();
    if(std::abs(anchor.z()-body.z())>1.5f*safe ||
       (anchor-entrance).head<2>().norm()>9.f*safe) continue;
    Vector3f incoming=velocity;
    if(prefix.size()>1) {
      for(size_t i=prefix.size()-1;i>0;--i) {
        incoming=anchor-prefix[i-1];if(incoming.norm()>safe) break;
      }
    }
    double margin=-1.;
    auto climb=measuredPreviousAscent(anchor,margin);
    if(climb.size()<2) {
      const auto openings=measuredRises(anchor,safe);
      if(openings.empty()) continue;
      climb=measuredClimb(anchor,incoming,openings,margin);
    }
    if(climb.size()<2 || climb.back().z()<anchor.z()+1.05f*safe) continue;
    const auto initial=FastPlannerManager::boundedGuidePrefix(climb,2.2f*safe);
    if(initial.size()<2) continue;
    const Vector3f forward=initial.back().cast<float>()-anchor;
    const float alignment=incoming.norm()>.4f && forward.norm()>.4f ?
                          incoming.normalized().dot(forward.normalized()) : 0.f;
    if(alignment<-.2f) continue;
    double length=0.;
    for(size_t j=1;j<climb.size();++j) length+=(climb[j]-climb[j-1]).norm();
    const double rise=climb.back().z()-anchor.z();
    const double rank=2.5*rise-.17*length-.27*ahead+.7*safe*alignment;
    if(rank<=best_rank) continue;
    auto joined=prefix;
    if(joined.size()>1 && !certifyPassGuide(joined,nullptr,active)) continue;
    joined.insert(joined.end(),climb.begin()+1,climb.end());
    const double common=std::min(active,margin<0. ? safe : margin);
    if(!certifyPassGuide(joined,nullptr,common)) continue;
    best_rank=rank;best_margin=common;best=std::move(joined);
  }
  if(best.size()<2 || (through_door && !readyToPreviewAscent(body,entrance,&best))) return false;
  const Vector3f tip=best.back();
  floor_trace_.startingAscent(body,safe);
  level_passage_=false;reverse_pass_=false;passage_intent_.reset();
  landing_intent_=tip;landing_closest_=(tip-body).norm();landing_progress_stamp_=pass_scan_stamp_;
  setMeasuredGoal(body,tip-best[best.size()-2],best,best_margin);
  guide.swap(best);
  ROS_INFO("EPIC joined a measured ascent before reaching the floor entrance, remaining entrance distance %.2fm",
           double((body-entrance).norm()));
  return true;
}

bool FastExplorationManager::inspectStoreyOutlet(FrontierRoute &prefix,double clearance) {
  if(!floor_by_floor_ || prefix.empty() || outlet_commitment_) return false;
  const auto graph=planner_manager_->topo_graph_;
  const auto astar=graph->parallel_bubble_astar_;
  const auto map=astar->lidar_map_interface_;
  const float safe=astar->safe_distance_;
  const Vector3f body=prefix.front(),tip=prefix.back();
  const auto probes=floor_trace_.outletProbes(tip,safe);
  if(probes.empty()) return false;
  const double active=clearance<0. ? safe : clearance;
  std::vector<std::pair<float,Vector3f>> seen;
  for(const auto &probe:probes) {
    // Try the projection only at a location that the current storey's
    // own LiDAR rays traverse, with enough clearance for an observation.
    Vector3f ahead=probe-tip;ahead.z()=0.f;
    if(ahead.norm()<.1f) continue;
    ahead.normalize();
    const Vector3f side(-ahead.y(),ahead.x(),0.f);
    const std::array<Vector3f,5> shifts{Vector3f::Zero(),.5f*safe*side,-.5f*safe*side,
                                          .45f*safe*ahead,-.45f*safe*ahead};
    float best=-std::numeric_limits<float>::infinity();Vector3f best_point=probe;
    for(const auto &shift:shifts) for(float dz:{0.f,-.5f*safe,.5f*safe}) {
      const Vector3f p=probe+shift+Vector3f(0,0,dz);
      if(std::abs(p.z()-tip.z())>1.55f*safe || !map->IsInBox(p) ||
         !frontier_manager_ptr_->observedFree(p) ||
         frontier_manager_ptr_->recentlySurveyed(p,1.9f*safe,safe,.4f*safe)) continue;
      const float room=map->getDisToOcc(p);
      if(room<graph->bubble_min_radius_+.06f) continue;
      const float score=(p-tip).head<2>().norm()+.7f*std::min(room,1.7f*safe)-
                        .8f*std::abs(p.z()-tip.z())-.55f*(p-probe).norm();
      if(score>best) {best=score;best_point=p;}
    }
    if(std::isfinite(best)) seen.emplace_back(best,best_point);
  }
  if(seen.empty()) return false;
  std::sort(seen.begin(),seen.end(),[](const auto &a,const auto &b){return a.first>b.first;});
  const auto starts=passSearchStarts(tip);
  FrontierRoute tested;
  for(const auto &[score,goal]:seen) {
    if(std::any_of(tested.begin(),tested.end(),[&](const auto &p){return (p-goal).norm()<1.6f*safe;})) continue;
    if(deferred_outlet_ && deferred_outlet_->second>pass_scan_stamp_ &&
       (goal-deferred_outlet_->first).norm()<2.5f*safe) continue;
    tested.push_back(goal);
    double margin=-1.;
    auto route=approachPassGoal(tip,starts,goal,margin);
    if(route.size()>=2 && (route.back()-goal).norm()<.9f*safe &&
       (route.back()-tip).head<2>().norm()>2.3f*safe &&
       withinEndpointHeights(route,tip,route.back(),safe*.7f) && certifyPassGuide(route,nullptr,margin)) {
      const double common=std::min(active,margin<0. ? safe : margin);
      auto joined=prefix;
      if(joined.size()>1 && !certifyPassGuide(joined,nullptr,active)) return false;
      joined.insert(joined.end(),route.begin()+1,route.end());
      if(!certifyPassGuide(joined,nullptr,common)) continue;
      const Vector3f target=joined.back();
      commitObservedOutlet(body,target,safe);
      landing_intent_.reset();passage_intent_=target;level_guide_intent_=target;
      survey_beyond_level_goal_=true;level_passage_=true;
      // The egress from a measured blind recess legitimately retraces its
      // approach. Use the same departure classification as a normal branch.
      const auto leg=pass_.assess(joined,0.,[this](const Vector3f &a,const Vector3f &b) {
        return connectedAcrossPass(a,b);
      });
      reverse_pass_=leg.leg==CorridorPass::Leg::Return;
      passage_closest_=(target-body).norm();passage_progress_stamp_=pass_scan_stamp_;
      setMeasuredGoal(body,target-tip,joined,common);prefix.swap(joined);
      ROS_INFO("EPIC observed a storey outlet along an earlier floor departure at (%.2f %.2f %.2f)",
               target.x(),target.y(),target.z());
      return true;
    }
    if(tested.size()>=3) break;
  }
  return false;
}

bool FastExplorationManager::extendThroughLevelOutlet(FrontierRoute &guide,double clearance) {
  if (!floor_by_floor_ || outlet_commitment_ || guide.size()<2) return false;
  const auto graph=planner_manager_->topo_graph_;
  const auto astar=graph->parallel_bubble_astar_;
  const float safe=astar->safe_distance_;
  const Vector3f body=guide.front(),tip=guide.back();
  if(tip.z()>=body.z()-.4f*safe && inspectStoreyOutlet(guide,clearance)) return true;
  if (!floor_trace_.climbingAt(tip,safe) || tip.z()<body.z()-.4f*safe || !visibleLevelOutlet(tip)) return false;
  if (level_passage_ && (tip-level_guide_intent_).norm()>2.5f*safe) return false;
  Vector3f incoming=tip-guide[guide.size()-2];
  if(incoming.norm()>.01f) incoming.normalize();
  const float required=graph->bubble_min_radius_+.06f;
  for(float margin:{safe,narrowPassClearance(),narrowPassClearance(true)}) {
    auto route=frontier_manager_ptr_->levelPassage(tip,incoming,margin,[](const Vector3f &) {return false;},required);
    if(route.size()<2) continue;
    const Vector3f delta=route.back()-tip;
    if(delta.head<2>().norm()<3.5f*safe || std::abs(delta.z())>safe ||
       frontier_manager_ptr_->recentlySurveyed(route.back(),2.8f*safe,safe,.4f*safe)) continue;
    if(!certifyPassGuide(route,nullptr,margin)) continue;
    auto joined=guide;
    const double active=clearance<0. ? safe : clearance;
    if(margin<active && !certifyPassGuide(joined,nullptr,active)) return false;
    joined.insert(joined.end(),route.begin()+1,route.end());
    const double common=std::min(active,double(margin));
    if(!certifyPassGuide(joined,nullptr,common)) continue;
    const Vector3f goal=joined.back();
    level_passage_=true;
    survey_beyond_level_goal_=true;
    landing_intent_.reset();
    level_guide_intent_=goal;
    passage_intent_=goal;
    passage_closest_=(body-goal).norm();
    passage_progress_stamp_=pass_scan_stamp_;
    setMeasuredGoal(body,delta,joined,common);
    guide.swap(joined);
    ROS_INFO("EPIC joined a measured horizontal outlet %.2fm beyond the climbing guide",double(delta.head<2>().norm()));
    return true;
  }
  return false;
}

FrontierRoute FastExplorationManager::previewMeasuredPass(const FrontierRoute &path,const Vector3f &velocity,
                                                            float required,float active,double &certified) const {
  const auto astar=planner_manager_->topo_graph_->parallel_bubble_astar_;
  const float safe=astar->safe_distance_;
  const float normal=active<0.f ? safe : active;
  const auto map=astar->lidar_map_interface_;
  const bool need_long=level_passage_ && required>7.f*safe && velocity.head<2>().norm()>1.5f;
  float latest=normal+.01f;
  for(float tier:{normal,narrowPassClearance(),narrowPassClearance(true)}) {
    if(tier>=latest-.005f) continue;
    if(tier<normal-.005f && !need_long) break;
    latest=tier;
    auto room=[&](const Vector3f &p) {
      if(!map->IsInBox(p) || !frontier_manager_ptr_->observedFree(p) ||
          p.z()<astar->flight_min_z_ || p.z()>astar->flight_max_z_) return 0.f;
      return float(map->getDisToOcc(p));
    };
    auto clear=[&](const Vector3f &a,const Vector3f &b) {
      FrontierRoute edge{a,b};
      return observedPass(edge) && astar->collisionCheck_shortenPath(edge,tier);
    };
    FrontierRoute alternate;
    // Recheck both walls while traversing a long aisle. The current chord
    // can still be long while its far end has moved next to an obstacle.
    if(need_long && intermediate_observation_) {
      alternate=centerObservedAisle(velocity,path,tier,room,clear);
      if(alternate.size()>1 && (alternate[1]-alternate.front()).norm()<std::min(6.f*safe,.65f*required))
        alternate.clear();
    }
    if(alternate.empty()) alternate=rollingVisibilityGuide(velocity,path,tier,astar->resolution_,room,clear,required);
    if(alternate.size()<2) continue;
    // Keep the established tier unless the narrower, already measured line
    // yields several metres of preview and its entire ordered route is valid.
    if(tier<normal-.005f && ((alternate[1]-alternate[0]).norm()<std::min(required*.65f,6.f*safe) ||
                              !certifyPassGuide(alternate,nullptr,tier,true))) continue;
    certified=tier;
    return alternate;
  }
  return {};
}

bool FastExplorationManager::improveTransitBend(FrontierRoute &path,const Vector3f &velocity) {
  if(!intermediate_observation_ || followingEndTurn() || path.size()<3) return false;
  const auto astar=planner_manager_->topo_graph_->parallel_bubble_astar_;
  const auto &limits=planner_manager_->gcopter_config_;
  const float safe=astar->safe_distance_,normal=selected_pass_clearance_<0. ? safe : selected_pass_clearance_;
  const double horizon=12.*safe;
  auto duration=[&](const FrontierRoute &route) {
    auto local=route;
    splitFrontierRoute(local,horizon);
    return estimateTransitTime(local,velocity,.6f*safe,limits->maxVelMag,limits->maxAccMag,limits->maxJerkMag);
  };
  const double original=duration(path);
  if(!std::isfinite(original)) return false;
  float last=normal+.01f;
  for(float margin:{normal,narrowPassClearance(),narrowPassClearance(true)}) {
    if(margin>=last-.005f) continue;
    last=margin;
    auto clear=[&](const Vector3f &a,const Vector3f &b){
      FrontierRoute segment{a,b};
      return observedPass(segment) && astar->collisionCheck_shortenPath(segment,margin);
    };
    auto improved=fitMeasuredBend(path,margin,2.f*astar->resolution_,clear);
    const auto map=astar->lidar_map_interface_;
    auto alternate=optimizeTransitWindow(path,velocity,margin,limits->maxVelMag,limits->maxAccMag,limits->maxJerkMag,
        [&](const Vector3f &p) {
          if(!map->IsInBox(p) || !frontier_manager_ptr_->observedFree(p) ||
             p.z()<astar->flight_min_z_ || p.z()>astar->flight_max_z_) return 0.f;
          return float(map->getDisToOcc(p));
        },clear);
    const bool changed=!alternate.empty() && (improved.empty() || duration(alternate)<.96*duration(improved));
    if(changed) improved.swap(alternate);
    if(improved.empty()) continue;
    // A smaller established clearance is justified only for a shorter predicted
    // travel time, with the complete resulting guide rechecked at that margin.
    if(margin<normal-.005f && duration(improved)>.88*original) continue;
    if(!certifyPassGuide(improved,nullptr,margin,true)) continue;
    path.swap(improved);
    if(changed) ROS_INFO("EPIC selected a lower-curvature passage between measured cross sections");
    selected_pass_clearance_=margin;
    ROS_INFO("EPIC fitted a measured transit bend at %.2fm clearance",double(margin));
    return true;
  }
  return false;
}

bool FastExplorationManager::followsMeasuredFloor(const FrontierRoute &guide) const {
  if(!floor_by_floor_ || !level_passage_ || guide.size()<2) return false;
  const float safe=planner_manager_->topo_graph_->parallel_bubble_astar_->safe_distance_;
  const Vector3f body=guide.front();
  const auto prefix=FastPlannerManager::boundedGuidePrefix(guide,8.f*safe);
  if(prefix.size()<2) return false;
  Vector3f axis=prefix.back().cast<float>()-body;
  axis.z()=0.f;
  const float distance=axis.norm();
  if(distance<2.8f*safe) return false;
  axis/=distance;
  if(pass_.measuredApproach(body,axis)<6.f*safe) return false;
  // A recovering height error at the body can exceed the height threshold
  // for a stair. The measured incoming track and the forward lane jointly
  // distinguish a straight hallway from an actual ramp or a switchback.
  const float height=prefix.back().z();
  float along=0.f;
  for(const auto &point:prefix) {
    const Vector3f delta=point.cast<float>()-body;
    const float next=delta.dot(axis);
    if(next<along-.3f*safe || (delta-next*axis).head<2>().norm()>.8f*safe ||
       std::abs(point.z()-height)>(next<1.5f*safe ? 1.25f : .7f)*safe) return false;
    along=std::max(along,next);
  }
  return true;
}

bool FastExplorationManager::onMeasuredStairLanding(const FrontierRoute &guide) const {
  if(!floor_by_floor_ || outlet_commitment_ || !level_passage_ || guide.size()<2) return false;
  const float safe=planner_manager_->topo_graph_->parallel_bubble_astar_->safe_distance_;
  const Vector3f &body=guide.front(),&tip=guide.back();
  // A landing may be level between two flights. Require the body's measured
  // climb and the footprint of a sloping, previously flown connection. A
  // flat corridor at an analogous height does not qualify on height alone.
  if(!floor_trace_.climbingAt(body,safe) || !floor_trace_.inAscentFootprint(tip,safe) ||
     (level_guide_intent_-tip).norm()>2.5f*safe || tip.z()<body.z()-.35f*safe ||
     (tip-body).head<2>().norm()>9.f*safe) return false;
  return !visibleLevelOutlet(body) && !visibleLevelOutlet(tip);
}

bool FastExplorationManager::endingInRise(const FrontierRoute &guide) const {
  if(!floor_by_floor_ || guide.size()<2) return false;
  const float safe=planner_manager_->topo_graph_->parallel_bubble_astar_->safe_distance_;
  const Vector3f tip=guide.back();
  float length=0.f;
  Vector3f before=tip;
  // Use the actual tail of the active path. A guide can start out level,
  // turn into the stairwell, and become steep many metres after its start.
  for(size_t i=guide.size()-1;i>0 && length<4.f*safe;--i) {
    const Vector3f edge=guide[i-1]-guide[i];
    const float step=std::min(edge.norm(),4.f*safe-length);
    if(step<.001f) continue;
    before+=step*edge.normalized();
    length+=step;
  }
  const Vector3f delta=tip-before;
  return length>1.5f*safe && delta.z()>.6f*safe && delta.z()>.17f*length;
}

bool FastExplorationManager::continueBeforeStairTip(FrontierRoute &guide,const Vector3f &velocity,double clearance) {
  if(!floor_by_floor_ || !intermediate_observation_ || outlet_commitment_ || end_turn_ ||
     guide.size()<2 || pass_scan_stamp_-last_stair_bypass_scan_<.4) return false;
  const auto graph=planner_manager_->topo_graph_;
  const float safe=graph->parallel_bubble_astar_->safe_distance_;
  const Vector3f body=guide.front(),tip=guide.back();
  if(!endingInRise(guide) && !floor_trace_.climbingAt(body,safe)) return false;
  if(tip.z()<body.z()-.3f*safe) return false;
  float length=0.f;
  for(size_t i=1;i<guide.size();++i) length+=(guide[i]-guide[i-1]).norm();
  if(length<.5f*safe || length>9.f*safe) return false;
  last_stair_bypass_scan_=pass_scan_stamp_;
  const double active=clearance<0. ? safe : clearance;
  // The intermediate observation can lie in a pocket beside the through
  // lane. If no extension from that tip exists, try leaving the certified
  // approach a little earlier. Keep actual storey outlets ahead of a climb.
  const auto astar=graph->parallel_bubble_astar_;
  const auto &limits=planner_manager_->gcopter_config_;
  const float horizon=std::clamp(2.5f*velocity.norm()+2.f*safe,4.f*safe,12.f*safe);
  const float old_sight=movingSightDistance(guide,1.5f*astar->resolution_,.75f*astar->resolution_,horizon);
  const bool blocked=blockedAhead(guide,active).has_value() ||
      !frontier_manager_ptr_->observedFree(tip) || graph->parallel_bubble_astar_->lidar_map_interface_->getDisToOcc(tip)<active+.011;
  const bool new_position=!stair_transit_origin_ || body.z()>stair_transit_origin_->z()+.6f*safe ||
      (body-*stair_transit_origin_).head<2>().norm()>10.f*safe;
  const bool lateral_ok=new_position && (blocked || old_sight<std::max(1.6f*safe,.68f*velocity.norm()));
  double winning=-std::numeric_limits<double>::infinity(),best_margin=active;
  bool lateral_won=false;
  FrontierRoute best;
  float best_rise=0.f,last=length+safe;
  for(float requested:{length,std::max(.4f*safe,length-1.8f*safe),.6f*safe,0.f}) {
    if(last-requested<.4f*safe) continue;
    last=requested;
    auto prefix=guide;
    if(requested>0.f) splitFrontierRoute(prefix,requested);else prefix.resize(1);
    if(prefix.size()>1 && !certifyPassGuide(prefix,nullptr,active,true)) continue;
    const Vector3f anchor=prefix.back();
    if(inspectStoreyOutlet(prefix,clearance)) {guide.swap(prefix);return true;}
    const auto openings=measuredRises(anchor,safe);
    if(openings.empty()) continue;
    Vector3f incoming=velocity;
    if(prefix.size()>1) {
      const auto approach=prefix.back()-prefix[prefix.size()-2];
      if(approach.norm()>.01f) incoming=std::max(.7f,velocity.norm())*approach.normalized();
    }
    double margin=-1.;
    auto climb=chooseMeasuredAscent(anchor,incoming,openings,margin);
    if(climb.size()<2 || climb.back().z()<anchor.z()+.9f*safe) continue;
    const bool alternate_observation=climb.back().z()<tip.z()+.55f*safe;
    if(alternate_observation) {
      // The old upper viewpoint can be hidden by the rail. An ordered
      // approach towards the same measured upper opening may finish at
      // similar altitude on its other side. Actual body progress owns
      // subsequent lateral substitutions, so nearby points cannot trade
      // back and forth on a flat landing.
      if(!lateral_ok || climb.back().z()<body.z()+1.05f*safe || climb.back().z()<tip.z()-.70f*safe ||
         std::none_of(openings.begin(),openings.end(),[&](const Vector3f &opening) {
           return opening.z()>body.z()+1.35f*safe && (opening-climb.back()).norm()+.8f*safe<(opening-body).norm();
         })) continue;
    } else if((climb.back()-tip).norm()<1.25f*safe) continue;
    const auto lead=FastPlannerManager::boundedGuidePrefix(climb,1.7f*safe);
    if(lead.size()<2) continue;
    const Vector3f departure=lead.back().cast<float>()-anchor;
    if(incoming.norm()>.45f && departure.dot(incoming)<.20f*departure.norm()*incoming.norm()) continue;
    const double common=std::min(active,margin<0. ? double(safe) : margin);
    prefix.insert(prefix.end(),climb.begin()+1,climb.end());
    if(!certifyPassGuide(prefix,nullptr,common,true)) continue;
    const float rise=prefix.back().z()-body.z();
    const float sight=movingSightDistance(prefix,1.5f*astar->resolution_,.75f*astar->resolution_,horizon);
    if(alternate_observation && !blocked && sight<old_sight+.5f*safe) continue;
    const double duration=estimateTransitTime(prefix,velocity,.8f*safe,limits->maxVelMag,limits->maxAccMag,limits->maxJerkMag);
    const double score=sight+1.4*safe*rise/(.6+duration)+.30*std::min(rise,3.5f*safe)+.25*common;
    if(score>winning) {winning=score;best=std::move(prefix);best_margin=common;best_rise=rise;lateral_won=alternate_observation;}
  }
  if(best.empty()) return false;
  const Vector3f goal=best.back();
  if(lateral_won) {
    stair_transit_origin_=body;
    ROS_INFO("EPIC reconnected an intermediate observation beside the obstructed stair sightline");
  }
  landing_intent_=goal;landing_closest_=(body-goal).norm();landing_progress_stamp_=pass_scan_stamp_;
  level_passage_=false;reverse_pass_=false;passage_intent_.reset();
  setMeasuredGoal(body,best.back()-best[best.size()-2],best,best_margin);
  guide.swap(best);
  ROS_INFO("EPIC joined a measured stair approach with a longer moving sightline, rise %.2fm",best_rise);
  return true;
}

bool FastExplorationManager::extendObservedAscent(FrontierRoute &suffix, double clearance, bool approaching_rise) {
  if(outlet_commitment_) return false;
  if (!floor_by_floor_ || suffix.size()<2) return false;
  const auto graph=planner_manager_->topo_graph_;
  const auto astar=graph->parallel_bubble_astar_;
  const float safe=astar->safe_distance_;
  const Vector3f body=suffix.front(),tip=suffix.back();
  if (tip.z()<body.z()-.35f*safe || !frontier_manager_ptr_->observedFree(tip)) return false;
  if(level_passage_) {
    if((level_guide_intent_-tip).norm()>2.5f*safe) return false;
    const bool active_landing=onMeasuredStairLanding(suffix);
    if(!active_landing && ((!approaching_rise && !floor_trace_.climbingAt(tip,safe)) ||
        tip.z()<body.z()+.35f*safe || (approaching_rise && !readyToPreviewAscent(body,tip)))) return false;
  }
  const auto candidates=measuredRises(tip,safe);
  double shape_margin=-1.;
  const Vector3f departure=tip-suffix[suffix.size()-2];
  const auto speed=planner_manager_->local_data_.curr_vel_.norm();
  const Vector3f velocity=departure.norm()>.01f ? (float(speed)*departure.normalized()).eval() : Vector3f::Zero();
  auto shape=chooseMeasuredAscent(tip,velocity,candidates,shape_margin);
  if(shape.size()>1 && shape.back().z()>tip.z()+.6f*safe) {
    auto joined=suffix;
    if(shape_margin>=(clearance<0.?safe:clearance) || certifyPassGuide(joined,nullptr,clearance)) {
      joined.insert(joined.end(),shape.begin()+1,shape.end());
      const double joined_margin=std::min(clearance<0.?safe:clearance,shape_margin);
      if(certifyPassGuide(joined,nullptr,joined_margin)) {
        level_passage_=false;
        if(approaching_rise) floor_trace_.startingAscent(body,safe);
        setMeasuredGoal(body,joined.back()-tip,joined,joined_margin);
        suffix.swap(joined);
        return true;
      }
    }
  }
  const auto starts=passSearchStarts(tip);
  size_t tried=0;
  for (const auto &goal : candidates) {
    if (goal.z()<tip.z()+.75f*safe || (goal-tip).head<2>().norm()>10.f*safe ||
        (goal-body).norm()<1.5f*safe) continue;
    double margin=-1.;
    auto continuation=connectPassRoute(tip,starts,goal,.018);
    if (continuation.empty()) {
      margin=narrowPassClearance();
      if (astar->lidar_map_interface_->getDisToOcc(tip)>margin+.012f)
        continuation=observationStep(tip,{tip},goal,margin);
    }
    if (!continuation.empty() && continuation.back().z()>tip.z()+.45f*safe &&
        (continuation.back()-tip).norm()>1.25f*safe) {
      auto joined=suffix;
      const float old_margin=clearance<0. ? safe : float(clearance);
      const float new_margin=margin<0. ? safe : float(margin);
      if (new_margin>=old_margin || certifyPassGuide(joined,nullptr,clearance)) {
        joined.insert(joined.end(),continuation.begin()+1,continuation.end());
        const float joined_margin=std::min(old_margin,new_margin);
        if (certifyPassGuide(joined,nullptr,joined_margin)) {
          const Vector3f end=joined.back();
          level_passage_=false;
          if(approaching_rise) floor_trace_.startingAscent(body,safe);
          setMeasuredGoal(body,end-tip,joined,joined_margin);
          if ((goal-end).norm()<safe) {
            landing_intent_=goal;
            landing_closest_=(body-goal).norm();
            landing_progress_stamp_=pass_scan_stamp_;
          }
          suffix.swap(joined);
          ROS_INFO("EPIC extended witnessed ascent %.2fm in height before the stair endpoint",
              double(end.z()-tip.z()));
          return true;
        }
      }
    }
    if (++tried>=3) break;
  }
  return false;
}

bool FastExplorationManager::continueAcrossJunction(const FrontierRoute &remaining, const Vector3f &velocity,
                                                    FrontierRoute &guide) {
  if (!floor_by_floor_ || outlet_commitment_ || remaining.size() < 2) return false;
  const float safe = planner_manager_->topo_graph_->parallel_bubble_astar_->safe_distance_;
  const float speed = velocity.head<2>().norm();
  if (speed < safe || std::abs(velocity.z()) > .32f*speed) return false;
  const Vector3f axis(velocity.x()/speed, velocity.y()/speed, 0.f);
  const auto &body = remaining.front();
  // Any sideways observation can be deferred at a junction. The new
  // branch must itself have a continuous observed forward lane extending
  // past the bend. A closed wall or a genuinely bending corridor fails it.
  float turn_at = 0.f;
  bool turns = false;
  for (const auto &p : remaining) {
    const Vector3f delta = p-body;
    const float along = delta.dot(axis);
    if (std::abs(delta.z()) > 1.3f*safe) return false;
    if (along > 15.f*safe) return false;
    if ((delta-along*axis).head<2>().norm() > 1.3f*safe+.18f*std::max(0.f,along)) {
      turns = true;
      break;
    }
    turn_at = std::max(turn_at,along);
  }
  if (!turns) {
    // The bounded prefix on the way back can end at the junction before
    // its turn towards the entrance appears in the published guide.
    const Vector3f after=level_guide_intent_-remaining.back();
    turns=after.head<2>().norm()>2.f*safe && (after-after.dot(axis)*axis).head<2>().norm()>.6f*after.head<2>().norm();
  }
  if (!turns) return false;
  auto select=[&](FrontierRoute &route,const Vector3f &target,double margin) {
    rememberPassage(level_guide_intent_);
    reverse_pass_=false;
    level_passage_=true;
    passage_intent_=target;
    passage_closest_=(target-body).norm();
    passage_progress_stamp_=pass_scan_stamp_;
    level_guide_intent_=target;
    survey_beyond_level_goal_=true;
    setMeasuredGoal(body,route.back()-route[route.size()-2],route,margin);
    guide.swap(route);
    ROS_INFO("EPIC continuing through surveyed junction into forward branch, %.2fm ahead", double((target-body).dot(axis)));
  };
  {
    const float range=std::min(float(level_preview_distance_),turn_at+12.f*safe);
    for (float margin:{safe,narrowPassClearance(),narrowPassClearance(true)}) {
      auto route=observedLevelLane(body,axis,range,margin);
      if (route.size()<2 || (route.back()-body).dot(axis)<turn_at+3.5f*safe ||
          frontier_manager_ptr_->recentlySurveyed(route.back(),3.6f*safe,1.5f*safe,.4f*safe)) continue;
      const Vector3f target=route.back();
      select(route,target,margin);
      return true;
    }
  }
  std::vector<std::pair<float,Vector3f>> choices;
  for (const auto &branch : pending_passages_) {
    const auto delta = branch.goal-body;
    const float along = delta.dot(axis);
    if (branch.next_attempt>pass_scan_stamp_ || std::abs(delta.z())>1.5f*safe ||
        along<turn_at+3.5f*safe || along>.95f*level_preview_distance_ ||
        (delta-along*axis).head<2>().norm()>1.7f*safe+.10f*along ||
        !branch.hasNewEvidence(frontier_manager_ptr_->nearbyFreeCells(branch.goal,3.f*safe)) ||
        frontier_manager_ptr_->recentlySurveyed(branch.goal,3.6f*safe,1.5f*safe,.4f*safe) ||
        frontier_manager_ptr_->inspectedPassage(branch.goal,safe)) continue;
    choices.emplace_back(along-(delta-along*axis).norm(), branch.goal);
  }
  std::sort(choices.begin(),choices.end(),[](const auto &a,const auto &b){return a.first>b.first;});
  if (choices.empty()) return false;
  const auto starts = passSearchStarts(body);
  for (size_t i=0; i<std::min<size_t>(2,choices.size()); ++i) {
    const Vector3f target=choices[i].second;
    double margin=-1.;
    auto candidate=forwardLevelPrefix(approachPassGoal(body,starts,target,margin),axis,level_preview_distance_,safe);
    if (candidate.size()<2 || (candidate.back()-body).dot(axis)<turn_at+2.f*safe ||
        !certifyPassGuide(candidate,nullptr,margin)) continue;
    select(candidate,target,margin);
    return true;
  }
  return false;
}

FrontierRoute FastExplorationManager::repairFloorReturn(const FrontierRoute &track,double &clearance) const {
  if(track.size()<2) return {};
  const auto graph=planner_manager_->topo_graph_;
  const auto astar=graph->parallel_bubble_astar_;
  const auto map=astar->lidar_map_interface_;
  const float safe=astar->safe_distance_;
  if(map->getDisToOcc(track.back())<graph->bubble_min_radius_+.04f) return {};
  const std::array<float,3> margins{safe,narrowPassClearance(),narrowPassClearance(true)};
  for(float margin:margins) {
    auto original=track;
    if(certifyPassGuide(original,nullptr,margin)) {clearance=margin;return original;}
  }
  const auto room=[&](const Vector3f &p) {
    return map->IsInBox(p) && p.z()>=astar->flight_min_z_ && p.z()<=astar->flight_max_z_ &&
           frontier_manager_ptr_->observedFree(p) ? float(map->getDisToOcc(p)) : 0.f;
  };
  // A single new return from a handrail can cut a long previously flown
  // corridor into short terminal goals. Search beside that centreline all
  // the way to its original visible endpoint before truncating the track.
  for(float margin:margins) {
    const auto clear=[&](const Vector3f &a,const Vector3f &b) {
      FrontierRoute segment{a,b};
      if(!observedPass(segment)) return false;
      if(room(a)+room(b)>(b-a).norm()+2.f*margin+.025f) return true;
      return certifyPassGuide(segment,nullptr,margin,true);
    };
    auto lane=repairObservedBend(track,margin,1.25f*safe,room,clear,level_preview_distance_);
    if(!certifyPassGuide(lane,nullptr,margin,true)) continue;
    clearance=margin;
    ROS_INFO("EPIC recovered a continuous measured floor-return lane, %.2fm at %.2fm clearance",
             double((lane.back()-lane.front()).norm()),double(margin));
    return lane;
  }
  return {};
}

bool FastExplorationManager::repairStairGuide(FrontierRoute &remaining,double clearance) {
  if(!floor_by_floor_ || remaining.size()<2 || !ed_->next_goal_node_ ||
     (remaining.back()-ed_->next_goal_node_->center_).norm()>.02f) return false;
  const auto graph=planner_manager_->topo_graph_;
  const auto astar=graph->parallel_bubble_astar_;
  const auto map=astar->lidar_map_interface_;
  if(map->getDisToOcc(remaining.back())<graph->bubble_min_radius_+.02f) return false;
  const float active=clearance<0. ? astar->safe_distance_ : clearance;
  const auto room=[&](const Vector3f &p) {
    if(!map->IsInBox(p) || !frontier_manager_ptr_->observedFree(p) ||
       p.z()<astar->flight_min_z_ || p.z()>astar->flight_max_z_) return 0.f;
    return float(map->getDisToOcc(p));
  };
  float last=std::numeric_limits<float>::infinity();
  for(float margin:{active,narrowPassClearance(),narrowPassClearance(true)}) {
    if(margin>=last-.01f) continue;
    last=margin;
    auto route=remaining;
    if(!certifyPassGuide(route,nullptr,margin)) {
      const auto clear=[&](const Vector3f &a,const Vector3f &b) {
        FrontierRoute edge{a,b};
        return certifyPassGuide(edge,nullptr,margin,true);
      };
      route=repairObservedBend(remaining,margin,1.15f*astar->safe_distance_,room,clear);
      if(!certifyPassGuide(route,nullptr,margin,true)) continue;
    }
    selected_pass_route_=route;
    selected_pass_clearance_=margin;
    remaining.swap(route);
    ROS_INFO("EPIC reconnected the measured stair guide around a changed scan, clearance %.2fm",margin);
    return true;
  }
  return false;
}

bool FastExplorationManager::repairLevelGuide(FrontierRoute &remaining, double remaining_length,
                                              double clearance,const Vector3f &velocity) {
  if (remaining.size() < 2) return false;
  const auto astar = planner_manager_->topo_graph_->parallel_bubble_astar_;
  const double active = clearance < 0. ? astar->safe_distance_ : clearance;
  const float safe=astar->safe_distance_;
  const Vector3f body=remaining.front(), tip=remaining.back(), delta=tip-body;
  const float horizontal=delta.head<2>().norm();
  if(extendFromAisleApproach(remaining,clearance)) return true;
  if(intermediate_observation_ && remaining_length>5.f*safe && velocity.head<2>().norm()>1.5f) {
    double margin=active;
    const auto &limits=planner_manager_->gcopter_config_;
    const float preview=2.f*safe+levelGuideRenewalDistance(velocity.norm(),limits->maxAccMag,
                                                           limits->maxJerkMag,.4,safe);
    auto next=previewMeasuredPass(remaining,velocity,preview,active,margin);
    if(next.size()>1 && certifyPassGuide(next,nullptr,margin,true)) {
      const Vector3f downstream=next.back()-next[next.size()-2];
      setMeasuredGoal(body,downstream,next,margin);
      remaining.swap(next);
      ROS_INFO("EPIC retained the measured aisle past a changed obstacle at %.2fm clearance",margin);
      return true;
    }
  }
  // A shifted obstacle can invalidate one side of an otherwise straight
  // corridor. Search lanes from the body, on both sides of that old line,
  // rather than repeatedly stopping at the last point before the obstacle.
  if (horizontal>2.f*safe && std::abs(delta.z())<safe) {
    const Vector3f axis(delta.x()/horizontal,delta.y()/horizontal,0.f);
    const bool straight=std::all_of(remaining.begin(),remaining.end(),[&](const auto &p) {
      const Vector3f offset=p-body;
      const float along=offset.dot(axis);
      return along>=-safe && along<=horizontal+safe && std::abs(offset.z())<1.25f*safe &&
          (offset-along*axis).head<2>().norm()<1.5f*safe;
    });
    const bool survey=survey_beyond_level_goal_ && (level_guide_intent_-tip).norm()<2.f*safe;
    const float reach=survey ? float(level_preview_distance_) :
        std::min(float(level_preview_distance_),std::max(horizontal,(level_guide_intent_-body).dot(axis)));
    if (straight && reach>horizontal+safe) {
      for (const float margin : {safe,narrowPassClearance(),narrowPassClearance(true)}) {
        auto route=observedLevelLane(body,axis,reach,margin);
        if (route.size()<2 || (route.back()-body).dot(axis)<horizontal+safe ||
            (survey && frontier_manager_ptr_->recentlySurveyed(route.back(),2.8f*safe,safe,.4f*safe))) continue;
        if (survey) {
          level_guide_intent_=route.back();
          passage_intent_=route.back();
          passage_closest_=(route.back()-body).norm();
          passage_progress_stamp_=pass_scan_stamp_;
        }
        setMeasuredGoal(body,axis,route,margin);
        ROS_INFO("EPIC repaired observed forward lane around updated scan, %.2fm with %.2fm clearance",
            double((route.back()-body).dot(axis)),double(margin));
        remaining.swap(route);
        return true;
      }
    }
  }
  for (float margin : {narrowPassClearance(), narrowPassClearance(true)}) {
    if (margin >= active - .01) continue;
    auto route = remaining;
    if (!certifyPassGuide(route, nullptr, margin)) continue;
    // The old centreline can still have ample physical clearance in the new
    // scan even when it no longer meets the ordinary open-passage margin.
    // Reuse the complete witnessed itinerary before selecting a closer goal.
    if (!extendLevelGuide(route, remaining_length, margin)) {
      if (route.size() < 2 || (route.back()-route.front()).norm() < astar->safe_distance_ ||
          astar->lidar_map_interface_->getDisToOcc(route.back()) < planner_manager_->topo_graph_->bubble_min_radius_ + .02f)
        continue;
      const Vector3f ahead = route.back() - route[route.size()-2];
      setMeasuredGoal(route.front(), ahead, route, margin);
    }
    remaining.swap(route);
    ROS_INFO("EPIC renewed observed narrow passage at %.2fm with %.2fm remaining", double(margin), remaining_length);
    return true;
  }
  return false;
}

FrontierRoute FastExplorationManager::observedLevelLane(const Vector3f &start, const Vector3f &heading,
                                                        float distance, float margin) const {
  const auto graph=planner_manager_->topo_graph_;
  const auto astar=graph->parallel_bubble_astar_;
  const auto map=astar->lidar_map_interface_;
  const float safe=astar->safe_distance_, stride=1.5f*safe;
  const float end_room=std::max(safe,float(graph->bubble_min_radius_))+.05f;
  if (distance<2.f*safe || !frontier_manager_ptr_->observedFree(start) ||
      map->getDisToOcc(start)<=margin+.012f) return {};
  const Vector3f side(-heading.y(),heading.x(),0.f);
  struct Sample {Vector3f p; float cost, room; int parent;};
  std::vector<Sample> samples{{start,0.f,float(map->getDisToOcc(start)),-1}};
  std::vector<int> previous{0};
  int endpoint=-1;
  for (int station=1; !previous.empty(); ++station) {
    const float along=std::min(distance,station*stride);
    std::vector<int> next;
    float best_cost=std::numeric_limits<float>::infinity();
    for(int lateral=-3; lateral<=3; ++lateral) for(int height=-1; height<=1; ++height) {
      const Vector3f p=start+along*heading+(lateral*.5f*safe)*side+Vector3f(0,0,height*.35f*safe);
      if (!map->IsInBox(p) || !frontier_manager_ptr_->observedFree(p) ||
          p.z()<astar->flight_min_z_ || p.z()>astar->flight_max_z_) continue;
      const float room=map->getDisToOcc(p);
      if (room<=margin+.012f) continue;
      int parent=-1;
      float best=std::numeric_limits<float>::infinity();
      for(int index:previous) {
        const auto &a=samples[index];
        const Vector3f delta=p-a.p;
        if (std::abs(delta.dot(side))>safe+.001f) continue;
        const float length=delta.norm();
        const float cost=a.cost+length+.20f*std::abs(delta.dot(side))+.35f*std::abs(delta.z());
        if (cost>=best) continue;
        FrontierRoute edge{a.p,p};
        if (!observedPass(edge)) continue;
        if (length>a.room+room-2.f*margin-.01f && !astar->collisionCheck_shortenPath(edge,margin)) continue;
        best=cost; parent=index;
      }
      if(parent<0) continue;
      const float cost=best+.04f*std::abs(lateral)+.05f*std::abs(height)+
          .08f*safe/std::max(.1f,std::min(room-margin,1.5f*safe));
      const int index=int(samples.size());
      samples.push_back({p,cost,room,parent});
      next.push_back(index);
      if(room>=end_room && cost<best_cost) {best_cost=cost;endpoint=index;}
    }
    if (along>=distance-.001f) break;
    previous.swap(next);
  }
  FrontierRoute route;
  for(int i=endpoint;i>=0;i=samples[i].parent) route.push_back(samples[i].p);
  std::reverse(route.begin(),route.end());
  return certifyPassGuide(route,nullptr,margin) ? route : FrontierRoute{};
}

bool FastExplorationManager::planObservedEndTurn(FrontierRoute &approach,const Vector3f &velocity,double clearance) {
  if (end_turn_ || outlet_commitment_ || !floor_by_floor_ || !level_passage_ || !survey_beyond_level_goal_ || approach.size()<2)
    return false;
  const auto graph=planner_manager_->topo_graph_;
  const auto astar=graph->parallel_bubble_astar_;
  const auto map=astar->lidar_map_interface_;
  const float safe=astar->safe_distance_;
  const Vector3f end=approach.back(),body=approach.front();
  Vector3f forward=end-body;
  const float span=forward.head<2>().norm();
  if(span<2.f*safe || span>14.f*safe || std::abs(forward.z())>.8f*safe ||
     velocity.head<2>().norm()<.6f) return false;
  forward.z()=0.f;
  forward.normalize();
  // A newly exposed recess at a landing is not a long corridor. Carrying
  // an automatic return beyond its mouth can send the aircraft all the
  // way back to the previous observation. Let the frontier selector pick
  // its exit unless a sustained approach was actually flown in this lane.
  const float flown=pass_.measuredApproach(body,forward);
  if(flown<6.f*safe) return false;
  if(forward.dot(velocity)<.75f*velocity.norm() || forward.dot(pass_.heading())<.85f ||
     !pass_.wallAhead(end,[&](const Vector3f &p) {return map->getDisToOcc(p);})) return false;
  if(interceptReturnAscent(approach,velocity,clearance)) return true;
  // The wall opposite the access doorway does not end the surveyed
  // corridor. Reaching it inbound along the storey's real departure in
  // reverse must not install a U-turn back across the same doorway.
  if(floor_trace_.returningThroughEntrance(body,end,safe)) return false;

  if(floor_trace_.inAscentFootprint(end,safe)) return false;
  if(floor_trace_.entrance() && floor_trace_.surveyedLevel(body,safe)) {
    const Vector3f portal=*floor_trace_.entrance()-body;
    const float along=portal.dot(forward);
    if(along>=-safe && along<span+2.f*safe && std::abs(portal.z())<1.8f*safe &&
       (portal-along*forward).head<2>().norm()<2.8f*safe) return false;
  }
  if(!measuredCorridorCap(approach,forward,safe,
       [&](const Vector3f &p){return frontier_manager_ptr_->observedFree(p);},
       [&](const Vector3f &p){return map->IsInBox(p)?float(map->getDisToOcc(p)):std::numeric_limits<float>::infinity();},
       [&](const Vector3f &p){return frontier_manager_ptr_->recentlySurveyed(p,2.5f*safe,1.4f*safe,.4f*safe);}))
    return false;

  // If the wall marks a corner with a fresh side corridor, survey that
  // opening first. A narrow column or unmeasured void is never a closed end.
  const Vector3f side(-forward.y(),forward.x(),0);
  for (int sign:{-1,1}) for (float angle:{.15f,.65f}) {
    const Vector3f axis=std::cos(angle)*float(sign)*side+std::sin(angle)*forward;
    bool open=true;
    for(float x=.5f*safe;x<=4.5f*safe;x+=.5f*safe) {
      const Vector3f p=end+x*axis;
      if(!frontier_manager_ptr_->observedFree(p) || map->getDisToOcc(p)<narrowPassClearance(true)) {open=false;break;}
    }
    if(open && !frontier_manager_ptr_->recentlySurveyed(end+4.5f*safe*axis,2.5f*safe,safe,.4f*safe)) return false;
  }
  // A long uninterrupted inbound flight can cross a T after visiting
  // its opposite wing. The flown distance then extends past the actual
  // next branch. Connect to the storey entrance when both the opposite
  // lane and the sideways return are witnessed on this same storey.
  const float active=clearance<0. ? safe : float(clearance);
  FrontierRoute branch;
  std::optional<Vector3f> return_junction;
  float to_branch=0.f;
  if(floor_trace_.entrance() && floor_trace_.surveyedLevel(body,safe)) {
    const Vector3f entrance=*floor_trace_.entrance();
    const auto junction=floor_trace_.returnJunction(end,body,forward,safe);
    if(junction && (end-junction->lane).dot(forward)>3.5f*safe &&
       frontier_manager_ptr_->recentlySurveyed(junction->lane,2.7f*safe,1.5f*safe,.4f*safe))
      return_junction=junction->lane;
    const Vector3f branch_goal=junction ? junction->side : entrance;
    to_branch=(end-(junction ? junction->lane : entrance)).dot(forward);
    const Vector3f joint=end-to_branch*forward;
    const float lateral=(joint-branch_goal).head<2>().norm();
    if(to_branch>3.5f*safe && to_branch<float(level_preview_distance_)+span &&
       lateral<12.f*safe && std::abs(joint.z()-branch_goal.z())<1.6f*safe &&
       frontier_manager_ptr_->recentlySurveyed(joint,2.7f*safe,1.5f*safe,.4f*safe) &&
       frontier_manager_ptr_->recentlySurveyed(joint-4.5f*safe*forward,2.7f*safe,1.5f*safe,.4f*safe) &&
       frontier_manager_ptr_->recentlySurveyed(joint-8.f*safe*forward,2.7f*safe,1.5f*safe,.4f*safe)) {
      // Plan through the upcoming intersection while the endpoint is still
      // ahead. Starting on its near side allows a smooth cut into the door.
      const Vector3f anchor=joint+std::min(1.8f*safe,.22f*to_branch)*forward;
      if(frontier_manager_ptr_->observedFree(anchor) && map->getDisToOcc(anchor)>narrowPassClearance(true)) {
        double reached_margin=-1.;
        auto link=approachPassGoal(anchor,passSearchStarts(anchor),branch_goal,reached_margin);
        if(link.size()>=2 && (link.back()-branch_goal).norm()<1.4f*safe && observedPass(link))
          branch=std::move(link);
      }
    }
  }
  for(float radius:{1.4f*safe,1.12f*safe,.90f*safe,.72f*safe,.54f*safe}) {
    std::vector<std::pair<float,Vector3f>> centers;
    for(float retreat:{0.f,.5f*safe,safe}) for(float dz:{0.f,-.32f*safe,.32f*safe})
      for(int i=-2;i<=2;++i) {
        const Vector3f offset=.55f*safe*i*side-retreat*forward+Vector3f(0,0,dz);
        float room=std::numeric_limits<float>::infinity();
        for(int j=0;j<=6 && room>narrowPassClearance(true);++j) {
          const float t=float(M_PI)*j/6;
          const Vector3f p=end+offset+radius*((std::sin(t)-1.f)*forward+std::cos(t)*side);
          room=map->IsInBox(p) && frontier_manager_ptr_->observedFree(p) ?
              std::min(room,float(map->getDisToOcc(p))) : 0.f;
        }
        if(room>narrowPassClearance(true)+.012f)
          centers.emplace_back(room-.14f*offset.norm()-.12f*retreat,offset);
      }
    std::stable_sort(centers.begin(),centers.end(),[](const auto &a,const auto &b){return a.first>b.first;});
    for(size_t i=0;i<std::min<size_t>(4,centers.size());++i) for(int sign:{-1,1}) {
    const Vector3f offset=centers[i].second;
    float length=std::min(std::clamp(span+4.f*safe,7.f*safe,14.f*safe),span+flown-2.f*safe);
    if(!branch.empty()) length=std::min(length,(end-branch.front()).dot(forward));
    for(float margin:{safe,narrowPassClearance(),narrowPassClearance(true)}) {
      if(margin>active+.01f) continue;
      auto loop=observedEndTurn(approach,forward,radius,astar->resolution_,length,sign,
          [&](const Vector3f &a,const Vector3f &b) {
            FrontierRoute edge{a,b};
            return observedPass(edge) && astar->collisionCheck_shortenPath(edge,margin);
          },offset);
      if(loop.size()<3 || map->getDisToOcc(loop.back())<graph->bubble_min_radius_+.05f ||
         !certifyPassGuide(loop,nullptr,margin,true)) continue;
      // Complete the ordered turn with a known way back. Use only the part
      // of our own flight on this floor which still connects to the return
      // lane; its first bend is a new junction decision. A ray-confirmed
      // turn must not create an artificial stopping point halfway down
      // the very same previously flown corridor.
      const auto trail=floor_trace_.returnPath(body,level_preview_distance_,safe);
      auto tail=observedReturnLane(loop.back(),-forward,trail,safe,
          [&](const Vector3f &a,const Vector3f &b) {
            FrontierRoute edge{a,b};
            return observedPass(edge) && astar->collisionCheck_shortenPath(edge,margin);
          },[&](const Vector3f &p) {return map->getDisToOcc(p)>graph->bubble_min_radius_+.05f;});
      // The floor-return trail can be shortened when the aircraft crosses
      // itself. The length of the straight approach actually flown remains
      // independent evidence of how far this return lane may be extended.
      // Stop at its origin so a turnaround cannot pull us past the junction.
      float witnessed=std::min(float(level_preview_distance_),
          (loop.back()-body).dot(forward)+flown-safe);
      if(return_junction) witnessed=std::min(witnessed,(loop.back()-*return_junction).dot(forward));
      const float old_reach=tail.empty() ? 0.f : (loop.back()-tail.back()).dot(forward);
      bool returned_to_branch=false;
      if(!branch.empty()) {
        FrontierRoute joined{loop.back()};
        if((loop.back()-branch.front()).norm()>.02f) joined.push_back(branch.front());
        joined.insert(joined.end(),branch.begin()+1,branch.end());
        if(certifyPassGuide(joined,nullptr,margin)) {
          tail=std::move(joined);
          returned_to_branch=true;
        } else {
          // Keep only the certified part towards the junction if its
          // downstream edge has changed since it was measured.
          const float upper=std::max(0.f,(loop.back()-branch.front()).dot(forward));
          if(old_reach>upper+.1f*safe) tail=observedLevelLane(loop.back(),-forward,upper,margin);
        }
      }
      if(branch.empty() && witnessed>old_reach+2.5f*safe) {
        auto lane=observedLevelLane(loop.back(),-forward,witnessed,margin);
        FrontierRoute accepted{loop.back()};
        for(size_t j=1;j<lane.size();++j) {
          const Vector3f edge=lane[j]-lane[j-1];
          const int samples=std::max(1,int(std::ceil(edge.norm()/safe)));
          bool flown_here=true;
          for(int k=1;k<=samples && flown_here;++k) {
            const Vector3f q=lane[j-1]+float(k)/samples*edge;
            flown_here=frontier_manager_ptr_->recentlySurveyed(q,2.5f*safe,1.5f*safe,.4f*safe);
          }
          if(!flown_here) break;
          accepted.push_back(lane[j]);
        }
        if(accepted.size()>1 && (loop.back()-accepted.back()).dot(forward)>old_reach+safe)
          tail=std::move(accepted);
      }
      if(tail.size()>1) {
        loop.insert(loop.end(),tail.begin()+1,tail.end());
        ROS_INFO("EPIC joined %.2fm of previously flown, reobserved corridor after the turnaround",
                 double((tail.back()-tail.front()).norm()));
      }
      end_turn_=EndTurn{end+offset,forward,false};
      survey_beyond_level_goal_=false;
      level_guide_intent_=branch.empty()?loop.back():*floor_trace_.entrance();
      // The published lane ends at today's visibility horizon. The observed
      // junction remains the objective when it lies farther along the return.
      if(branch.empty() && return_junction && (loop.back()-*return_junction).dot(forward)>1.6f*safe) {
        level_guide_intent_=*return_junction;
        ROS_INFO("EPIC retained the surveyed junction beyond the current return guide, %.2fm ahead",
                 double((loop.back()-*return_junction).dot(forward)));
      }
      if(returned_to_branch) ROS_INFO("EPIC joined the measured storey junction after the dead-end turn");
      passage_intent_.reset();
      reverse_pass_=false;
      setMeasuredGoal(body,-forward,loop,margin);
      approach.swap(loop);
      ROS_INFO("EPIC planned an observed dead-end turn through the farthest point, radius %.2f, clearance %.2f",radius,margin);
      return true;
    }
    }
  }
  return false;
}

bool FastExplorationManager::extendNewPass(FrontierRoute &guide,double length,double margin) {
  const bool original_return=reverse_pass_;
  bool connected=false;
  if(level_passage_) {
    if(onMeasuredStairLanding(guide))
      connected=extendThroughLevelOutlet(guide,margin) || extendObservedAscent(guide,margin,true);
    if(!connected) connected=extendLevelGuide(guide,length,margin,true) || extendThroughPassage(guide,margin);
  } else if(endingInRise(guide)) {
    connected=extendThroughLevelOutlet(guide,margin) || extendObservedAscent(guide,margin,true);
  }
  // Adding a downstream continuation does not change the decision that
  // explicitly permitted leaving a blind pocket in the other direction.
  if(connected) reverse_pass_=original_return || reverse_pass_;
  return connected;
}

bool FastExplorationManager::extendFromAisleApproach(FrontierRoute &remaining,double clearance) {
  if(!floor_by_floor_ || !level_passage_ || !intermediate_observation_ || outlet_commitment_ ||
     end_turn_ || remaining.size()<2 || pass_scan_stamp_-last_aisle_bypass_scan_<.19) return false;
  const auto astar=planner_manager_->topo_graph_->parallel_bubble_astar_;
  const float safe=astar->safe_distance_,active=clearance<0. ? safe : clearance;
  const Vector3f body=remaining.front(),tip=remaining.back(),delta=tip-body;
  const float distance=delta.head<2>().norm();
  if(distance<2.4f*safe || distance>18.f*safe || std::abs(delta.z())>1.15f*safe) return false;
  const Vector3f axis=Vector3f(delta.x(),delta.y(),0.f)/distance;
  if(pass_.measuredApproach(body,axis)<3.8f*safe) return false;
  float before=0.f,length=0.f;
  for(size_t i=1;i<remaining.size();++i) {
    const Vector3f offset=remaining[i]-body;
    const float along=offset.dot(axis);
    if(along<before-.38f*safe || (offset-along*axis).head<2>().norm()>1.45f*safe ||
       std::abs(offset.z())>1.25f*safe) return false;
    before=std::max(before,along);length+=(remaining[i]-remaining[i-1]).norm();
  }
  const Vector3f onward=level_guide_intent_-tip;
  const float along=onward.dot(axis);
  const bool survey=survey_beyond_level_goal_ && std::abs(along)<1.3f*safe &&
      (onward-along*axis).norm()<1.8f*safe;
  if(!survey && (along<2.f*safe || (onward-along*axis).norm()>2.f*safe+.10f*along)) return false;
  const float horizon=std::min(float(level_preview_distance_),
      survey ? distance+12.f*safe : distance+along);
  if(horizon<distance+1.8f*safe) return false;
  last_aisle_bypass_scan_=pass_scan_stamp_;
  // A transit observation at the edge of a narrow aisle can lose its
  // clearance as we approach. Seek the same onward lane from an earlier
  // point before that observation becomes a terminal target.
  for(float cut:{3.f*safe,6.f*safe}) {
    auto prefix=remaining;
    if(length<=cut) prefix.resize(1);else splitFrontierRoute(prefix,length-cut);
    const Vector3f anchor=prefix.back();
    const float available=horizon-(anchor-body).dot(axis);
    for(float margin:{safe,narrowPassClearance(),narrowPassClearance(true)}) {
      auto lane=observedLevelLane(anchor,axis,available,margin);
      if(lane.size()<2 || (lane.back()-tip).dot(axis)<1.8f*safe ||
         (survey && frontier_manager_ptr_->recentlySurveyed(lane.back(),2.8f*safe,safe,.4f*safe))) continue;
      const float common=std::min(active,margin);
      auto joined=prefix;
      joined.insert(joined.end(),lane.begin()+1,lane.end());
      if(!certifyPassGuide(joined,nullptr,common,true)) continue;
      if(survey) {
        level_guide_intent_=joined.back();passage_intent_=joined.back();
        passage_closest_=(joined.back()-body).norm();passage_progress_stamp_=pass_scan_stamp_;
      }
      const float gain=(joined.back()-tip).dot(axis);
      setMeasuredGoal(body,axis,joined,common);
      remaining.swap(joined);
      ROS_INFO("EPIC continued %.2fm beyond a limiting aisle observation from its measured approach",gain);
      return true;
    }
    if(prefix.size()==1) break;
  }
  return false;
}

bool FastExplorationManager::extendLevelGuide(FrontierRoute &remaining,
                                              double remaining_length,double clearance,bool new_departure) {
  return extendFromLevelTip(remaining,remaining_length,clearance,new_departure) ||
         extendFromAisleApproach(remaining,clearance);
}

bool FastExplorationManager::extendFromLevelTip(FrontierRoute &remaining,
                                              double remaining_length, double clearance, bool new_departure) {
  const auto graph = planner_manager_->topo_graph_;
  const auto astar = graph->parallel_bubble_astar_;
  const float safe = astar->safe_distance_;
  const auto &active = new_departure ? remaining : ed_->path_next_goal_;
  if (remaining.size() < 2 || active.size() < 2 || level_preview_distance_ <= remaining_length)
    return false;

  const Vector3f tip = remaining.back();
  size_t before = active.size() - 1;
  while (before > 0 && (tip - active[before]).head<2>().norm() < 3.f * safe) --before;
  Vector3f forward = tip - active[before];
  const float horizontal = forward.head<2>().norm();
  if (horizontal < safe || std::abs(forward.z()) > .3f * horizontal) return false;
  forward.z() = 0.f;
  forward /= horizontal;
  const Vector3f approach = tip - remaining.front();
  if (approach.dot(forward) < .85f * approach.norm()) return false;

  if(outlet_commitment_ && (outlet_commitment_->goal-remaining.front()).dot(forward)<=0.f) return false;
  const Vector3f to_intent = level_guide_intent_ - tip;
  const float along_intent=to_intent.dot(forward);
  const bool same_section=std::abs(along_intent)<1.25f*safe &&
      (to_intent-along_intent*forward).norm()<1.8f*safe &&
      observedPass({tip,level_guide_intent_}) && connectedAcrossPass(tip,level_guide_intent_);
  const bool survey_ahead=survey_beyond_level_goal_ && (to_intent.norm()<=safe || same_section);
  double available = level_preview_distance_ - remaining_length;
  if (!survey_ahead) available = std::min(available, double(to_intent.dot(forward)));
  if (available < 2. * safe) return false;
  const float step = astar->resolution_;
  const float endpoint_room = std::max(safe, float(graph->bubble_min_radius_)) + .05f;
  const auto map = astar->lidar_map_interface_;
  const float active_margin = clearance < 0. ? safe : float(clearance);
  float previous_margin = std::numeric_limits<float>::infinity();
  // Try the same clearances used for approaching a measured bottleneck, while
  // the aircraft still has braking distance on its current guide.
  for (const float margin : {safe, narrowPassClearance(), narrowPassClearance(true)}) {
    if (margin >= previous_margin - .01f) continue;
    previous_margin = margin;
    const bool narrow = margin < safe - .01f;
    if (narrow && (!frontier_manager_ptr_->observedFree(tip) ||
                   map->getDisToOcc(tip) <= margin + .012f)) continue;

    Vector3f end = tip;
    for (double length = step; length <= available; length += step) {
      const Vector3f sample = tip + forward * float(length);
      if (!map->IsInBox(sample) || !frontier_manager_ptr_->observedFree(sample)) break;
      const double room = map->getDisToOcc(sample);
      if (room <= margin + .011) break;
      if (room >= endpoint_room) end = sample;
    }
    FrontierRoute extra{tip, end};
    if ((end-tip).squaredNorm() < step*step*.25f ||
        !observedPass(extra) || !astar->collisionCheck_shortenPath(extra, margin)) extra.resize(1);
    double gain = (extra.back()-tip).dot(forward);
    if (gain < std::min(available-safe, std::max(8.*safe,remaining_length))) {
      auto lane=observedLevelLane(tip,forward,float(available),margin);
      if (lane.size()>1 && (lane.back()-tip).dot(forward)>gain+safe) {
        extra=std::move(lane);
        gain=(extra.back()-tip).dot(forward);
      }
    }
    if (gain < std::min(available, std::max(6.*safe, remaining_length))) {
      // The dense search is local. A target inside its search sphere lets it
      // finish promptly when a short lateral offset clears the obstruction.
      const double reach = narrow ? std::min(available, double(narrowObservationRadiusFactor * margin)) : available;
      const Vector3f target = tip + forward * float(reach);
      const auto starts = narrow ? FrontierRoute{tip} : passSearchStarts(tip);
      auto tail = forwardLevelPrefix(observationStep(tip, starts, target, narrow ? margin : -1.),
                                     forward, available, safe);
      if (tail.size() >= 2 && (tail.back()-tip).dot(forward) > gain+safe &&
          map->getDisToOcc(tail.back()) >= endpoint_room && certifyPassGuide(tail, nullptr, margin)) {
        extra = std::move(tail);
        gain = (extra.back()-tip).dot(forward);
      }
    }
    end = extra.back();
    if (gain < 2.*safe || (survey_ahead &&
        frontier_manager_ptr_->recentlySurveyed(end, 2.8f*safe, safe, .4f*safe))) continue;
    auto guide = remaining;
    if (margin < active_margin && !certifyPassGuide(guide, nullptr, clearance)) return false;
    guide.insert(guide.end(), extra.begin()+1, extra.end());
    const double joined_margin = narrow ? std::min(active_margin, margin) : clearance;
    if (!certifyPassGuide(guide, nullptr, joined_margin)) continue;

    if (survey_ahead) {
      level_guide_intent_ = end;
      passage_intent_ = end;
      passage_closest_ = (end - remaining.front()).norm();
      passage_progress_stamp_ = pass_scan_stamp_;
    }
    setMeasuredGoal(remaining.front(), forward, guide, joined_margin);
    if (narrow)
      ROS_INFO("EPIC verified narrow level continuation: %.2fm gain, %.2fm remaining, %.2fm clearance",
               gain, remaining_length, double(margin));
    remaining.swap(guide);
    return true;
  }
  return false;
}

float FastExplorationManager::narrowPassClearance(bool fine) const {
  const auto astar=planner_manager_->topo_graph_->parallel_bubble_astar_;
  const auto &cfg=planner_manager_->gcopter_config_;
  const double reserve=astar->resolution_*(fine ? .75 : 1.);
  return std::min(double(astar->safe_distance_),
      std::max(double(cfg->dilateRadiusSoft),cfg->dilateRadiusHard+cfg->startRecoveryMargin+reserve));
}

FrontierRoute FastExplorationManager::measuredNarrowPass(const Vector3f &body,bool rising,double &clearance) const {
  const auto graph=planner_manager_->topo_graph_;
  const float normal=graph->parallel_bubble_astar_->safe_distance_;
  const float room=planner_manager_->lidar_map_interface_->getDisToOcc(body);
  for (float narrow : {narrowPassClearance(),narrowPassClearance(true)}) {
    if (narrow+.01f>=normal || room<=narrow+.012f) continue;
    auto route=frontier_manager_ptr_->levelPassage(body,pass_.heading(),narrow,[&](const Vector3f &p) {
      return std::any_of(blocked_points_.begin(),blocked_points_.end(),[&](const auto &b){
        return (p-b.first).norm()<.7f*normal;
      });
    },std::max(normal+.08f,float(graph->bubble_min_radius_)+.04f),rising,nullptr,true);
    if (rising ? certifyStairStep(route,narrow) : certifyPassGuide(route,nullptr,narrow)) {
      clearance=narrow;
      return route;
    }
  }
  return {};
}

FrontierRoute FastExplorationManager::approachPassGoal(const Vector3f &body,const FrontierRoute &starts,
                                                      const Vector3f &goal,double &clearance) const {
  clearance=-1.;
  auto route=connectPassRoute(body,starts,goal,.035);
  if (!route.empty()) return route;
  route=observationStep(body,starts,goal);
  const float gap=(goal-body).norm();
  float best=route.empty() ? gap : (goal-route.back()).norm();
  const auto astar=planner_manager_->topo_graph_->parallel_bubble_astar_;
  for (float narrow : {narrowPassClearance(),narrowPassClearance(true)}) {
    if (best<.65f || gap-best>3.f*astar->safe_distance_) break;
    if (narrow+.01f>=astar->safe_distance_) continue;
    auto departures=starts;
    if (astar->lidar_map_interface_->getDisToOcc(body)>narrow+.012f) departures={body};
    else if(departures.empty()) departures=passSearchStarts(body,narrow);
    auto candidate=observationStep(body,departures,goal,narrow);
    if (candidate.empty() || (goal-candidate.back()).norm()+.35f>=best) continue;
    best=(goal-candidate.back()).norm();
    clearance=narrow;
    route.swap(candidate);
  }
  if (clearance>=0.) ROS_INFO("EPIC measured bottleneck from (%.2f %.2f %.2f) to (%.2f %.2f %.2f), certified clearance %.2fm",
      body.x(),body.y(),body.z(),route.back().x(),route.back().y(),route.back().z(),clearance);
  return route;
}

int FastExplorationManager::setMeasuredGoal(const Vector3f &body, const Vector3f &look, const FrontierRoute &route,double clearance) {
  const auto graph = planner_manager_->topo_graph_;
  frontier_manager_ptr_->selectInspectionTask(route.back(),graph->parallel_bubble_astar_->safe_distance_);
  selected_pass_route_ = route;
  selected_pass_clearance_ = clearance;
  intermediate_intent_.reset();
  unresolved_rise_origin_.reset();
  intermediate_observation_ = true;
  const auto target = std::make_shared<TopoNode>();
  target->is_viewpoint_ = true;
  target->center_ = route.back();
  target->yaw_ = look.head<2>().norm() > .1f ? std::atan2(look.y(), look.x()) : graph->odom_node_->yaw_;
  target->neighbors_.insert(graph->odom_node_);
  target->paths_[graph->odom_node_] = FrontierRoute(route.rbegin(), route.rend());
  ed_->global_tour_ = {body, target->center_};
  planner_manager_->local_data_.end_yaw_ = target->yaw_;
  updateGoalNode(target);
  frontier_empty_since_ = -1.;
  planner_manager_->graph_visualizer_->vizTour(ed_->global_tour_, VizColor::RED, "global");
  return SUCCEED;
}

FrontierRoute FastExplorationManager::observationStep(const Vector3f &position, const FrontierRoute &starts,
                                                      const Vector3f &goal,double clearance) const {
  const auto astar = planner_manager_->topo_graph_->parallel_bubble_astar_;
  const auto map = astar->lidar_map_interface_;
  const bool fine = clearance>=0.;
  const float normal = fine ? clearance : astar->safe_distance_;
  const bool tight = fine && clearance<narrowPassClearance()-.01f;
  const float spacing = astar->resolution_ * (tight ? .5f : fine ? 2.f/3.f : 1.f);
  auto indexOf=[&](const Vector3f &p) -> Eigen::Vector3i {
    if (fine) return (p/spacing).array().floor().cast<int>();
    Eigen::Vector3i k; astar->posToIndex(p,k); return k;
  };
  const float min_gain = normal;
  const float initial_gap = (position - goal).norm();
  const float radius=(fine?narrowObservationRadiusFactor:6.f)*normal;
  const double lower_bound=observationStepLowerBound(initial_gap,radius);
  const double tolerance=std::max(2.f*spacing,.1f*radius);
  if (starts.empty() || initial_gap <= min_gain) return {};
  struct Vertex { Vector3f p; int parent; float length; double room; };
  vector<Vertex> nodes;
  std::priority_queue<std::pair<float, int>> queue;
  std::unordered_set<Eigen::Vector3i, v3i_hash> seen;
  auto add = [&](const Vector3f &p, int parent, float distance,double room) {
    nodes.push_back({p, parent, distance,room});
    queue.emplace(-((goal - p).norm() + .3f * distance), int(nodes.size() - 1));
    seen.insert(indexOf(p));
  };
  for (const auto &start : starts) add(start, -1, (position - start).norm(),map->getDisToOcc(start));
  int best = -1;
  float best_value = initial_gap - .7f * min_gain;
  const float endpoint_margin = std::max(float(planner_manager_->topo_graph_->bubble_min_radius_), normal) + .01f;
  // The nearest-obstacle distance also bounds how close a valid endpoint
  // can be to an obstructed goal. Avoid searching an impossible gain band.
  const double minimum_gap=std::max(0.,double(endpoint_margin)-map->getDisToOcc(goal));
  if(minimum_gap>double(initial_gap-min_gain)+1.e-4) return {};
  while (!queue.empty() && nodes.size() < (tight ? 30000u : fine ? 18000u : 1800u)) {
    const int index = queue.top().second;
    queue.pop();
    const auto node = nodes[index];
    const float gap = (goal - node.p).norm();
    const float value = gap + .12f * node.length;
    if (initial_gap - gap >= min_gain && (node.p - position).norm() >= min_gain &&
        value < best_value && node.room > endpoint_margin) {
      best = index;
      best_value = value;
      // A remote observation need not exhaust the whole local grid after
      // finding an almost best possible, fully measured advance. This
      // lower bound includes the distance from the actual body to the seed.
      if(gap<min_gain || best_value<=lower_bound+tolerance) break;
    }
    const Eigen::Vector3i center=indexOf(node.p);
    for (int axis = 0; axis < 3; ++axis) for (int sign : {-1, 1}) {
      Eigen::Vector3i next = center;
      next[axis] += sign;
      if (seen.count(next)) continue;
      Vector3f p;
      if (fine) p=(next.cast<float>().array()+.5f)*spacing;
      else astar->IndexToPos(p, next);
      if ((p - position).norm() > radius || p.z() < astar->flight_min_z_ || p.z() > astar->flight_max_z_ ||
          (pass_height_margin_ >= 0. && (p.z() < std::min(position.z(), goal.z()) - pass_height_margin_ ||
                                        p.z() > std::max(position.z(), goal.z()) + pass_height_margin_))) continue;
      FrontierRoute edge{node.p, p};
      if(fine && !frontier_manager_ptr_->observedFree(p)) {
        seen.insert(next);
        continue;
      }
      const double room=map->getDisToOcc(p);
      if(fine && room<=normal+.012f) {
        seen.insert(next);
        continue;
      }
      const float edge_length=(p-node.p).norm();
      const bool overlapping=clearanceBallsOverlap(edge_length,node.room,room,normal);
      if(!observedPass(edge) || (!overlapping && !astar->collisionCheck_shortenPath(edge,normal))) continue;
      add(p,index,node.length+edge_length,room);
    }
  }
  if (best < 0) return {};
  FrontierRoute route;
  for (int i = best; i >= 0; i = nodes[i].parent) route.push_back(nodes[i].p);
  if ((route.back() - position).norm() > .01f) route.push_back(position);
  std::reverse(route.begin(), route.end());
  return certifyPassGuide(route,nullptr,clearance) ? route : FrontierRoute{};
}

bool FastExplorationManager::certifyPassGuide(FrontierRoute &route, const char **reason, double clearance, bool keep_order) const {
  if (route.size() < 2 || !withinEndpointHeights(route, route.front(), route.back(), pass_height_margin_)) {
    if (reason) *reason = route.size() < 2 ? "empty route" : "endpoint height band";
    return false;
  }
  const auto astar = planner_manager_->topo_graph_->parallel_bubble_astar_;
  const double normal = clearance < 0. ? astar->safe_distance_ : clearance;
  auto shortened = route;
  if (!keep_order && !end_turn_ && astar->collisionCheck_shortenPath(shortened,normal) && observedPass(shortened)) {
    route.swap(shortened);
    return true;
  }
  // A geometric shortcut can cross unobserved air on the inside of a bend.
  // Keep the supplied edges if they are individually witnessed and clear.
  if (!observedPass(route)) {
    if (reason) *reason = "unobserved, outside bounds or recently blocked segment";
    return false;
  }
  if (reason) *reason = "route clearance";
  const auto &config = planner_manager_->gcopter_config_;
  const bool recovering = config->startCorridorRecovery &&
      astar->lidar_map_interface_->getDisToOcc(route.front()) <= normal + .011;
  for (size_t i = 1; i < route.size(); ++i) {
    const bool departure = i == 1 && recovering;
    if (departure && ((route[1] - route[0]).norm() > 2. * normal ||
        astar->lidar_map_interface_->getDisToOcc(route[1]) <= normal + .01)) return false;
    FrontierRoute edge{route[i-1], route[i]};
    if (!astar->collisionCheck_shortenPath(edge, departure ?
        config->dilateRadiusHard + config->startRecoveryMargin : normal)) return false;
  }
  return true;
}

bool FastExplorationManager::certifyStairStep(FrontierRoute &route,double clearance) const {
  if (certifyPassGuide(route,nullptr,clearance)) return true;
  if (route.size()<3 || withinEndpointHeights(route,route.front(),route.back(),pass_height_margin_)) return false;
  // A switchback can require descending slightly to clear the underside
  // of the stairs before the ascent. Deliver a fully checked approach to
  // the turn, then observe the rising leg from there.
  const auto graph=planner_manager_->topo_graph_;
  const float safe=graph->parallel_bubble_astar_->safe_distance_;
  const auto summit=route.back();
  while (route.size()>2) {
    route.pop_back();
    if ((route.back()-route.front()).norm()<safe ||
        planner_manager_->lidar_map_interface_->getDisToOcc(route.back())<graph->bubble_min_radius_+.02f) continue;
    if (certifyPassGuide(route,nullptr,clearance)) {
      ROS_INFO("EPIC measured stair approach (%.2f %.2f %.2f) towards rising branch (%.2f %.2f %.2f)",
          route.back().x(),route.back().y(),route.back().z(),summit.x(),summit.y(),summit.z());
      return true;
    }
  }
  return false;
}

bool FastExplorationManager::selectedPassGuide(const Vector3f &position, FrontierRoute &path) const {
  if (!corridor_pass_enabled_ || selected_pass_route_.size() < 2 ||
      (selected_pass_route_.back() - ed_->next_goal_node_->center_).norm() > .01f) return false;
  auto ready = selected_pass_route_;
  const Vector3f old_start=ready.front();
  if((old_start-position).norm()<=.5f) {
    ready.front()=position;
    if(certifyPassGuide(ready,nullptr,selected_pass_clearance_)) {path.swap(ready);return true;}
  }
  // Map processing can advance the vehicle beyond the departure used by
  // the higher-level search. Project along that same ordered path and check
  // the connector, endpoint height band and whole suffix on the current scan.
  double progress=0.;
  ready=remainingFrontierRoute(selected_pass_route_,position,old_start,progress);
  if(progress<.15f || ready.size()<2 || !certifyPassGuide(ready,nullptr,selected_pass_clearance_)) return false;
  ROS_INFO("EPIC resumed the selected observation %.2fm along its measured approach",progress);
  path.swap(ready);
  return true;
}

bool FastExplorationManager::continuePassGuide(const Vector3f &position, const FrontierRoute &tail,
                                               FrontierRoute &path, const Vector3f *split_endpoint) const {
  if (!corridor_pass_enabled_ || tail.size() < 2 || (tail.front() - position).norm() > .65f ||
      (tail.back() - (split_endpoint ? *split_endpoint : ed_->next_goal_node_->center_)).norm() > .01f) return false;
  FrontierRoute ready = tail;
  if ((position - ready.front()).norm() > .01f) ready.insert(ready.begin(), position);
  else ready.front() = position;
  if (!certifyPassGuide(ready,nullptr,selected_pass_clearance_)) return false;
  path.swap(ready);
  return true;
}

bool FastExplorationManager::acceptPassDeparture(FrontierRoute &path, bool verified) {
  if (!corridor_pass_enabled_) return true;
  // A newer body pose may have required a different native connector. Apply
  // the same height and clearance checks to the exact route handed off.
  if (!verified && !certifyPassGuide(path)) return false;
  if (reverse_pass_) return true;
  const auto actual = pass_.assess(path, 0., [this](const Vector3f &a, const Vector3f &b) {
    return connectedAcrossPass(a, b);
  });
  if (actual.leg != CorridorPass::Leg::Return) return true;
  // Local reconnection can change the approach since the global graph search.
  // Allow another native connection/candidate on the next map update.
  frontier_manager_ptr_->deferViewpoint(path.back(), 2.);
  return false;
}

void FastExplorationManager::commitPass(const FrontierRoute &path) {
  if (corridor_pass_enabled_) pass_.commit(path, reverse_pass_);
  reverse_pass_ = false;
}

bool FastExplorationManager::extendTourPath(vector<Eigen::Vector3f> &path) {
  epic_ros::NodeHandle nh;double limit;
  nh.param("exploration/tour_lookahead_distance",limit,0.);
  double corner_speed;nh.param("exploration/corridor_corner_preview_speed",corner_speed,0.);
  if(corner_speed>0. && planner_manager_->local_data_.curr_vel_.norm()>corner_speed)return false;
  if(!std::isfinite(limit) || limit<=0. || limit>40. || path.size()<2 ||
      ed_->global_tour_.size()<3 || (path.back()-ed_->global_tour_[1]).norm()>.1f)return false;
  double used=0.;
  for(size_t i=1;i<path.size();++i) {
    if(!path[i].allFinite() || !path[i-1].allFinite())return false;
    used+=(path[i]-path[i-1]).norm();
  }
  if(used>=limit-.1)return false;
  auto extended=path;bool changed=false;
  for(size_t j=2;j<ed_->global_tour_.size() && used<limit-.1;++j) {
    vector<Eigen::Vector3f> segment;
    if(planner_manager_->topo_graph_->parallel_bubble_astar_->search(
         extended.back(),ed_->global_tour_[j],segment,.01,false)!=ParallelBubbleAstar::REACH_END || segment.size()<2) {
      if (corridor_pass_enabled_) continue;
      break;
    }
    if(!planner_manager_->topo_graph_->parallel_bubble_astar_->collisionCheck_shortenPath(segment) ||
        (corridor_pass_enabled_ && !observedPass(segment))) continue;
    // Defer a native tour leg which immediately reverses the current pass.
    // Its frontier remains unvisited and may be selected at the next junction.
    // Test the actual free-space route, not a bearing through a shelf.
    if (corridor_pass_enabled_) {
      if (!pass_.allowsPreview(extended, segment, reverse_pass_,
          [this](const Vector3f &a, const Vector3f &b) { return connectedAcrossPass(a, b); })) continue;
    } else if (directional_frontiers_ && extended.size()>1 &&
               frontierContinuationCost(segment,extended.back()-extended[extended.size()-2])>=300.) continue;
    auto prefix=FastPlannerManager::boundedGuidePrefix(segment,limit-used);
    if(prefix.size()<2)break;
    for(size_t i=1;i<prefix.size();++i) {
      used+=(prefix[i]-prefix[i-1]).norm();
      extended.push_back(prefix[i].cast<float>());
    }
    changed=true;
    if((extended.back()-ed_->global_tour_[j]).norm()>.05f)break;
  }
  if(!changed)return false;
  path.swap(extended);
  ROS_INFO("Native exploration preview: %.1fm guide", used);
  return true;
}

void FastExplorationManager::updateGoalNode(const TopoNode::Ptr &validated_source) {
  if (ed_->global_tour_.size() < 2)
    return;
  Eigen::Vector3f goal = ed_->global_tour_[1];
  vector<pair<TopoNode::Ptr,vector<Eigen::Vector3f>>> preserved;
  unordered_set<TopoNode::Ptr> preserved_anchors,persistent;
  vector<TopoNode::Ptr> queue;
  const auto odom_node=planner_manager_->topo_graph_->odom_node_;
  if(odom_node){persistent.insert(odom_node);queue.push_back(odom_node);}
  for(size_t i=0;i<queue.size();++i)for(const auto&next:queue[i]->neighbors_)
    if(next && !next->is_viewpoint_ && persistent.insert(next).second)queue.push_back(next);
  auto source=validated_source;
  if(!source && (ed_->next_goal_node_->center_-goal).norm()<1.e-4f)
    source=ed_->next_goal_node_;
  auto astar=planner_manager_->topo_graph_->parallel_bubble_astar_;
  auto preserve=[&](const TopoNode::Ptr &anchor,vector<Eigen::Vector3f> path) {
    if(!anchor || anchor==ed_->next_goal_node_ || !persistent.count(anchor) || path.size()<2 ||
        (path.front()-goal).norm()>.2f || (path.back()-anchor->center_).norm()>.2f)return;
    path.front()=goal;path.back()=anchor->center_;
    for(const auto&p:path)if(!p.allFinite() || p.z()<astar->flight_min_z_-1.e-5 ||
        p.z()>astar->flight_max_z_+1.e-5)return;
    bool certified;
    if (corridor_pass_enabled_ && anchor == odom_node) {
      std::reverse(path.begin(), path.end());
      certified = certifyPassGuide(path,nullptr,source==validated_source ? selected_pass_clearance_ : -1.);
      std::reverse(path.begin(), path.end());
    } else certified = astar->collisionCheck_shortenPath(path);
    if(certified && preserved_anchors.insert(anchor).second)
      preserved.emplace_back(anchor,std::move(path));
  };
  if(source && (source->center_-goal).norm()<1.e-4f) {
    for(const auto&anchor:source->neighbors_) {
      if(!anchor || anchor->is_viewpoint_)continue;
      const auto found=source->paths_.find(anchor);
      if(found!=source->paths_.end())preserve(anchor,found->second);
    }
    if(preserved.empty() && odom_node) {
      // A selected target may connect through other temporary viewpoints.
      // Materialize that ORIGINAL graph route back to its persistent prefix,
      // so deleting the temporary nodes cannot delete the selected route.
      vector<TopoNode::Ptr> chain;
      if(planner_manager_->topo_graph_->graphSearch(odom_node,source,chain,.01) && chain.size()>1) {
        size_t anchor_index=0;
        while(anchor_index+1<chain.size() && persistent.count(chain[anchor_index+1]))++anchor_index;
        vector<Eigen::Vector3f> route;bool valid=anchor_index+1<chain.size();
        for(size_t i=anchor_index;valid && i+1<chain.size();++i) {
          const auto edge=chain[i]->paths_.find(chain[i+1]);
          if(edge==chain[i]->paths_.end() || edge->second.size()<2 ||
              (edge->second.front()-chain[i]->center_).norm()>.2f ||
              (edge->second.back()-chain[i+1]->center_).norm()>.2f){valid=false;break;}
          auto part=edge->second;part.front()=chain[i]->center_;part.back()=chain[i+1]->center_;
          route.insert(route.end(),part.begin()+(route.empty()?0:1),part.end());
        }
        if(valid){std::reverse(route.begin(),route.end());preserve(chain[anchor_index],std::move(route));}
      }
    }
  }
  double goal_connection_timeout;
  epic_ros::NodeHandle nh;
  nh.param("parallel_astar/goal_connection_timeout", goal_connection_timeout, .001);
  if (!std::isfinite(goal_connection_timeout) || goal_connection_timeout <= 0.)
    throw std::invalid_argument("Goal connection timeout must be positive");


  struct PairPtrHash {
    std::size_t
    operator()(const std::pair<TopoNode::Ptr, TopoNode::Ptr> &p) const {
      return std::hash<TopoNode::Ptr>()(p.first) ^
             std::hash<TopoNode::Ptr>()(p.second);
    }
  };

  Eigen::Vector3i idx;
  planner_manager_->topo_graph_->getIndex(goal, idx);
  vector<TopoNode::Ptr> pre_nbrs;
  for (int i = -1; i <= 1; i++)
    for (int j = -1; j <= 1; j++)
      for (int k = -1; k <= 1; k++) {
        Eigen::Vector3i tmp_idx = idx;
        tmp_idx(0) = idx(0) + i;
        tmp_idx(1) = idx(1) + j;
        tmp_idx(2) = idx(2) + k;
        auto region = planner_manager_->topo_graph_->getRegionNode(tmp_idx);
        if (region) {
          for (auto &topo : region->topo_nodes_) {
            if (topo == ed_->next_goal_node_ || topo->is_viewpoint_ || preserved_anchors.count(topo))
              continue;
            pre_nbrs.emplace_back(topo);
          }
        }
      }
  if(!preserved_anchors.count(planner_manager_->topo_graph_->odom_node_) &&
      std::find(pre_nbrs.begin(),pre_nbrs.end(),planner_manager_->topo_graph_->odom_node_)==pre_nbrs.end())
    pre_nbrs.push_back(planner_manager_->topo_graph_->odom_node_);
  std::unordered_map<std::pair<TopoNode::Ptr, TopoNode::Ptr>,
                     vector<Eigen::Vector3f>, PairPtrHash>
      edge2insert;
  for(auto &edge:preserved)
    edge2insert.emplace(std::make_pair(ed_->next_goal_node_,edge.first),std::move(edge.second));
  if(!preserved.empty())ROS_INFO("Preserved %zu validated native connections for selected goal handoff",preserved.size());
  mutex edge2insert_mtx;
  omp_set_num_threads(epic_ros::planningWorkerCount());
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (auto &nbr : pre_nbrs) {
    vector<Eigen::Vector3f> path;
    int res = planner_manager_->topo_graph_->parallel_bubble_astar_->search(
        goal, nbr->center_, path, goal_connection_timeout);
    if (res == ParallelBubbleAstar::REACH_END &&
        planner_manager_->topo_graph_->parallel_bubble_astar_
            ->collisionCheck_shortenPath(path)) {
      edge2insert_mtx.lock();
      edge2insert.insert({std::make_pair(ed_->next_goal_node_, nbr), path});
      edge2insert_mtx.unlock();
    }
  }
  // Detach with the OLD position before moving the goal. Never retain edges
  // whose stored paths end at a previous TSP target when reconnection fails.
  // Native updateOdomNode intentionally retains outgoing odom edges only.
  // Such an incoming edge is absent from goal->neighbors_, so removeNode
  // alone cannot detach it when the same goal pointer moves to a new target.
  const auto odom=planner_manager_->topo_graph_->odom_node_;
  if(odom) {
    odom->neighbors_.erase(ed_->next_goal_node_);
    odom->paths_.erase(ed_->next_goal_node_);
    odom->weight_.erase(ed_->next_goal_node_);
    odom->unreachable_nbrs_.erase(ed_->next_goal_node_);
  }
  Eigen::Vector3i old_idx;
  planner_manager_->topo_graph_->getIndex(ed_->next_goal_node_->center_, old_idx);
  if (planner_manager_->topo_graph_->getRegionNode(old_idx))
    planner_manager_->topo_graph_->removeNode(ed_->next_goal_node_);
  ed_->next_goal_node_->center_ = goal;
  ed_->next_goal_node_->is_viewpoint_ = true;
  auto goal_region = planner_manager_->topo_graph_->getRegionNode(idx);
  if (goal_region) goal_region->topo_nodes_.insert(ed_->next_goal_node_);
  if (edge2insert.size() > 0) {
    for (auto &edge : edge2insert) {
      ed_->next_goal_node_->neighbors_.insert(edge.first.second);
      ed_->next_goal_node_->paths_.insert({edge.first.second, edge.second});
      double cost;
      planner_manager_->topo_graph_->parallel_bubble_astar_->calculatePathCost(
          edge.second, cost);
      ed_->next_goal_node_->weight_[edge.first.second] = cost;
      auto nbr = edge.first.second;
      nbr->neighbors_.insert(ed_->next_goal_node_);
      nbr->weight_[ed_->next_goal_node_] = cost;
      vector<Eigen::Vector3f> path = edge.second;
      std::reverse(path.begin(), path.end());
      nbr->paths_[ed_->next_goal_node_] = path;
    }
  }
}
} // namespace fast_planner
