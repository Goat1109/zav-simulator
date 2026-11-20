/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2023-12-21 21:31:51
 * @LastEditTime: 2024-03-10 11:39:33
 * @Description:
 * @
 * @Copyright (c) 2023 by ning-zelin, All Rights Reserved.
 */

#ifndef _EXPLORATION_MANAGER_H_
#define _EXPLORATION_MANAGER_H_

#include <Eigen/Eigen>
#include <epic_planner/corridor_pass.h>
#include <frontier_manager/frontier_manager.h>
#include <memory>
#include <optional>
#include <omp.h>
#include <opencv2/opencv.hpp>
#include <pointcloud_topo/graph.h>
#include <plan_manage/planner_manager.h>
#include <epic_ros2/ros.hpp>
#include <vector>
using Eigen::Vector3d;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
class EDTEnvironment;
class SDFMap;
class FastPlannerManager;
struct ExplorationParam;
struct ExplorationData;

enum EXPL_RESULT { NO_FRONTIER, FAIL, START_FAIL, SUCCEED, FRONTIER_PENDING };

class FastExplorationManager {
public:
  typedef shared_ptr<FastExplorationManager> Ptr;
  FastExplorationManager();
  ~FastExplorationManager();
  shared_ptr<ExplorationData> ed_;
  shared_ptr<ExplorationParam> ep_;
  epic_ros::Timer frontier_timer_;
  FrontierManager::Ptr frontier_manager_ptr_;
  double goal_yaw;
  bool use_velocity_direction_cost_ = false;
  bool directional_frontiers_ = false;
  bool corridor_pass_enabled_ = false;
  bool intermediate_observation_ = false;
  bool level_passage_ = false;
  double selected_pass_clearance_ = -1.;
  bool selectLevelPassage(const Eigen::Vector3f &body, bool interrupting_ascent = false);
  bool planObservedEndTurn(FrontierRoute &approach,const Eigen::Vector3f &velocity,double clearance);
  bool followingEndTurn() const {return end_turn_.has_value();}
  bool extendNewPass(FrontierRoute &guide,double length,double margin);
  double last_aisle_bypass_scan_=-1.;
  bool extendFromLevelTip(FrontierRoute &remaining,double remaining_length,double clearance,bool new_departure);
  bool extendFromAisleApproach(FrontierRoute &remaining,double clearance);
  bool extendLevelGuide(FrontierRoute &remaining, double remaining_length, double clearance,
                        bool new_departure=false);
  bool extendThroughPassage(FrontierRoute &guide,double clearance);
  bool interceptReturnAscent(FrontierRoute &guide,const Eigen::Vector3f &velocity,double clearance);
  bool followingStoreyOutlet() const {return outlet_commitment_.has_value();}
  bool inspectStoreyOutlet(FrontierRoute &prefix,double clearance);
  bool extendThroughLevelOutlet(FrontierRoute &guide,double clearance);
  FrontierRoute previewMeasuredPass(const FrontierRoute &path,const Eigen::Vector3f &velocity,
                                    float required,float active,double &certified) const;
  bool improveTransitBend(FrontierRoute &path,const Eigen::Vector3f &velocity);
  bool followsMeasuredFloor(const FrontierRoute &guide) const;
  bool onMeasuredStairLanding(const FrontierRoute &guide) const;
  bool endingInRise(const FrontierRoute &guide) const;
  bool continueBeforeStairTip(FrontierRoute &guide,const Eigen::Vector3f &velocity,double clearance);
  bool extendObservedAscent(FrontierRoute &suffix, double clearance, bool approaching_rise = false);
  FrontierRoute repairFloorReturn(const FrontierRoute &track,double &clearance) const;
  bool repairStairGuide(FrontierRoute &remaining,double clearance);
  bool repairLevelGuide(FrontierRoute &remaining, double remaining_length, double clearance, const Eigen::Vector3f &velocity);
  bool continueAcrossJunction(const FrontierRoute &remaining, const Eigen::Vector3f &velocity,
                              FrontierRoute &guide);
  double latestPassScan() const {return pass_scan_stamp_;}
  void observePass(const Eigen::Vector3f &position, double scan_stamp);
  bool acceptPassDeparture(FrontierRoute &path, bool verified);
  bool selectedPassGuide(const Eigen::Vector3f &position, FrontierRoute &path) const;
  bool continuePassGuide(const Eigen::Vector3f &position, const FrontierRoute &tail, FrontierRoute &path,
                         const Eigen::Vector3f *split_endpoint = nullptr) const;
  void commitPass(const FrontierRoute &path);
  bool observedPass(const FrontierRoute &path) const;
  std::optional<Eigen::Vector3f> blockedAhead(const FrontierRoute &remaining, double clearance = -1.) const;
  void avoidBlockedPoint(const Eigen::Vector3f &point, bool stalled);
  bool external_reselect_from_current_ = false;
  Eigen::Vector3f preferred_heading_ = Eigen::Vector3f::UnitY();
  shared_ptr<FastPlannerManager> planner_manager_;
  // ViewpointForest::Ptr vps_forest_;
  double getPathCost(TopoNode::Ptr &n1, Eigen::Vector3d v1, float &yaw1, TopoNode::Ptr &n2,
                     float &yaw2, double *initial_turn_cost = nullptr, FrontierRoute *verified_path = nullptr);
  static double initialDirectionCost(const vector<Eigen::Vector3f> &path,
                                     const Eigen::Vector3d &velocity,
                                     double weight, double yaw_rate);
  double getPathCostWithoutTopo(TopoNode::Ptr &n1, Eigen::Vector3d v1, float &yaw1, TopoNode::Ptr &n2, float &yaw2);
  void initialize(epic_ros::NodeHandle &nh, FrontierManager::Ptr frt_manager,
                  FastPlannerManager::Ptr planner_manager);
  int frontierStatus(bool screened = false);
  double frontier_empty_since_ = -1.;
  int planGlobalPath(const Vector3d &pos, const Vector3d &vel);
  void solveLHK(Eigen::MatrixXd &cost_mat, vector<int> &indices);

  void surfaceFrtCalllback(const epic_ros::TimerEvent &e);
  void goalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);
  void updateGoalNode(const TopoNode::Ptr &validated_source = nullptr);
  bool extendTourPath(vector<Eigen::Vector3f> &path);
  bool reanchorLocalPath(vector<Eigen::Vector3f> &path, const Eigen::Vector3f &boundary,
                        const Eigen::Vector3f &velocity);
private:
  bool clearLastStairTread(const Eigen::Vector3f &body);
  std::optional<Eigen::Vector3f> sampled_rise_landing_;
  CorridorPass pass_;
  FloorTrace floor_trace_;
  struct OutletCommitment {Eigen::Vector3f goal; float distance; double progress;};
  std::optional<OutletCommitment> outlet_commitment_;
  std::optional<std::pair<Eigen::Vector3f,double>> deferred_outlet_;
  void commitObservedOutlet(const Eigen::Vector3f &body,const Eigen::Vector3f &goal,float clearance);

  struct EndTurn {Eigen::Vector3f tip, forward; bool reached;};
  std::optional<EndTurn> end_turn_;
  bool readyToPreviewAscent(const Eigen::Vector3f &body,const Eigen::Vector3f &tip,const FrontierRoute *approach=nullptr) const;
  bool visibleLevelOutlet(const Eigen::Vector3f &body,const FrontierRoute *approach=nullptr) const;
  bool chose_initial_opening_ = false;
  bool reverse_pass_ = false, expand_pass_search_ = false;
  double pass_scan_stamp_ = -1.;
  double pass_height_margin_ = -1.;
  bool floor_by_floor_ = false;
  bool floor_model_supported_ = true;
  bool hasReturnWork(const Eigen::Vector3f &body,float clearance) const;
  double floor_support_stamp_ = -1.;
  double level_preview_distance_ = 0.;
  Eigen::Vector3f level_guide_intent_ = Eigen::Vector3f::Zero();
  bool survey_beyond_level_goal_ = false;
  std::optional<Eigen::Vector3f> passage_intent_;
  std::optional<Eigen::Vector3f> landing_intent_;
  double last_stair_bypass_scan_=-1.;
  std::optional<Eigen::Vector3f> stair_transit_origin_;
  float landing_closest_ = 0.f;
  double landing_progress_stamp_ = -1.;
  float passage_closest_ = 0.f;
  double passage_progress_stamp_ = -1.;
  std::optional<Eigen::Vector3f> unresolved_rise_origin_;
  size_t unresolved_rise_evidence_ = 0;
  double unresolved_rise_since_ = -1.;
  std::vector<PassageBranch> pending_passages_;
  std::optional<Eigen::Vector3f> last_branch_scan_position_;
  struct RisingQuery {Eigen::Vector3f tip;float clearance;FrontierRoute openings;};
  mutable std::vector<RisingQuery> rising_queries_;
  FrontierRoute measuredRises(const Eigen::Vector3f &tip,float clearance) const;
  void rememberPassage(const Eigen::Vector3f &goal);
  std::optional<Eigen::Vector3f> intermediate_intent_;
  FrontierRoute selected_pass_route_;
  FrontierRoute passSearchStarts(const Eigen::Vector3f &position,double margin=-1.) const;
  std::optional<int> recoverFlownTrack(const Eigen::Vector3f &body);
  FrontierRoute connectPassRoute(const Eigen::Vector3f &body, const FrontierRoute &starts,
                                 const Eigen::Vector3f &goal, double budget, const char **reason = nullptr) const;
  std::optional<int> planRisingOpening(const Eigen::Vector3f &body);
  FrontierRoute measuredClimb(const Eigen::Vector3f &body,const Eigen::Vector3f &heading,
                              const FrontierRoute &openings,double &clearance) const;
  FrontierRoute chooseMeasuredAscent(const Eigen::Vector3f &body,const Eigen::Vector3f &velocity,
                                    const FrontierRoute &openings,double &margin) const;
  FrontierRoute measuredPreviousAscent(const Eigen::Vector3f &body, double &clearance) const;
  int setMeasuredGoal(const Eigen::Vector3f &body, const Eigen::Vector3f &look, const FrontierRoute &route, double clearance = -1.);
  float narrowPassClearance(bool fine = false) const;
  FrontierRoute observedLevelLane(const Eigen::Vector3f &start, const Eigen::Vector3f &heading,
                                  float distance, float margin) const;
  FrontierRoute measuredNarrowPass(const Eigen::Vector3f &body,bool rising,double &clearance) const;
  FrontierRoute observationStep(const Eigen::Vector3f &position, const FrontierRoute &starts,
                               const Eigen::Vector3f &goal, double clearance = -1.) const;
  FrontierRoute approachPassGoal(const Eigen::Vector3f &body,const FrontierRoute &starts,
                                const Eigen::Vector3f &goal,double &clearance) const;
  std::vector<std::pair<Eigen::Vector3f, double>> blocked_points_;
  bool certifyPassGuide(FrontierRoute &route, const char **reason = nullptr, double clearance = -1., bool keep_order = false) const;
  bool repairHeightExcursions(FrontierRoute &path, const Eigen::Vector3f &body, float margin) const;
  bool certifyStairStep(FrontierRoute &route,double clearance = -1.) const;
  bool connectedAcrossPass(const Eigen::Vector3f &a, const Eigen::Vector3f &b) const;
};

} // namespace fast_planner

#endif
