#pragma once

#include <epic_planner/frontier_route.h>
#include <functional>
#include <optional>

namespace fast_planner {

double levelGuideRenewalDistance(double speed, double acceleration, double jerk,
                                 double handoff_time, double clearance);
FrontierRoute forwardLevelPrefix(FrontierRoute route, const Eigen::Vector3f &heading,
                                 double arc_limit, float clearance);
FrontierRoute roundObservedTurns(const FrontierRoute &path, float radius, float spacing,
    const std::function<bool(const Eigen::Vector3f &, const Eigen::Vector3f &)> &clear);

FrontierRoute trimDepartureRetrace(const FrontierRoute &path,const Eigen::Vector3f &velocity,float clearance,
    const std::function<bool(const Eigen::Vector3f &,const Eigen::Vector3f &)> &clear);

FrontierRoute alignMeasuredDeparture(const FrontierRoute &path, const Eigen::Vector3f &velocity,
    float clearance, float spacing, const std::function<bool(const Eigen::Vector3f &, const Eigen::Vector3f &)> &clear);

// An observation pose can be slightly off the flyable centreline. Coming
// back to the same approach is worthwhile only after its local map grows.
struct PassageBranch {
  Eigen::Vector3f goal;
  double next_attempt = 0.;
  std::optional<size_t> free_cells_at_approach;
  bool hasNewEvidence(size_t cells) const {
    if (!free_cells_at_approach) return true;
    const size_t previous = *free_cells_at_approach;
    return cells >= previous + std::max<size_t>(1, (previous + 19) / 20);
  }
};

// Actual motion from the entrance of a level. A failed inspection must not
// strand the vehicle at a dead end simply because the way out is surveyed.
FrontierRoute improveObservedPreview(const FrontierRoute &route,const Eigen::Vector3f &velocity,float clearance,
    float resolution,const std::function<bool(const Eigen::Vector3f &,const Eigen::Vector3f &)> &clear);
float movingSightDistance(const FrontierRoute &route,float spacing,float tolerance,float horizon);

FrontierRoute centerObservedAisle(const Eigen::Vector3f &velocity,const FrontierRoute &route,float clearance,
    const std::function<float(const Eigen::Vector3f &)> &room,const std::function<bool(const Eigen::Vector3f &,const Eigen::Vector3f &)> &clear);

FrontierRoute shapeTransitApproach(const FrontierRoute &path,const Eigen::Vector3f &velocity,float clearance,float resolution,
    const std::function<float(const Eigen::Vector3f &)> &room, const std::function<bool(const Eigen::Vector3f &,const Eigen::Vector3f &)> &clear);

FrontierRoute smoothLevelApproach(const FrontierRoute &path,const Eigen::Vector3f &velocity,
    float clearance,float resolution,const std::function<bool(const Eigen::Vector3f &,const Eigen::Vector3f &)> &clear);

FrontierRoute rollingVisibilityGuide(const Eigen::Vector3f &body_velocity,const FrontierRoute &path,
    float clearance,float resolution,const std::function<float(const Eigen::Vector3f &)> &room,
    const std::function<bool(const Eigen::Vector3f &,const Eigen::Vector3f &)> &clear,float required_preview=0.f);

FrontierRoute repairObservedBend(const FrontierRoute &path,float margin,float spacing,
    const std::function<float(const Eigen::Vector3f &)> &room,
    const std::function<bool(const Eigen::Vector3f &,const Eigen::Vector3f &)> &clear,float horizon_limit=0.f);

FrontierRoute fitMeasuredBend(const FrontierRoute &path, float clearance, float spacing,
    const std::function<bool(const Eigen::Vector3f &, const Eigen::Vector3f &)> &clear);

float horizontalObservationGain(const Eigen::Vector3f &p,float clearance,
    const std::function<bool(const Eigen::Vector3f &)> &free,
    const std::function<float(const Eigen::Vector3f &)> &room);

FrontierRoute optimizeTransitWindow(const FrontierRoute &route,const Eigen::Vector3f &velocity,float clearance,
    double speed_limit,double acceleration,double jerk,const std::function<float(const Eigen::Vector3f &)> &room,
    const std::function<bool(const Eigen::Vector3f &,const Eigen::Vector3f &)> &clear);

double estimateTransitTime(const FrontierRoute &route,const Eigen::Vector3f &velocity,float spacing,
    double speed_limit,double acceleration,double jerk);

FrontierRoute smoothMeasuredBand(const FrontierRoute &path,float clearance,float spacing,
    const std::function<float(const Eigen::Vector3f &)> &room,
    const std::function<bool(const Eigen::Vector3f &,const Eigen::Vector3f &)> &clear);
FrontierRoute observedClimb(const Eigen::Vector3f &body, const Eigen::Vector3f &velocity, const FrontierRoute &goals,
    float clearance, float minimum, const std::function<float(const Eigen::Vector3f &)> &room,
    const std::function<bool(const Eigen::Vector3f &)> &unvisited);
double observationStepLowerBound(double gap,double radius);
bool clearanceBallsOverlap(double distance,double first_room,double second_room,double clearance);
bool hasSupportingFloor(const Eigen::Vector3f &body,float clearance,
    const std::function<bool(const Eigen::Vector3f &)> &observed,
    const std::function<float(const Eigen::Vector3f &)> &room);
bool measuredCorridorCap(const FrontierRoute &approach,const Eigen::Vector3f &direction,float clearance,
    const std::function<bool(const Eigen::Vector3f &)> &free,
    const std::function<float(const Eigen::Vector3f &)> &room,
    const std::function<bool(const Eigen::Vector3f &)> &surveyed);

FrontierRoute observedEndTurn(const FrontierRoute &approach, const Eigen::Vector3f &heading, float radius,
    float spacing, float return_length, int side,
    const std::function<bool(const Eigen::Vector3f &,const Eigen::Vector3f &)> &clear,
    const Eigen::Vector3f &apex_offset=Eigen::Vector3f::Zero());

FrontierRoute observedReturnLane(const Eigen::Vector3f &start,const Eigen::Vector3f &inbound,
    const FrontierRoute &reverse_track,float clearance,
    const std::function<bool(const Eigen::Vector3f &,const Eigen::Vector3f &)> &clear,
    const std::function<bool(const Eigen::Vector3f &)> &goal_free);

bool hasHorizontalOutlet(const Eigen::Vector3f &body, float clearance,
    const std::function<bool(const Eigen::Vector3f &)> &free,
    const std::function<bool(const Eigen::Vector3f &)> &unvisited);

FrontierRoute measuredAscentTrace(const FrontierRoute &track,float clearance);

class FloorTrace {
public:
  void observe(const Eigen::Vector3f &position,float clearance);
  std::optional<Eigen::Vector3f> landingArrival(const Eigen::Vector3f &position,float clearance) const;
  void startingAscent(const Eigen::Vector3f &position,float clearance);
  FrontierRoute outletProbes(const Eigen::Vector3f &position,float clearance) const;
  bool inAscentFootprint(const Eigen::Vector3f &p,float clearance) const;
  bool surveyedLevel(const Eigen::Vector3f &p,float clearance) const {
    return swept_level_ && std::abs(p.z()-height_)<1.8f*clearance;
  }
  bool climbingAt(const Eigen::Vector3f &body,float clearance) const {
    return current_ascent_ && body.z()>current_ascent_->base+.75f*clearance;
  }
  struct Junction {Eigen::Vector3f lane,side;};
  std::optional<Junction> lateralJunction(const Eigen::Vector3f &end,const Eigen::Vector3f &axis,float clearance) const;
  std::optional<Junction> returnJunction(const Eigen::Vector3f &end,const Eigen::Vector3f &body,
                                        const Eigen::Vector3f &direction,float clearance) const;
  bool returningThroughEntrance(const FrontierRoute &remaining,float clearance) const;
  bool returningThroughEntrance(const Eigen::Vector3f &body,const Eigen::Vector3f &tip,float clearance) const;
  bool needsReturn(const Eigen::Vector3f &position,float clearance) const;
  FrontierRoute recentReturn(const Eigen::Vector3f &p,float horizon,float clearance) const;
  FrontierRoute returnPath(const Eigen::Vector3f &position,float horizon,float clearance) const;
  std::vector<FrontierRoute> ascentCandidates(const Eigen::Vector3f &position, const Eigen::Vector3f &heading,
                                              float horizon, float clearance) const;
  const std::optional<Eigen::Vector3f> &entrance() const {return entrance_;}
private:
  void refineEntrance(const Eigen::Vector3f &position,float clearance);
  FrontierRoute recent_, trail_;
  FrontierRoute access_track_;
  float access_distance_=0.f;
  std::optional<Eigen::Vector3f> entrance_;
  float height_=0.f;
  bool outside_=false, swept_level_=false, return_completed_=false;
  struct LevelSurvey {float height; FrontierRoute departure;};
  std::vector<LevelSurvey> surveys_;
  struct Ascent { float base; FrontierRoute trace; };
  std::vector<Ascent> ascents_;
  std::optional<Ascent> current_ascent_;
};

// All geometry is in the LiDAR map frame. A pass lasts across viewpoint and
// cluster replacements; only an accepted change of direction starts a new one.
class CorridorPass {
public:
  using Point = Eigen::Vector3f;
  using Connected = std::function<bool(const Point &, const Point &)>;
  enum class Leg { Forward, Branch, Return };
  enum class Reason { Continue, AwaitScan, NoForward, Wall, Gain };
  struct Candidate {
    Leg leg;
    double length;
    double retraced;
    double visible_area;
  };
  struct Selection {
    std::vector<size_t> indices;
    Reason reason;
    bool reverses() const {
      return reason == Reason::NoForward || reason == Reason::Wall || reason == Reason::Gain;
    }
  };

  // metres, seconds between distinct scans, and relative gain per metre.
  void configure(double clearance, double confirmation, double gain_ratio);
  void observe(const Point &position);
  void commit(const FrontierRoute &guide, bool reverse);
  Candidate assess(const FrontierRoute &path, double visible_area, const Connected &connected) const;
  bool allowsPreview(const FrontierRoute &prefix, const FrontierRoute &leg, bool new_pass,
                     const Connected &connected) const;
  Selection select(const std::vector<Candidate> &candidates, double scan_stamp, bool wall);
  bool wallAhead(const Point &position, const std::function<float(const Point &)> &distance) const;
  float measuredApproach(const Point &body,const Point &direction) const;
  const Point &heading() const { return heading_; }

private:
  Candidate assessAgainst(const FrontierRoute &history, const FrontierRoute &path,
                          const Point &heading, double area, const Connected &connected) const;
  double clearance_ = .8, confirmation_ = 1.5, gain_ratio_ = 2.5;
  double missing_since_ = -1., guide_progress_ = 0.;
  FrontierRoute trail_, guide_;
  Point heading_ = Point::Zero(), last_position_ = Point::Zero();
};
} // namespace fast_planner
