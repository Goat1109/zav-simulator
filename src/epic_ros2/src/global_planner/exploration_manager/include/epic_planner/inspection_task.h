#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace fast_planner {
// Only completed, sensor-confirmed approaches can exhaust a local task.
// A subsequent increase of measured evidence makes the task eligible again.
class InspectionTasks {
 public:
  using Point = Eigen::Vector3f;

  void select(const Point &goal, float radius, size_t evidence) {
    if (!goal.allFinite() || !std::isfinite(radius) || radius <= 0.f) {
      active_ = false;
      return;
    }
    if (active_ && (goal - goal_).norm() < radius_) return;
    if (has_sensor_ && (last_sensor_ - goal).norm() < 2.f * radius) {
      active_ = false;
      return;
    }
    active_ = true;
    goal_ = goal;
    radius_ = radius;
    initial_ = evidence;
  }

  void observe(const Point &sensor, size_t evidence) {
    if (!sensor.allFinite()) return;
    last_sensor_ = sensor;
    has_sensor_ = true;
    if (!active_ || (sensor - goal_).norm() > radius_) return;
    auto found = std::find_if(completed_.begin(), completed_.end(), [&](const Entry &entry) {
      return (entry.goal - goal_).norm() < .5f * radius_;
    });
    const bool gain = evidence > initial_;
    if (found == completed_.end()) {
      if (completed_.size() == 1024) completed_.erase(completed_.begin());
      completed_.push_back({goal_, evidence, gain ? 0u : 1u});
    } else {
      found->empty_visits = gain ? 0u : found->empty_visits + 1;
      found->evidence = evidence;
    }
    active_ = false;
  }

  bool exhausted(const Point &goal, float radius, size_t evidence) const {
    return std::any_of(completed_.begin(), completed_.end(), [&](const Entry &entry) {
      return (entry.goal - goal).norm() < .5f * radius &&
             entry.empty_visits >= 2 && evidence <= entry.evidence;
    });
  }

  bool previouslyExhausted(const Point &goal, float radius) const {
    return std::any_of(completed_.begin(), completed_.end(), [&](const Entry &entry) {
      return entry.empty_visits >= 2 && (entry.goal - goal).norm() < .5f * radius;
    });
  }

  const Point *activeGoal() const {
    return active_ ? &goal_ : nullptr;
  }

 private:
  struct Entry {
    Point goal;
    size_t evidence;
    unsigned empty_visits;
  };
  std::vector<Entry> completed_;
  bool active_ = false;
  bool has_sensor_ = false;
  Point last_sensor_ = Point::Zero();
  Point goal_ = Point::Zero();
  float radius_ = 0.f;
  size_t initial_ = 0;
};
}
