#pragma once
#include <visualization_msgs/msg/marker.hpp>
#include <chrono>
#include <mutex>
#include <optional>

namespace epic_ros {
// Only the planning callback writes a snapshot. A separate executor group can
// publish it while planning is busy, but may not certify an indefinitely stuck
// planner. All shared state is protected; no live planner fields are read.
class PlannerStatus {
  using Clock=std::chrono::steady_clock;
  mutable std::mutex mutex_;
  visualization_msgs::msg::Marker marker_;
  Clock::time_point updated_{};
public:
  void update(const visualization_msgs::msg::Marker& marker){
    std::lock_guard<std::mutex> lock(mutex_);marker_=marker;updated_=Clock::now();
  }
  std::optional<visualization_msgs::msg::Marker> snapshot(double timeout_s=5.)const{
    std::lock_guard<std::mutex> lock(mutex_);
    if(marker_.text.empty())return std::nullopt;
    auto result=marker_;
    if(std::chrono::duration<double>(Clock::now()-updated_).count()>timeout_s)result.text="CAUTION";
    return result;
  }
};
}
