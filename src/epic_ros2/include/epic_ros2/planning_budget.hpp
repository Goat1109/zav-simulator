#pragma once
#include <algorithm>
#include <chrono>
#include <limits>

namespace epic_ros {
// Cooperative wall-clock budget, independent of /clock. Each planning thread
// binds the same absolute deadline; nested searches can only shorten it.
class PlanningBudget {
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;
private:
  inline static thread_local TimePoint deadline_ = TimePoint::max();
public:
  class Scope {
    TimePoint previous_;
  public:
    explicit Scope(TimePoint deadline):previous_(deadline_) { deadline_=std::min(deadline_,deadline); }
    explicit Scope(double seconds):Scope(Clock::now()+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds))) {}
    ~Scope() { deadline_=previous_; }
    Scope(const Scope&)=delete;
    Scope& operator=(const Scope&)=delete;
  };
  static TimePoint deadline() { return deadline_; }
  static bool expired() { return Clock::now()>=deadline_; }
  static double remaining(double requested) {
    if (deadline_==TimePoint::max()) return requested;
    return std::max(0.,std::min(requested,std::chrono::duration<double>(deadline_-Clock::now()).count()));
  }
};
}
