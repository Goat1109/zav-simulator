#pragma once
#include <epic_ros2/ros.hpp>
#include <omp.h>
#include <algorithm>

namespace epic_ros {
// Share the CPU with LIO, simulation and flight control. Keep one budget for
// all planning loops rather than overriding OpenMP with 4/6 workers per call.
inline int planningWorkerCount() {
  static const int count=[] {
    NodeHandle nh;
    int requested=2;
    nh.param("exploration/planning_workers",requested,2);
    return std::max(1,std::min(requested,omp_get_num_procs()));
  }();
  return count;
}
}
