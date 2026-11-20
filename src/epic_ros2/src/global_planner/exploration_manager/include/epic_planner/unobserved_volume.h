#pragma once
#include <Eigen/Core>
#include <array>
#include <vector>

namespace fast_planner {
// Evidence of a local 3D unknown component, including one bending around a
// corner. This scores information only; it never certifies a flight segment.
template<class Unknown>
bool hasConnectedUnknownVolume(const Eigen::Vector3i &seed,Unknown unknown) {
  using Cell=Eigen::Vector3i;
  if(!unknown(seed)) return false;
  std::array<bool,343> seen{};
  auto index=[](const Cell &local) {
    return (local.x()+3)*49+(local.y()+3)*7+local.z()+3;
  };
  std::vector<Cell> queue{Cell::Zero()};
  seen[index(Cell::Zero())]=true;
  Cell lower=Cell::Zero(),upper=Cell::Zero();
  unsigned count=0;
  for(size_t head=0;head<queue.size() && head<96;++head) {
    const Cell current=queue[head];
    lower=lower.cwiseMin(current);upper=upper.cwiseMax(current);++count;
    const Cell span=upper-lower;
    if(count>=12 && (span.array()>=1).all() && (span.array()>=2).count()>=2) return true;
    for(int axis=0;axis<3;++axis)for(int sign:{-1,1}) {
      Cell next=current;next[axis]+=sign;
      if((next.array().abs()>3).any() || seen[index(next)]) continue;
      seen[index(next)]=true;
      if(unknown(seed+next)) queue.push_back(next);
    }
  }
  return false;
}
}
