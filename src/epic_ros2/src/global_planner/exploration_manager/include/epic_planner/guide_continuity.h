#pragma once
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <stdexcept>

namespace fast_planner {
// Serialization contract shared by native and ordered guides. Subdivision
// preserves every original corner and every already-certified straight edge.
inline void boundGuideEdges(std::vector<Eigen::Vector3f>& path,float maximum=2.f) {
  if(!std::isfinite(maximum) || maximum<=0.f)throw std::invalid_argument("Invalid guide edge limit");
  if(path.size()<2)return;
  std::vector<Eigen::Vector3f> sampled{path.front()};
  for(size_t i=1;i<path.size();++i){
    const Eigen::Vector3f edge=path[i]-path[i-1];
    const int steps=std::max(1,int(std::ceil(edge.norm()/maximum)));
    for(int j=1;j<steps;++j)sampled.push_back(path[i-1]+(float(j)/steps)*edge);
    sampled.push_back(path[i]);
  }
  path.swap(sampled);
}

// A route revision may start with an obsolete short reverse hook. Preserve
// legitimate reversals unless a forward, fully measured connector is safe.
template<class Certified>
bool forwardDeparture(std::vector<Eigen::Vector3f>& path,const Eigen::Vector3f& velocity,Certified certified){
  if(path.size()<3 || velocity.head<2>().norm()<.2f)return false;
  const Eigen::Vector2f forward=velocity.head<2>().normalized();
  Eigen::Vector2f first=path[1].head<2>()-path.front().head<2>();
  if(first.norm()<.03f || first.dot(forward)>=0.f)return false;
  float arc=0.f;
  for(size_t i=1;i<path.size();++i){
    arc+=(path[i]-path[i-1]).norm();if(arc>2.f)break;
    const Eigen::Vector2f delta=path[i].head<2>()-path.front().head<2>();
    if(delta.norm()<.15f || delta.dot(forward)<.5f*delta.norm())continue;
    const std::vector<Eigen::Vector3f> join{path.front(),path[i]};
    if(!certified(join))continue;
    path.erase(path.begin()+1,path.begin()+i);return true;
  }
  return false;
}
}
