#pragma once
#include <zav_super/missions.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace argo_super {
// Ordered 3D projection and chord-bounded preview, matching the simulator's
// PolylineFollower. A local arc window prevents shortcuts at crossings/returns.
class PathFollower {
  std::vector<Waypoint> points_;
  std::vector<double> arc_;
  double progress_=0.,target_=0.,chord_=.15;
  size_t segment_=0;
  bool reversal(size_t i)const {
    return (points_[i].position-points_[i-1].position).normalized().dot(
      (points_[i+1].position-points_[i].position).normalized())<-.5;
  }
  double bounded(double end)const {
    const auto origin=at(progress_).position;
    auto valid=[&](double s){
      const Eigen::Vector3d d=at(s).position-origin;
      for(size_t i=1;i<arc_.size();++i){
        if(arc_[i]<=progress_ || arc_[i]>=s)continue;
        const auto& p=points_[i].position;
        const double f=d.squaredNorm()>1e-12?std::clamp((p-origin).dot(d)/d.squaredNorm(),0.,1.):0.;
        if((p-origin-f*d).norm()>chord_)return false;
      }
      return true;
    };
    double previous=progress_;
    for(size_t i=1;i<=arc_.size();++i){
      const double s=i<arc_.size()?arc_[i]:end;
      if(s<=progress_)continue;
      const double next=std::min(s,end);
      if(!valid(next)){
        double lo=previous,hi=next;
        for(int n=0;n<20;++n){const double mid=(lo+hi)/2.;if(valid(mid))lo=mid;else hi=mid;}
        return lo;
      }
      previous=next;if(next>=end)break;
    }
    return end;
  }
  double nearestArc(const Eigen::Vector3d& position,double end) const {
    double best=1e100,best_arc=progress_;
    for(size_t i=segment_;i+1<points_.size() && arc_[i]<=end+1e-9;++i){
      const double len=arc_[i+1]-arc_[i],lo=std::max(0.,progress_-arc_[i]),hi=std::min(len,end-arc_[i]);
      if(lo>hi)continue;
      const Eigen::Vector3d d=(points_[i+1].position-points_[i].position)/len;
      const double along=std::clamp((position-points_[i].position).dot(d),lo,hi);
      const double distance=std::round((position-points_[i].position-along*d).norm()*1e8)/1e8;
      if(distance<best){best=distance;best_arc=arc_[i]+along;}
    }
    return best_arc;
  }
public:
  double lookahead=4.;
  PathFollower()=default;
  explicit PathFollower(const std::vector<Waypoint>& points,double chord=.15):chord_(chord){
    if(!std::isfinite(chord) || chord<=0.)throw std::invalid_argument("Invalid chord error");
    for(const auto& p:points){
      if(!p.position.allFinite() || !std::isfinite(p.yaw))throw std::invalid_argument("Invalid path vertex");
      if(points_.empty() || (p.position-points_.back().position).norm()>1e-6)points_.push_back(p);
    }
    if(points_.size()<2)throw std::invalid_argument("Path requires two distinct vertices");
    arc_.push_back(0.);
    for(size_t i=1;i<points_.size();++i)arc_.push_back(arc_.back()+(points_[i].position-points_[i-1].position).norm());
  }
  bool empty()const{return points_.empty();}
  double progress()const{return progress_;}
  double targetArc()const{return target_;}
  double length()const{return arc_.empty()?0.:arc_.back();}
  size_t segment()const{return segment_;}
  bool same(const std::vector<Waypoint>& points)const{
    std::vector<Waypoint> unique;
    for(const auto& p:points)if(unique.empty() || (p.position-unique.back().position).norm()>1e-6)unique.push_back(p);
    if(unique.size()!=points_.size())return false;
    for(size_t i=0;i<unique.size();++i)if((unique[i].position-points_[i].position).norm()>1e-6 || std::abs(angle(unique[i].yaw-points_[i].yaw))>1e-6)return false;
    return true;
  }
  Waypoint at(double s)const{
    size_t i=0;while(i+2<arc_.size() && arc_[i+1]<s)++i;
    const double f=std::clamp((s-arc_[i])/(arc_[i+1]-arc_[i]),0.,1.);
    return {points_[i].position+f*(points_[i+1].position-points_[i].position),
      points_[i].yaw+f*angle(points_[i+1].yaw-points_[i].yaw)};
  }
  // Carry progress only across the connected, visible start of the new route.
  void inheritProgress(const PathFollower& old,const Eigen::Vector3d& position){
    if(old.empty())return;
    const auto previous=old.at(old.progress_).position;
    const double end=bounded(std::min(length(),lookahead));
    const double seed=nearestArc(previous,end);
    const auto projected=at(seed).position;
    if((projected-previous).norm()>chord_ || (projected-position).norm()>chord_)return;
    // Never seed past a reversal on a route that folds back near itself.
    for(size_t i=1;i+1<arc_.size() && arc_[i]<=seed;++i)if(reversal(i))return;
    progress_=seed;
  }
  Waypoint update(const Eigen::Vector3d& position,const Eigen::Vector3d* last_request=nullptr,double advance=0.){
    if(empty() || !position.allFinite() || !std::isfinite(lookahead) || lookahead<=0.)throw std::invalid_argument("Invalid follower input");
    for(size_t i=segment_+1;i+1<points_.size() && arc_[i]<=progress_+lookahead;++i)
      if(reversal(i) && (position-points_[i].position).norm()<.35){progress_=std::max(progress_,arc_[i]);segment_=i;break;}
    double end=bounded(std::min(length(),progress_+lookahead));
    for(size_t i=segment_+1;i+1<points_.size() && arc_[i]<=end;++i)
      if(reversal(i) && (position-points_[i].position).norm()>=.35){end=arc_[i];break;}
    progress_=nearestArc(position,end);
    while(segment_+2<points_.size() && arc_[segment_+1]<=progress_+1e-8)++segment_;
    end=bounded(std::min(length(),progress_+lookahead));
    for(size_t i=segment_+1;i+1<points_.size() && arc_[i]<=end;++i)if(reversal(i)){end=arc_[i];break;}
    if(last_request){
      // Reproject the issued world-space goal, never its old route index.
      // If a revision shortens the safe preview, its corner bound takes priority.
      const double previous=nearestArc(*last_request,end);
      target_=std::min(end,previous+advance);
    }else target_=std::max(target_,end);
    return at(target_);
  }
};
inline double brakingLookahead(double base,double speed,double acceleration,double jerk,double maximum){
  const double duration=speed<=acceleration*acceleration/jerk?2.*std::sqrt(speed/jerk):speed/acceleration+acceleration/jerk;
  return std::min(maximum,base+.5*speed+.5*speed*duration);
}
}
