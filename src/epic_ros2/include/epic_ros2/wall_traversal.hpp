#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

namespace epic_ros {
// Navigation coverage is earned by measured motion, never by offering a goal.
// A directed pass distinguishes the two walls of a narrow corridor even when
// the outbound and inbound body strips overlap. No building coordinates enter
// this journal; current-map collision checks belong to its caller.
class WallTraversal {
public:
  using Point=Eigen::Vector3f;
  using Route=std::vector<Point>;
  struct Sample { Point position,direction; };
  struct Branch { Route route; size_t id; };
private:
  float spacing_,tolerance_;
  std::vector<Sample> flown_;
  std::vector<Branch> branches_;
  Route active_;
  std::optional<Point> last_,heading_;
  size_t next_id_=1;
  float progress_=0.f,travel_=0.f;
  struct Entry {Point anchor,inward;bool returning=false;};
  std::vector<Entry> entries_;
  std::optional<Entry> proposed_entry_;
public:
  explicit WallTraversal(float spacing=.25f,float tolerance=.6f):spacing_(spacing),tolerance_(tolerance) {}
  static float length(const Route& route) {
    float value=0.f;for(size_t i=1;i<route.size();++i)value+=(route[i]-route[i-1]).norm();return value;
  }
  void observe(const Point& body) {
    if(!body.allFinite())return;
    if(!last_){last_=body;return;}
    const Point delta=body-*last_;const float distance=delta.norm();
    if(distance<spacing_*.5f)return;
    // A reset/jump must not mark the intervening space visited.
    if(distance>3.f){last_=body;return;}
    const Point direction=delta/distance;
    if(direction.head<2>().norm()>.5f)heading_=direction;
    const int steps=std::max(1,int(std::ceil(distance/spacing_)));
    for(int j=1;j<=steps;++j)flown_.push_back({*last_+delta*(float(j)/steps),direction});
    travel_+=distance;last_=body;
    while(!entries_.empty() && entries_.back().returning){
      const auto& entry=entries_.back();const Point delta=body-entry.anchor;
      if(delta.dot(entry.inward)>=-tolerance_ ||
         (delta-delta.dot(entry.inward)*entry.inward).head<2>().norm()>4.f*tolerance_)break;
      entries_.pop_back();
    }
    branches_.erase(std::remove_if(branches_.begin(),branches_.end(),[&](const auto& b){
      return freshLength(b.route)<spacing_;}),branches_.end());
  }
  bool visited(const Point& point,const Point& direction) const {
    for(const auto& sample:flown_)
      if((sample.position-point).head<2>().squaredNorm()<tolerance_*tolerance_ &&
         std::abs(sample.position.z()-point.z())<1.f && sample.direction.dot(direction)>.5f)return true;
    return false;
  }
  float freshLength(const Route& route) const {
    float fresh=0.f;
    for(size_t i=1;i<route.size();++i){
      const Point edge=route[i]-route[i-1];const float distance=edge.norm();if(distance<.01f)continue;
      const int steps=std::max(1,int(std::ceil(distance/spacing_)));
      for(int j=1;j<=steps;++j)
        if(!visited(route[i-1]+edge*((j-.5f)/steps),edge/distance))fresh+=distance/steps;
    }
    return fresh;
  }
  void offer(const Route& route) {
    if(route.size()<2 || !std::all_of(route.begin(),route.end(),[](const auto& p){return p.allFinite();}) ||
       freshLength(route)<2.f*spacing_)return;
    const Point edge=route.back()-route[route.size()-2];if(edge.norm()<.01f)return;
    for(auto& branch:branches_){
      const Point previous=branch.route.back()-branch.route[branch.route.size()-2];
      if((branch.route.back()-route.back()).norm()<2.f*tolerance_ && previous.normalized().dot(edge.normalized())>.5f){
        branch.route=route;return;
      }
    }
    branches_.push_back({route,next_id_++});
  }
  void proposeEntry(const Point& anchor,const Point& inward){
    if(inward.head<2>().norm()>.1f)proposed_entry_=Entry{anchor,Point(inward.x(),inward.y(),0.f).normalized(),false};
  }
  void clearProposedEntry(){proposed_entry_.reset();}
  void beginReturn(){if(!entries_.empty())entries_.back().returning=true;}
  bool mayFollow(const Route& route)const{
    for(const auto& entry:entries_){
      if(entry.returning)continue;
      for(size_t i=1;i<route.size();++i){
        const float a=(route[i-1]-entry.anchor).dot(entry.inward),b=(route[i]-entry.anchor).dot(entry.inward);
        if(a<-tolerance_ || b>=-tolerance_ || a-b<.01f)continue;
        const Point crossing=route[i-1]+((a+tolerance_)/(a-b))*(route[i]-route[i-1]);
        if((crossing-entry.anchor).head<2>().norm()<4.f*tolerance_ &&
           std::abs(crossing.z()-entry.anchor.z())<1.f)return false;
      }
    }
    return true;
  }
  void commit(const Route& route) {
    active_=route;progress_=0.f;offer(route);
    if(proposed_entry_){
      if(entries_.empty() || (entries_.back().anchor-proposed_entry_->anchor).norm()>2.f*tolerance_ ||
          entries_.back().inward.dot(proposed_entry_->inward)<.8f)entries_.push_back(*proposed_entry_);
      proposed_entry_.reset();
    }
  }
  // Monotone projection keeps a crossing or a nearby parallel wall from
  // consuming a later, unflown part of the active chain.
  Route remaining(const Point& body) {
    if(active_.size()<2)return {};
    float arc=0.f,best=2.f*tolerance_,chosen_arc=progress_;size_t next=0;Point projection=body;
    for(size_t i=1;i<active_.size();++i){
      const Point edge=active_[i]-active_[i-1];const float distance=edge.norm();if(distance<.01f)continue;
      const float u=std::clamp((body-active_[i-1]).dot(edge)/(distance*distance),0.f,1.f);
      const float at=arc+u*distance;const Point p=active_[i-1]+u*edge;
      if(at>=progress_-.2f && at<=progress_+3.f && (p-body).norm()<best){
        best=(p-body).norm();projection=p;chosen_arc=at;next=i;
      }
      arc+=distance;
    }
    if(!next)return {};
    progress_=std::max(progress_,chosen_arc);
    if((body-active_.back()).norm()<.35f && length(active_)-progress_<.6f){active_.clear();return {};}
    Route route{body};if((projection-body).norm()>.05f)route.push_back(projection);
    for(size_t i=next;i<active_.size();++i)if((active_[i]-route.back()).norm()>.05f)route.push_back(active_[i]);
    return route.size()>1 ? route : Route{};
  }
  void clearActive(){active_.clear();progress_=0.f;}
  const Route& active()const{return active_;}
  const std::vector<Branch>& branches()const{return branches_;}
  const std::optional<Point>& heading()const{return heading_;}
  float travel()const{return travel_;}
};
}
