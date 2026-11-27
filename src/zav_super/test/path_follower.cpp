#include <zav_super/path_follower.hpp>
#include <cassert>
#include <iostream>
using namespace argo_super;
int main(){
  PathFollower straight({{{0,0,1},0},{{1,0,1},0},{{2,0,1},0},{{8,0,1},0}});
  auto t=straight.update({.5,0,1});assert(std::abs(t.position.x()-4.5)<1e-8);
  t=straight.update({1,0,1});assert(t.position.x()>4.9); // advances while moving, no stops
  const double progress=straight.progress();straight.update({.2,0,1});assert(straight.progress()>=progress);
  assert(straight.same({{{0,0,1},0},{{1,0,1},0},{{2,0,1},0},{{8,0,1},0}}));
  PathFollower back({{{0,0,1},0},{{2,0,1},0},{{0,0,1},0}});
  assert(back.update({0,0,1}).position.x()<=2. && back.progress()==0.);
  assert(back.update({1,0,1}).position.x()>1.8 && back.progress()<2.);
  back.update({2,0,1});assert(back.progress()>=2.);
  back.update({1,0,1});assert(back.progress()>=3.);
  PathFollower crossing({{{0,0,1},0},{{2,2,1},0},{{0,2,1},0},{{2,0,1},0}});
  crossing.update({1,1,1});assert(crossing.progress()<2.); // no jump to return crossing
  PathFollower stair({{{0,0,1},0},{{2,0,1},0},{{2,0,3},0},{{4,0,3},0}});
  auto preview=stair.update({0,0,1});assert(preview.position.z()<1.17);
  stair.update({1.9,0,1});assert(stair.targetArc()>2.);
  PathFollower revised({{{.8,0,1},0},{{2,0,1},0},{{8,0,1},0}});
  revised.inheritProgress(straight,{1,0,1});
  assert(std::abs(revised.progress()-.2)<1e-8);
  Eigen::Vector3d last(1.3,0,1);
  for(int i=0;i<5;++i){
    const auto limited=revised.update({1,0,1},&last,.25);
    assert(std::abs(limited.position.x()-1.55)<1e-8);
  }
  PathFollower corner({{{0,0,1},0},{{0,.3,1},0},{{5,.3,1},0}});
  Eigen::Vector3d past_corner(4.,.3,1.);
  assert(corner.update({0,0,1},&past_corner,.5).position.x()<.3);
  PathFollower yaw({{{0,0,1},3.1},{{8,0,1},-3.1}});
  assert(std::abs(yaw.update({0,0,1}).yaw)>3.);
  assert(brakingLookahead(4.,1.,6.,10.,15.)>4.5);
  std::cout<<"PASS: continuous projection, no rewind, duplicates, reversal, crossing, stairs, yaw\n";
}
