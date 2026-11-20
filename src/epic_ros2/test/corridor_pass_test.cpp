#include <epic_planner/startup_heading.h>
#include <epic_planner/input_freshness.h>
#include <epic_planner/unobserved_volume.h>
#include <epic_planner/inspection_task.h>
#include <epic_planner/corridor_pass.h>
#include <cassert>
#include <limits>
#include <iostream>

using fast_planner::CorridorPass;
using fast_planner::FrontierRoute;
using Point = CorridorPass::Point;
using Leg = CorridorPass::Leg;
using Reason = CorridorPass::Reason;

int main() {
  {
    using fast_planner::guideHeading;
    const double pi=std::acos(-1.);
    assert(guideHeading(.1,2.5,0.,8.)==.1);
    assert(std::abs(guideHeading(.1,2.5,4.,8.)-1.3)<1.e-9);
    assert(std::abs(guideHeading(.1,2.5,8.,8.)-2.5)<1.e-9);
    assert(std::abs(guideHeading(pi-.1,-pi+.1,4.,8.)-pi)<1.e-9);
    assert(guideHeading(.1,2.5,0.,0.)==.1);
  }
  {
    fast_planner::StartupHeading start;
    double target=-2.35;assert(start.apply(.13,target));
    assert(std::abs(target-(.13-.30))<1.e-9);
    target=-2.35;assert(start.apply(-1.,target));
    assert(std::abs(target+1.30)<1.e-9);
    target=-2.35;assert(!start.apply(-2.,target));assert(target==-2.35);
    target=1.;assert(!start.apply(-2.,target));assert(target==1.); // No repeated startup stage.
    fast_planner::StartupHeading aligned;
    target=.5;assert(!aligned.apply(.13,target));
    fast_planner::StartupHeading departure;
    target=-2.35;
    assert(std::abs(departure.departurePreview(.13,10.,.8,target)-.4)<1.e-9);
    assert(target==.13); // First two seconds preserve measured heading.
    target=-2.35;assert(std::abs(departure.departurePreview(.13,11.9,.8,target)-.4)<1.e-9);
    assert(target==.13);
    target=-2.35;assert(std::abs(departure.departurePreview(.13,13.,.8,target)-.8)<1.e-9);
    assert(std::abs(target+.02)<1.e-9);
    target=-2.35;assert(departure.departurePreview(.13,14.,.8,target)==0.);
    assert(target==-2.35); // Ordinary flight no longer has the departure cap.
  }

  {
    fast_planner::InputFreshness fresh;
    assert(!fresh.observe(10.,9.44,true)); // Observed failed-start latency.
    for(int i=0;i<5;++i) assert(!fresh.observe(10.+.1*i,9.98+.1*i,true));
    assert(fresh.observe(10.5,10.48,true));
    assert(!fresh.observe(10.7,10.48,false)); // Receipt alone does not refresh it.
    assert(!fresh.observe(10.8,11.,false)); // Wrong/future clock epoch.
    assert(!fresh.observe(0.,0.,true)); // Simulator clock reset requires settling.
    assert(fresh.observe(.02,.01,false));
    assert(!fresh.observe(.5,.01,true)); // A paused/cached measurement cannot arm motion.
  }

  {
    using Cell=Eigen::Vector3i;
    for(const Cell seed:std::array<Cell,2>{Cell::Zero(),Cell(100,-20,5)}) {
      assert(fast_planner::hasConnectedUnknownVolume(seed,[](const Cell &){return true;}));
      assert(!fast_planner::hasConnectedUnknownVolume(seed,[&](const Cell &p){return p==seed;}));
      assert(!fast_planner::hasConnectedUnknownVolume(seed,[&](const Cell &p){return p.z()==seed.z();}));
      auto elbow=[&](const Cell &p) {
        const Cell d=p-seed;
        return d.z()>=0 && d.z()<=1 &&
          ((d.x()==0 && d.y()>=0 && d.y()<=3) ||
           (d.y()==3 && d.x()>=0 && d.x()<=3));
      };
      assert(elbow(seed) && !elbow(seed+Cell(1,0,0)));
      assert(fast_planner::hasConnectedUnknownVolume(seed,elbow));
      assert(!fast_planner::hasConnectedUnknownVolume(seed,[&](const Cell &p) {
        return elbow(p) && p.y()!=seed.y()+1;
      }));
    }
  }

  {
    for(double gap:{.5,2.,8.,20.}) for(double radius:{1.,4.,8.8}) {
      const Point goal(gap,0,0);
      const double bound=fast_planner::observationStepLowerBound(gap,radius);
      for(int i=-10;i<=10;++i)for(int j=-10;j<=10;++j) {
        const Point end(float(radius*i/10),float(radius*j/10),0);
        if(end.norm()>radius) continue;
        const double cost=(goal-end).norm()+.12*end.norm();
        assert(cost+1.e-5>=bound);
      }
    }
  }
  assert(fast_planner::clearanceBallsOverlap(.1,.9,.9,.8));
  assert(!fast_planner::clearanceBallsOverlap(.3,.81,.81,.8));
  assert(!fast_planner::clearanceBallsOverlap(.01,.79,.9,.8));
  assert(!fast_planner::clearanceBallsOverlap(10.,100.,100.,.8));

  {
    const auto observed=[](const Point &){return true;};
    const auto floor=[](const Point &p){return std::abs(p.z());};
    assert(fast_planner::hasSupportingFloor({0,0,1.6f},.8f,observed,floor));
    assert(!fast_planner::hasSupportingFloor({0,0,6.4f},.8f,observed,floor));
    const auto shelf=[](const Point &p){
      return std::hypot(std::max(0.f,std::abs(p.x())-.55f),p.z()-4.8f);
    };
    assert(!fast_planner::hasSupportingFloor({0,0,6.4f},.8f,observed,shelf));
    assert(!fast_planner::hasSupportingFloor({0,0,1.6f},.8f,[](const Point &){return false;},floor));
  }

  {
    fast_planner::InspectionTasks tasks;
    const Point goal(5,1,2),other(5,1,6);
    fast_planner::InspectionTasks invalid;
    invalid.select(goal,1.f,10);
    invalid.observe(Point::Constant(std::numeric_limits<float>::quiet_NaN()),10);
    assert(invalid.activeGoal());
    invalid.select(goal,-1.f,10);
    assert(!invalid.activeGoal());
    tasks.select(goal,1.f,100);
    tasks.observe({0,0,0},100);
    assert(!tasks.exhausted(goal,1.f,100));
    tasks.observe(goal,100);
    tasks.select(goal,1.f,100);tasks.observe(goal,100);
    assert(!tasks.exhausted(goal,1.f,100)); // Replanning in place is one visit.
    tasks.observe({0,0,0},100);
    tasks.select(goal,1.f,100);tasks.observe(goal,100);
    assert(tasks.exhausted(goal,1.f,100));
    assert(!tasks.exhausted(other,1.f,100));
    assert(!tasks.exhausted(goal,1.f,101));
    tasks.observe({0,0,0},101);
    tasks.select(goal,1.f,101);tasks.observe(goal,120);
    assert(!tasks.exhausted(goal,1.f,120));
    tasks.select(other,1.f,100);tasks.observe(other,100);
    assert(!tasks.exhausted(goal,1.f,120));
  }

  {
    const FrontierRoute retrace{{0,0,0},{-.8f,.2f,.2f},{.4f,.3f,0},{2.5f,.5f,0},{3,1,0}};
    const auto free=[](const Point &,const Point &){return true;};
    auto linked=fast_planner::trimDepartureRetrace(retrace,{2,0,0},.75f,free);
    assert(linked.size()>=2 && linked[1].x()>.8f && linked.back().isApprox(retrace.back()));
    assert(fast_planner::trimDepartureRetrace(retrace,{2,0,0},.75f,
        [](const Point &,const Point &){return false;}).empty());
    const FrontierRoute real_return{{0,0,0},{-1.9f,.2f,0},{-3,.3f,0},{3,.3f,0}};
    assert(fast_planner::trimDepartureRetrace(real_return,{2,0,0},.75f,free).empty());
  }
  {
    const FrontierRoute corridor{{0,0,1.2f},{12,0,1.2f}};
    auto room=[](const Point &p){return std::min(.92f+p.y(),2.4f-p.y());};
    auto clear=[&](const Point &a,const Point &b){return std::min(room(a),room(b))>.7f;};
    auto aisle=fast_planner::centerObservedAisle({4,0,0},corridor,.7f,room,clear);
    assert(aisle.size()>2 && aisle[1].y()>.2f && aisle.front()==corridor.front() && aisle.back()==corridor.back());
    assert(fast_planner::centerObservedAisle({0,4,0},corridor,.7f,room,clear).empty());
    assert(fast_planner::centerObservedAisle({4,0,0},corridor,.7f,room,[](const Point&,const Point&){return false;}).empty());
  }
  {
    const FrontierRoute straight{{0,0,0},{2,0,.5f},{4,0,1}};
    const FrontierRoute corner{{0,0,0},{2,0,0},{2,2,0}};
    assert(fast_planner::movingSightDistance(straight,.2f,.15f,3.5f)>3.49f);
    const float sight=fast_planner::movingSightDistance(corner,.2f,.15f,4.f);
    assert(sight>=1.99f && sight<2.4f);
  }
  {
    fast_planner::FloorTrace storey;
    for(int i=0;i<26;++i)storey.observe({.4f*i,4,1.3f},.8f);
    for(int i=1;i<16;++i)storey.observe({10,4.f-.4f*i,1.3f},.8f);
    for(int i=1;i<38;++i)storey.observe({10.f+.4f*i,-2,1.3f},.8f);
    const auto crossing=storey.lateralJunction({19,-2,1.3f},{1,0,0},.8f);
    assert(crossing && std::abs(crossing->lane.x()-10.f)<.5f && crossing->side.y()>-.4f);
    assert(!storey.lateralJunction({19,-2,6},{1,0,0},.8f));
    const auto back=storey.returnJunction({26,-2,1.3f},{24.8f,-2,1.3f},Point(.99f,.12f,0).normalized(),.8f);
    assert(back && std::abs(back->lane.x()-10.f)<.5f && std::abs(back->lane.y()+2.f)<.1f);
    assert(!storey.returnJunction({26,-2,5.3f},{24.8f,-2,5.3f},{1,0,0},.8f));
  }
  {
    const FrontierRoute lane{{0,0,0},{2,0,.1f},{3,0,.62f},{5,0,.65f},{9,0,.65f},{13,0,.65f}};
    const auto clear=[](const Point &,const Point &){return true;};
    auto smooth=fast_planner::smoothLevelApproach(lane,{4,0,-.25f},.8f,.2f,clear);
    assert(smooth.size()>2 && smooth.front().isApprox(lane.front()) && smooth.back().isApprox(lane.back()));
    assert(fast_planner::smoothLevelApproach(lane,{4,0,0},.8f,.2f,[](const Point &,const Point &){return false;}).empty());
    const FrontierRoute corner{{0,0,0},{2,0,0},{3,1,0},{3,4,0},{3,9,0}};
    assert(fast_planner::smoothLevelApproach(corner,{4,0,0},.8f,.2f,clear).empty());
  }
  {
    const FrontierRoute corner{{0,0,0},{1,0,0},{2,0,0},{3,0,0},{3,1,.2f},{3,2,.4f},{3,3,.5f}};
    const auto room=[](const Point &){return 2.f;};
    const auto yes=[](const Point &,const Point &){return true;};
    auto rounded=fast_planner::optimizeTransitWindow(corner,{2,0,0},.75f,5.,3.,8.,room,yes);
    assert(rounded.size()>2 && rounded.front().isApprox(corner.front()) && rounded.back().isApprox(corner.back()));
    assert(fast_planner::optimizeTransitWindow(corner,{2,0,0},.75f,5.,3.,8.,room,
          [](const Point &,const Point &){return false;}).empty());
    const FrontierRoute reverse{{0,0,0},{2,0,0},{3,0,0},{2,0,0},{0,0,0}};
    assert(fast_planner::optimizeTransitWindow(reverse,{2,0,0},.75f,5.,3.,8.,room,yes).empty());
  }
  {
    const FrontierRoute ramp{{0,0,0},{2,0,.65f},{4,0,1.3f},{6,0,1.9f},{8,0,2.5f}};
    const Point obstacle(2,0,.65f);
    const auto room=[&](const Point &p){return (p-obstacle).norm();};
    const auto clear=[&](const Point &a,const Point &b) {
      const Point line=b-a;
      const float t=std::clamp((obstacle-a).dot(line)/line.squaredNorm(),0.f,1.f);
      return (a+t*line-obstacle).norm()>.65f;
    };
    const auto detour=fast_planner::repairObservedBend(ramp,.65f,.8f,room,clear);
    assert(detour.size()>=3 && detour.front().isApprox(ramp.front()) && detour.back().isApprox(ramp.back()));
    for(size_t i=1;i<detour.size();++i) assert(clear(detour[i-1],detour[i]));
    assert(fast_planner::repairObservedBend(ramp,.65f,.8f,room,[](const Point &,const Point &){return false;}).empty());
  }
  {
    const FrontierRoute stairs{{0,0,0},{2,0,.5f},{3,0,1},{3.5f,0,1.25f},{3.5f,.5f,1.4f},
                               {3.5f,1,1.5f},{3.5f,1.5f,1.55f},{3,1.5f,1.75f},{1,1.5f,2.5f},{0,1.5f,3}};
    const auto yes=[](const Point &,const Point &){return true;};
    const auto width=[](const Point &){return 1.4f;};
    auto early=fast_planner::shapeTransitApproach(stairs,{2,0,.4f},.75f,.2f,width,yes);
    assert(early.size()>2 && early.front().isApprox(stairs.front()) && early.back().isApprox(stairs.back()));
    assert(fast_planner::shapeTransitApproach(stairs,{2,0,.4f},.75f,.2f,width,
        [](const Point &,const Point &){return false;}).empty());
    assert(fast_planner::shapeTransitApproach(stairs,Point::Zero(),.75f,.2f,width,yes).empty());
    auto curve=fast_planner::fitMeasuredBend(stairs,.75f,.3f,yes);
    assert(curve.size()>2 && curve.front().isApprox(stairs.front()) && curve.back().isApprox(stairs.back()));
    assert(fast_planner::fitMeasuredBend(stairs,.75f,.3f,[](const Point &,const Point &){return false;}).empty());
    const FrontierRoute corner{{0,0,0},{1,0,0},{2,0,0},{3,0,0},{3,1,0},{3,2,0},{3,3,0}};
    assert(!fast_planner::fitMeasuredBend(corner,.75f,.3f,yes).empty());
    const auto flat=fast_planner::shapeTransitApproach(corner,{2,0,0},.75f,.2f,width,yes);
    assert(!flat.empty() && flat.front().isApprox(corner.front()) && flat.back().isApprox(corner.back()));
    const FrontierRoute reverse{{0,0,0},{1,0,0},{2,0,0},{3,0,0},{2,0,0},{1,0,0},{0,0,0}};
    assert(fast_planner::fitMeasuredBend(reverse,.75f,.3f,yes).empty());
    const FrontierRoute straight{{0,0,0},{2,0,1},{4,0,2},{6,0,3}};
    assert(fast_planner::fitMeasuredBend(straight,.75f,.3f,yes).empty());
  }
  {
    const auto measured=[](const Point &p){return p.head<2>().norm()<1.5f;};
    const auto wall=[](const Point &p){return p.head<2>().norm()<1.4f ? 1.f : .1f;};
    const auto open=[](const Point &){return 2.f;};
    assert(fast_planner::horizontalObservationGain(Point::Zero(),.8f,measured,wall)==0.f);
    assert(fast_planner::horizontalObservationGain(Point::Zero(),.8f,measured,open)>.9f);
  }
  {
    const FrontierRoute approach{{0,0,1},{4,0,1},{7,0,1}};
    const auto known=[](const Point &){return true;};
    const auto unvisited=[](const Point &){return false;};
    const auto wall=[](const Point &p){return std::abs(1.25f-std::abs(p.y()));};
    assert(fast_planner::measuredCorridorCap(approach,Point::UnitX(),.8f,known,wall,unvisited));
    const auto junction=[&](const Point &p){return p.x()>4.f && p.x()<6.3f && p.y()>0.f ? 3.f : wall(p);};
    assert(!fast_planner::measuredCorridorCap(approach,Point::UnitX(),.8f,known,junction,unvisited));
    const auto unknown=[](const Point &p){return p.y()<.7f;};
    assert(!fast_planner::measuredCorridorCap(approach,Point::UnitX(),.8f,unknown,wall,unvisited));
  }
  {
    FrontierRoute track;
    for(int i=0;i<=12;++i) track.emplace_back(.5f*i,0,1);
    for(int i=1;i<=6;++i) track.emplace_back(6,.5f*i,1);
    for(int i=5;i>=0;--i) track.emplace_back(6.02f,.5f*i,1.04f);
    for(int i=1;i<=12;++i) track.emplace_back(6+.5f*i,0,1.f+.24f*i);
    for(int i=1;i<=12;++i) track.emplace_back(12,.5f*i,3.88f);
    const auto climb=fast_planner::measuredAscentTrace(track,.8f);
    assert(climb.size()>5 && climb.size()<track.size() && climb.front().x()>2.f);
    assert(climb.back().y()<3.f && climb.back().z()-climb.front().z()>2.4f);
    for(const auto &q:climb) assert(q.y()<.8f || q.z()>3.5f);
  }
  {
    fast_planner::FloorTrace trace;
    for(int i=0;i<12;++i) trace.observe({.5f*i,0,1.f},.8f);
    const auto back=trace.recentReturn({5.5f,0,1.f},3.3f,.8f);
    assert(back.size()>4 && std::abs(back.front().x()-5.5f)<.01f && back.back().x()>=2.2f);
    assert(trace.recentReturn({10,0,1.f},4.f,.8f).empty());
  }
  {
    const FrontierRoute direct{{0,0,0},{2,0,1},{4,0,2}};
    const FrontierRoute bends{{0,0,0},{1.5f,1.f,.5f},{2.5f,-1.f,.9f},{4,0,2.f}};
    const double smooth=fast_planner::estimateTransitTime(direct,{2,0,.5f},.4f,5.,3.,8.);
    assert(smooth<fast_planner::estimateTransitTime(bends,{2,0,.5f},.4f,5.,3.,8.));
    assert(smooth<fast_planner::estimateTransitTime(direct,{-2,0,-.5f},.4f,5.,3.,8.));
  }
  {
    const FrontierRoute bend{{0,0,0},{.8f,0,0},{1.4f,.7f,0},{2.4f,1.1f,0},{3.8f,1.3f,0},{5,1.3f,0}};
    const auto yes=[](const Point &,const Point &){return true;};
    const auto no=[](const Point &,const Point &){return false;};
    auto cut=fast_planner::improveObservedPreview(bend,{2,0,0},.8f,.2f,yes);
    assert(cut.size()>=2 && cut.size()<bend.size() && cut.front().isApprox(bend.front()) && cut.back().isApprox(bend.back()));
    assert(fast_planner::improveObservedPreview(bend,{2,0,0},.8f,.2f,no).empty());
    const FrontierRoute u{{0,0,0},{1,0,0},{2,0,0},{1.1f,.35f,0},{0,.5f,0}};
    assert(fast_planner::improveObservedPreview(u,{2,0,0},.8f,.2f,yes).empty());
  }
  {
    const FrontierRoute bend{{0,0,1},{1,0,1},{1.5f,.55f,1.3f},{2.1f,.85f,1.55f},
                          {2.9f,1.05f,1.8f},{3.9f,1.1f,1.95f}};
    const auto room=[](const Point &){return 1.5f;};
    const auto yes=[](const Point &,const Point &){return true;};
    const auto no=[](const Point &,const Point &){return false;};
    const auto guide=fast_planner::rollingVisibilityGuide({1.6f,0,0},bend,.7f,.2f,room,yes);
    assert(guide.size()>=2 && guide.front()==bend.front() && guide.back()==bend.back());
    assert((guide[1]-guide.front()).norm()>2.f);
    assert(fast_planner::rollingVisibilityGuide({1.6f,0,0},bend,.7f,.2f,room,no).empty());
    const FrontierRoute same{{0,0,0},{2,0,0},{4,0,0},{6,0,0}};
    assert(fast_planner::rollingVisibilityGuide({2,0,0},same,.7f,.2f,room,yes).empty());
  }
  {
    const FrontierRoute kink{{0,0,1},{1.9f,.65f,1},{3.5f,-.55f,1},{5.5f,.1f,1}};
    const auto clear=[](const Point &,const Point &){return true;};
    const auto wide=[](const Point &){return 3.f;};
    const auto smooth=fast_planner::smoothMeasuredBand(kink,.7f,.45f,wide,clear);
    assert(smooth.size()>kink.size() && smooth.front().isApprox(kink.front()) && smooth.back().isApprox(kink.back()));
    float sideways=0.f;for(const auto &q:smooth) sideways=std::max(sideways,std::abs(q.y()));
    assert(sideways<.60f);
    const auto blocked=[](const Point &,const Point &){return false;};
    assert(fast_planner::smoothMeasuredBand(kink,.7f,.45f,wide,blocked)==kink);
  }
  {
    const auto room=[](const Point &p) {
      return std::min(1.15f-std::abs(p.z()-.45f*p.x()),1.1f-std::abs(p.y()));
    };
    const auto fresh=[](const Point &p){return p.x()>1.f;};
    const auto path=fast_planner::observedClimb(Point::Zero(),{1,0,.4f},{{6,0,2.7f}},.8f,.65f,room,fresh);
    assert(path.size()>=5 && path.back().x()>4.5f && std::abs(path.back().z()-.45f*path.back().x())<.6f);
    const auto sloping_slot=[](const Point &p) {
      return std::min(1.04f-std::abs(p.x()-.5f*p.z()),1.05f-std::abs(p.y()));
    };
    const auto fresh_upper=[](const Point &p){return p.z()>.8f;};
    const auto upward=fast_planner::observedClimb(Point::Zero(),{.7f,0,1.1f},{{1.5f,0,3.f}},
                                                  .8f,.65f,sloping_slot,fresh_upper);
    assert(!upward.empty() && upward.back().z()>2.3f);
    const auto roof=[&](const Point &p){return p.z()>.6f ? 0.f : sloping_slot(p);};
    assert(fast_planner::observedClimb(Point::Zero(),{.7f,0,1.1f},{{1.5f,0,3.f}},.8f,.65f,roof,fresh_upper).empty());
    const auto wall=[&](const Point &p){return p.x()>1.3f ? 0.f : room(p);};
    assert(fast_planner::observedClimb(Point::Zero(),{1,0,.4f},{{6,0,2.7f}},.8f,.65f,wall,fresh).empty());
  }
  {
    const FrontierRoute hall{{0,0,1},{3,0,1},{6,0,1}};
    const auto clear=[](const Point &,const Point &p){return std::abs(p.y())<=.85f && p.x()<6.01f;};
    const auto loop=fast_planner::observedEndTurn(hall,{1,0,0},.72f,.2f,4.5f,1,clear);
    assert(loop.size()>12 && loop.front().isApprox(hall.front()) && loop.back().isApprox(Point(1.5,0,1)));
    assert(std::abs(fast_planner::routeTangentYaw(loop,0,.3f,0))<.2);
    assert(std::abs(std::abs(fast_planner::routeTangentYaw(loop,loop.size()-1,.3f,0))-M_PI)<.2);
    assert(std::any_of(loop.begin(),loop.end(),[&](auto p){return p.isApprox(hall.back());}));
    const auto closed=[](const Point &,const Point &p){return std::abs(p.y())<.45f;};
    assert(fast_planner::observedEndTurn(hall,{1,0,0},.72f,.2f,4.5f,-1,closed).empty());
  }
  {
    const auto fresh=[](const Point &p){return p.x()>4.f;};
    const auto slope=[](const Point &p){return std::abs(p.y())<1.2f && std::abs(p.z()-.45f*p.x())<.8f;};
    assert(!fast_planner::hasHorizontalOutlet(Point::Zero(),.8f,slope,fresh));
    const auto turn=[](const Point &p){
      return std::abs(p.z())<.6f && ((std::abs(p.x())<.7f && p.y()>-.7f && p.y()<2.6f) ||
                                     (std::abs(p.y()-2.f)<.7f && p.x()>-.7f));
    };
    assert(fast_planner::hasHorizontalOutlet(Point::Zero(),.8f,turn,fresh));
    assert(!fast_planner::hasHorizontalOutlet(Point::Zero(),.8f,turn,[](const Point &){return false;}));
  }
  {
    const auto visible=[](const Point &,const Point &) { return true; };
    const FrontierRoute projected{{0,0,0},{-.19f,.15f,0},{1.3f,.5f,.2f},{4.f,.7f,.4f}};
    const auto joined=fast_planner::alignMeasuredDeparture(projected,{1.7f,0,0},.8f,.2f,visible);
    assert(joined!=projected && joined.front().isApprox(projected.front()) && joined.back().isApprox(projected.back()));
    assert((joined[1]-joined[0]).normalized().x()>.94f);
    const auto blocked=[](const Point &,const Point &p){return p.x()<.1f || p.y()>.95f;};
    assert(fast_planner::alignMeasuredDeparture(projected,{1.7f,0,0},.8f,.2f,blocked)==projected);
    const FrontierRoute reverse{{0,0,0},{-1,0,0},{-2,0,0},{2,0,0}};
    assert(fast_planner::alignMeasuredDeparture(reverse,{1.7f,0,0},.8f,.2f,visible)==reverse);
  }
  {
    fast_planner::FloorTrace trace;
    assert(!trace.surveyedLevel(Point::Zero(),.8f));
    const float safe=.8f;
    for(int i=0;i<=12;++i) trace.observe({-6.f+.5f*i,0.f,1.f},safe);
    assert(trace.ascentCandidates({0,0,1},{0,1,0},12.f,safe).empty());
    assert(!trace.landingArrival({0,0,1},safe));
    trace.startingAscent({0,0,1},safe);
    for(int i=1;i<=8;++i) trace.observe({0,.5f*i,1.f+.25f*i},safe);
    const auto arrival=trace.landingArrival({0,4,3},safe);
    assert(arrival && arrival->dot(Point::UnitY())>.98f);
    for(int i=1;i<=4;++i) trace.observe({.5f*i,4,3},safe);
    for(int i=1;i<=8;++i) trace.observe({2,4.f-.5f*i,3.f+.25f*i},safe);
    for(int i=1;i<=12;++i) trace.observe({2.f-.5f*i,0,5},safe);
    assert(!trace.landingArrival({-4,0,5},safe));
    const auto candidates=trace.ascentCandidates({0,0,5},{0,1,0},12.f,safe);
    assert(!candidates.empty() && candidates.front().front().isApprox(Eigen::Vector3f(0,0,5)));
    assert(candidates.front().back().z()>6.f);
    assert(trace.ascentCandidates({30,0,5},{0,1,0},12.f,safe).empty());
  }
  {
    using fast_planner::FrontierRoute;
    const FrontierRoute bend{{0,0,1},{3,0,2},{3,3,3}};
    const auto visible=[](const auto &,const auto &){return true;};
    const auto smooth=fast_planner::roundObservedTurns(bend,1.f,.2f,visible);
    assert(smooth.size()>bend.size() && smooth.front().isApprox(bend.front()) &&
           smooth.back().isApprox(bend.back()));
    for(size_t i=1;i<smooth.size();++i)
      assert(smooth[i].z()>=smooth[i-1].z() && !smooth[i].isApprox(bend[1]));
    const auto outside=[](const Eigen::Vector3f &a,const Eigen::Vector3f &b){
      return (a.y()==0.f && b.y()==0.f) || (a.x()==3.f && b.x()==3.f);
    };
    assert(fast_planner::roundObservedTurns(bend,1.f,.2f,outside)==bend);
    const FrontierRoute reversal{{0,0,1},{4,0,1},{0,0,1}};
    assert(fast_planner::roundObservedTurns(reversal,1.f,.2f,visible)==reversal);
  }
  using fast_planner::levelGuideRenewalDistance;
  assert(std::abs(levelGuideRenewalDistance(0., 2.5, 5., 1., .8) - 2.4) < 1.e-6);
  assert(std::abs(levelGuideRenewalDistance(4., 2.5, 5., 1., .8) - 10.6) < 1.e-6);
  assert(levelGuideRenewalDistance(1., 2.5, 5., 1., .8) <
         levelGuideRenewalDistance(2., 2.5, 5., 1., .8));
  assert(levelGuideRenewalDistance(4., 2.5, 5., 1.5, .8) >
         levelGuideRenewalDistance(4., 2.5, 5., 1., .8));
  // A lateral adjustment in the same corridor is a continuation. A turn
  // back, side branch or climb must reach normal exploration selection.
  using fast_planner::forwardLevelPrefix;
  const FrontierRoute doorway{{0,0,1},{2,.5f,1},{6,.3f,1}};
  assert(forwardLevelPrefix(doorway, Point::UnitX(), 12., .8f).back() == doorway.back());
  for (const auto &exit : FrontierRoute{{3,1,1},{6,4,1},{7,0,3}}) {
    auto branch = doorway;
    branch.push_back(exit);
    assert(forwardLevelPrefix(branch, Point::UnitX(), 18., .8f).back() == doorway.back());
  }
  fast_planner::PassageBranch inspected{{2,1,1},0.,std::nullopt};
  assert(inspected.hasNewEvidence(40));
  inspected.free_cells_at_approach = 40;
  assert(!inspected.hasNewEvidence(39) && !inspected.hasNewEvidence(41));
  assert(inspected.hasNewEvidence(42));
  fast_planner::FloorTrace floors;
  for (float x=0.f;x<=10.f;x+=.4f) floors.observe({x,0,1.5f},.8f);
  assert(floors.needsReturn({9.8f,0,1.5f},.8f));
  const Point access_door=*floors.entrance();
  assert(floors.returningThroughEntrance(access_door+Point(3.f,0,0),access_door-Point(1.f,0,0),.8f));
  assert(!floors.returningThroughEntrance(access_door-Point(1.f,0,0),access_door+Point(3.f,0,0),.8f));
  assert(!floors.returningThroughEntrance(access_door+Point(0,3.f,0),access_door-Point(0,1.f,0),.8f));
  const FrontierRoute curved_return{access_door+Point(3.8f,0,0),access_door+Point(.2f,0,0),
                                    access_door-Point(1.f,0,0),access_door+Point(-1.f,6.f,0)};
  assert(!floors.returningThroughEntrance(curved_return.front(),curved_return.back(),.8f));
  assert(floors.returningThroughEntrance(curved_return,.8f));
  assert(!floors.returningThroughEntrance({access_door+Point(4,0,0),access_door+Point(3,0,0),
                                         access_door+Point(3,6,0)},.8f));
  const auto out=floors.returnPath({9.8f,0,1.5f},4.f,.8f);
  assert(out.size()>3 && out.back().x()<6.f);
  for (float x=9.6f;x>0.f;x-=.4f) floors.observe({x,0,1.5f},.8f);
  assert(!floors.needsReturn({0,0,1.5f},.8f));
  for(float x=.4f;x<=12.f;x+=.4f) floors.observe({x,0,1.5f},.8f);
  assert(!floors.needsReturn({12,0,1.5f},.8f));

  for (float z=1.5f;z<5.5f;z+=.4f) floors.observe({0,0,z},.8f);
  for (float x=0.f;x<=12.f;x+=.4f) floors.observe({x,0,5.5f},.8f);
  assert(floors.entrance() && floors.entrance()->z()>4.6f && floors.needsReturn({12,0,5.5f},.8f));
  // With two physically swept floors the next landing is a hypothesis.
  // A shaft several corridors away and an intermediate landing are distinct.
  // Near a landing the last part of the stairs can initially resemble a
  // short horizontal opening. Return to the established floor height.
  fast_planner::FloorTrace landing;
  for(int i=0;i<34;++i) landing.observe({.4f*i,0,1},.8f);
  for(int i=33;i>=0;--i) landing.observe({.4f*i,0,1},.8f);
  landing.startingAscent({0,0,1},.8f);
  for(int i=1;i<=10;++i) landing.observe({0,.32f*i,1.f+.32f*i},.8f);
  for(int i=1;i<=4;++i) landing.observe({0,3.2f-.4f*i,4.2f+.2f*i},.8f);
  for(int i=0;i<40;++i) landing.observe({.4f*i,1.6f,5.f+.04f*std::sin(.4f*i)},.8f);
  assert(landing.entrance() && std::abs(landing.entrance()->z()-5.f)<.35f);
  const auto way_back=landing.returnPath({15.6f,1.6f,5},40.f,.8f);
  assert(way_back.size()>1 && way_back.back().isApprox(*landing.entrance(),.1f));
  fast_planner::FloorTrace storeys;
  for(int floor=0;floor<2;++floor) {
    const float z=1.f+4.f*floor;
    for(int i=0;i<=36;++i) storeys.observe({.4f*i,0,z},.8f);
    for(int i=36;i>=0;--i) storeys.observe({.4f*i,0,z},.8f);
    if(floor==0) {
      assert(!storeys.outletProbes({0,0,4.f},.8f).empty());
      assert(storeys.outletProbes({0,0,1.8f},.8f).empty());
    }
    storeys.startingAscent({0,0,z},.8f);
    for(int j=1;j<10;++j) storeys.observe({0,.2f*std::sin(float(j)),z+.4f*j},.8f);
  }
  assert(!storeys.outletProbes({0,0,9.1f},.8f).empty());
  assert(storeys.outletProbes({0,0,6.1f},.8f).empty());
  assert(storeys.outletProbes({40,0,9.1f},.8f).empty());
  const auto open = [](const Point &, const Point &) { return true; };
  // A measured dead-end can have a long, previously flown return. Keep
  // its first junction available and discard any tail cut off by a new wall.
  const FrontierRoute past{{1,0,1},{0,0,1},{-1,.1f,1},{-3,0,1},{-5,0,1},{-6,2,1},{-6,5,1}};
  const auto valid_endpoint=[](const Point &){return true;};
  auto backtrack=fast_planner::observedReturnLane({0,0,1},{-1,0,0},past,.8f,open,valid_endpoint);
  assert(backtrack.size()==2 && backtrack.back().x()==-5 && backtrack.back().y()==0);
  auto blocked_back=fast_planner::observedReturnLane({0,0,1},{-1,0,0},past,.8f,
      [](const Point &,const Point &b){return b.x()>-4.f;},valid_endpoint);
  assert(blocked_back.size()==2 && blocked_back.back().x()==-3);
  assert(fast_planner::observedReturnLane({0,0,1},{1,0,0},past,.8f,open,valid_endpoint).empty());
  CorridorPass pass;
  pass.configure(.8, 1.5, 2.5);
  const FrontierRoute aisle{{0, 0, 1}, {12, 0, 1}};
  pass.commit(aisle, false);
  for (int x = 1; x <= 8; ++x) pass.observe({float(x), 0, 1});
  assert(pass.measuredApproach({8,0,1},{1,0,0})>7.f);
  assert(pass.measuredApproach({8,0,1},{0,1,0})<1.f);
  const auto forward = pass.assess({{8,0,1}, {14,0,1}}, 4., open);
  const auto branch = pass.assess({{8,0,1}, {8,5,2}}, 4., open);
  const auto rear = pass.assess({{8,0,1}, {3,.4f,1}, {3,3,1}}, 4., open);
  assert(forward.leg == Leg::Forward && branch.leg == Leg::Branch);
  assert(rear.leg == Leg::Return && rear.retraced > 4.);
  assert(pass.select({rear,branch,forward}, 10., false).indices == std::vector<size_t>{2});
  assert(pass.select({rear,branch}, 11., true).indices == std::vector<size_t>{1});
  // One empty forward shortlist or repeated planning on the same scan must
  // not flip the pass. Fresh forward evidence resets the confirmation.
  assert(pass.select({rear}, 12., false).reason == Reason::AwaitScan);
  assert(pass.select({rear}, 12., false).indices.empty());
  assert(pass.select({rear, forward}, 13., false).reason == Reason::Continue);
  assert(pass.select({rear}, 16., false).indices.empty());
  assert(pass.select({rear}, 17., false).indices.empty());
  assert(pass.select({rear}, 17.6, false).reason == Reason::NoForward);
  assert(pass.select({rear}, 18., true).reason == Reason::Wall);
  auto valuable_rear = rear;
  valuable_rear.visible_area = 40.;
  const auto diversion = pass.select({forward, valuable_rear}, 19., false);
  assert(diversion.reason == Reason::Gain && diversion.indices == std::vector<size_t>{1});
  const Point origin(8,0,1);
  assert(pass.wallAhead(origin, [](const Point &p) { return std::abs(p.x()-9.6f); }));
  assert(!pass.wallAhead(origin, [](const Point &p) { return std::abs(p.z()); }));
  assert(!pass.wallAhead(origin, [](const Point &p) { // narrow column leaves the passage open
    return std::hypot(p.x()-9.6f, p.y());
  }));

  // Closely spaced viewpoints can turn the last local tangent sideways or
  // backwards before LKH appends a long return (recorded Frontier guides).
  FrontierRoute prefix{{8,0,1}, {13,0,1}, {12.7f,.3f,1.5f}};
  assert(!pass.allowsPreview(prefix, {prefix.back(), {12.8f,.6f,1.5f}, {2,.3f,1}}, false, open));
  assert(pass.allowsPreview(prefix, {prefix.back(), {20,.3f,1}}, false, open));
  assert(!pass.allowsPreview({origin, {12,0,1}},
      {{12,0,1}, {15,0,1}, {15,.5f,1}, {4,.5f,1}}, false, open));
  // The same XY heading on the far side of a wall or on the next level is
  // not a retraced edge. Both the right-angle corner and the U-stair survive.
  auto wall_between = [](const Point &a, const Point &b) {
    return (a.y()-.55f)*(b.y()-.55f) > 0.f;
  };
  assert(pass.allowsPreview({origin,{14,0,1},{14,1.1f,1}},
      {{14,1.1f,1},{5,1.1f,1}}, false, wall_between));
  assert(pass.allowsPreview({origin,{14,0,1}}, {{14,0,1},{14,6,2}}, false, open));
  assert(pass.allowsPreview({origin,{14,0,2.5f},{14,2,3.5f}},
      {{14,2,3.5f},{7,2,5.5f}}, false, open));
  const FrontierRoute below_floor{{0,0,.9f},{.5f,.1f,-.9f},{5,2,.8f}};
  const FrontierRoute climb{{0,0,1},{2,0,2.5f},{2,2,3.5f},{0,2,5.5f}};
  assert(!fast_planner::withinEndpointHeights(below_floor, below_floor.front(), below_floor.back(), .4));
  assert(fast_planner::withinEndpointHeights(climb, climb.front(), climb.back(), .4));

  // A sanctioned return starts a new pass only on handoff. The same deferred
  // branch becomes an ordinary forward route on the way back.
  const FrontierRoute home{{8,0,1},{0,0,1}};
  assert(pass.allowsPreview(home, {{0,0,1},{-5,0,1}}, true, open));
  pass.commit(home, true);
  pass.observe({6,0,1});
  pass.observe({4,0,1});
  assert(pass.assess({{4,0,1},{1,0,1}}, 4., open).leg == Leg::Forward);
  assert(pass.assess({{4,0,1},{9,0,1}}, 4., open).leg == Leg::Return);
  std::cout << "PASS: corridor continuation, confirmed and high-gain returns, "
               "whole-guide retracing, measured walls, corners and stairs\n";
}
