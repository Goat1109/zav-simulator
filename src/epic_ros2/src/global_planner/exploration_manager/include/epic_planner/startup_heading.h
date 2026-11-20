#pragma once
#include <algorithm>
#include <cmath>
namespace fast_planner {
// Transport the observation heading along the guide instead of imposing the
// final yaw at its departure. Take the short angular arc across +/- pi.
inline double guideHeading(double body,double goal,double travelled,double length) {
  const double fraction=length>.01 ? std::clamp(travelled/length,0.,1.) : 0.;
  return body+fraction*std::remainder(goal-body,2.*3.14159265358979323846);
}
// Accommodate a large first heading change using short position previews and
// small yaw references. Once aligned, ordinary viewpoint handoff is unchanged.
class StartupHeading {
 public:
  double activation=1.0,release=.5,preview=.30;
  // Establish translational feedback before asking for a departure turn. Only
  // the geometric reference horizon changes; local motion limits stay intact.
  double departurePreview(double measured,double stamp,double clearance,double &goal) {
    if(!std::isfinite(stamp) || stamp<0. || !std::isfinite(clearance) || clearance<=0.)return -1.;
    if(departure_start_<0. || stamp<departure_start_)departure_start_=stamp;
    const double elapsed=stamp-departure_start_;
    if(elapsed>=4.)return 0.;
    const double fraction=std::clamp((elapsed-2.)/2.,0.,1.);
    const double error=std::remainder(goal-measured,2.*3.14159265358979323846);
    goal=measured+std::clamp(error,-preview*fraction,preview*fraction);
    return clearance*(.5+fraction);
  }
  bool apply(double measured,double &goal) {
    const double error=std::remainder(goal-measured,2.*3.14159265358979323846);
    if(!decided_) {active_=std::abs(error)>activation;decided_=true;}
    if(active_ && std::abs(error)<=release)active_=false;
    if(active_)goal=measured+std::clamp(error,-preview,preview);
    return active_;
  }
 private:
  bool decided_=false,active_=false;
  double departure_start_=-1.;
};
}
