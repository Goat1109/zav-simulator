#pragma once
#include <array>
#include <algorithm>
#include <cmath>
#include <vector>

namespace argo_super {
// A middle switch at startup is never a start command. Require a stable outside
// position followed by a stable middle edge; a rejected edge is consumed.
struct EpicSwitch {
  enum Event { NONE, START, STOP };
  int zone=-1;
  double since=0.;
  bool armed=false, running=false, consumed=true;
  Event update(double now,bool fresh,int pwm,bool ready,bool healthy) {
    const int next=!fresh?-1:(pwm>=1400 && pwm<=1600)?1:
      ((pwm>=800 && pwm<=1300)||(pwm>=1700 && pwm<=2200))?0:-1;
    if(next!=zone){zone=next;since=now;consumed=false;}
    if(next<0){armed=false;consumed=true;}
    if(running && (next!=1 || !healthy)) {
      running=false;armed=false;consumed=true;return STOP;
    }
    if(next==0 && now-since>=.3)armed=true;
    if(next==1 && now-since>=.3 && !consumed) {
      consumed=true;
      const bool start=armed && ready;armed=false;
      if(start){running=true;return START;}
    }
    return NONE;
  }
  void stop(){running=false;armed=false;consumed=true;}
};
using EpicPoint=std::array<double,3>;
struct EpicBounds {
  std::vector<EpicPoint> lower,upper;
  double floor=-.4,ceiling=9.5;
  bool contains(const EpicPoint& p,double margin=0.) const {
    for(double v:p)if(!std::isfinite(v))return false;
    if(p[2]<floor-margin || p[2]>ceiling+margin)return false;
    for(size_t i=0;i<lower.size();++i) {
      bool in=true;
      for(size_t k=0;k<3;++k)in=in && p[k]>=lower[i][k]-margin && p[k]<=upper[i][k]+margin;
      if(in)return true;
    }
    return false;
  }
  bool segment(const EpicPoint& a,const EpicPoint& b) const {
    double length=0.;for(size_t k=0;k<3;++k)length+=(b[k]-a[k])*(b[k]-a[k]);
    if(!std::isfinite(length) || length>4.01)return false;
    // Clip the segment against each box, then require its whole parameter
    // interval to be covered by their union (including arbitrarily small gaps).
    std::vector<std::array<double,2>> intervals;
    for(size_t i=0;i<lower.size();++i){
      double begin=0.,end=1.;bool valid=true;
      for(size_t k=0;k<3;++k){
        const double lo=k==2?std::max(lower[i][k],floor):lower[i][k];
        const double hi=k==2?std::min(upper[i][k],ceiling):upper[i][k];
        const double delta=b[k]-a[k];
        if(std::abs(delta)<1.e-12){if(a[k]<lo || a[k]>hi)valid=false;}
        else {double x=(lo-a[k])/delta,y=(hi-a[k])/delta;if(x>y)std::swap(x,y);begin=std::max(begin,x);end=std::min(end,y);}
      }
      if(valid && begin<=end)intervals.push_back({begin,end});
    }
    std::sort(intervals.begin(),intervals.end());
    double covered=0.;
    for(const auto& interval:intervals){if(interval[0]>covered+1.e-9)return false;covered=std::max(covered,interval[1]);}
    if(covered<1.-1.e-9)return false;
    return true;
  }
};
}
