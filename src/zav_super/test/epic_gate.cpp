#include <zav_super/epic_gate.hpp>
#include <cassert>
#include <limits>
using argo_super::EpicSwitch;
int main(){
  EpicSwitch g;
  assert(g.update(0.,true,1500,true,true)==EpicSwitch::NONE);
  assert(g.update(1.,true,1500,true,true)==EpicSwitch::NONE); // startup middle
  g.update(2.,true,1000,true,true);g.update(2.4,true,1000,true,true);
  g.update(3.,true,1500,true,true);
  assert(g.update(3.1,true,1500,true,true)==EpicSwitch::NONE); // debounce
  assert(g.update(3.4,true,1500,true,true)==EpicSwitch::START);
  assert(g.update(3.5,true,1500,false,true)==EpicSwitch::NONE); // moving is allowed
  assert(g.update(3.6,false,1500,true,true)==EpicSwitch::STOP);
  assert(g.update(4.,true,1500,true,true)==EpicSwitch::NONE);
  assert(g.update(5.,true,1500,true,true)==EpicSwitch::NONE); // no restart on RC recovery
  g.update(6.,true,2000,true,true);g.update(6.4,true,2000,true,true);
  g.update(7.,true,1500,false,true);
  assert(g.update(7.4,true,1500,false,true)==EpicSwitch::NONE); // rejected edge consumed
  assert(g.update(8.,true,1500,true,true)==EpicSwitch::NONE);
  g.update(9.,true,1000,true,true);g.update(9.4,true,1000,true,true);
  g.update(10.,true,1500,true,true);
  assert(g.update(10.4,true,1500,true,true)==EpicSwitch::START);
  assert(g.update(10.5,true,1500,true,false)==EpicSwitch::STOP);
  g.update(11.,true,1000,true,true);g.update(11.4,true,1000,true,true);
  g.update(12.,true,1500,true,true);g.update(12.4,true,1500,true,true);
  assert(g.update(12.5,true,1650,true,true)==EpicSwitch::STOP); // ambiguous PWM
  argo_super::EpicBounds bounds;
  bounds.lower={{-1.,-1.,0.},{1.,1.,0.}};bounds.upper={{1.,1.,2.},{2.,2.,2.}};
  assert(bounds.segment({0.,0.,1.},{.8,.8,1.}));
  assert(!bounds.segment({0.,0.,1.},{1.5,1.1,1.})); // both endpoints valid, gap between boxes
  assert(!bounds.contains({0.,0.,10.}));
  assert(!bounds.contains({std::numeric_limits<double>::quiet_NaN(),0.,1.}));
  assert(!bounds.segment({0.,0.,1.},{3.,0.,1.}));
  argo_super::EpicBounds joined;joined.lower={{-1.,-1.,0.},{1.,-1.,0.}};joined.upper={{1.,1.,2.},{5.,1.,2.}};
  assert(!joined.segment({0.,0.,1.},{4.,0.,1.})); // raw guide packet length cap
  assert(joined.segment({0.,0.,1.},{4.,0.,1.},10.)); // rolling target is separate
  std::array<argo_super::EpicPoint,8> c;
  for(size_t i=0;i<8;++i)c[i]={-.1+1.6*i/7.,0.,1.};
  int budget=10000;assert(joined.bezierInside(c,budget)); // shared face at a non-dyadic time
  c[4][1]=20.;budget=10000;assert(!joined.bezierInside(c,budget)); // endpoints remain inside
  argo_super::EpicBounds gap;gap.lower={{-1.,-1.,0.},{1.0001,-1.,0.}};gap.upper={{1.,1.,2.},{5.,1.,2.}};
  for(size_t i=0;i<8;++i)c[i]={-.1+1.6*i/7.,0.,1.};
  budget=10000;assert(!gap.bezierInside(c,budget));
}
