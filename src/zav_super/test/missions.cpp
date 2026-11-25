#include <zav_super/missions.hpp>
#include <cassert>
int main(){
  using namespace argo_super;
  Waypoint target{{1.,0.,1.},0.};
  assert(settled({1.08,0.,1.},{.08,0.,0.},target,0.));
  assert(!settled({1.15,0.,1.},{0.,0.,0.},target,0.));
  assert(!settled({1.08,0.,1.},{.12,0.,0.},target,0.));
  for(double yaw:{0.,.8,-2.9,3.13}){
    Eigen::Vector3d o(2.,-3.,1.2);auto w=patrol(o,yaw);
    auto center=(o+w[1].position)/2.;auto previous=o;
    for(size_t i=0;i<4;++i){
      assert(std::abs((w[i].position-previous).norm()-7.)<1e-8);
      auto midpoint=(previous+w[i].position)/2.;
      auto outward=(midpoint-center).normalized();
      assert(outward.dot(Eigen::Vector3d(std::cos(w[i].yaw),std::sin(w[i].yaw),0.))>1.-1e-8);
      assert(std::abs(w[i].position.z()-o.z())<1e-8);previous=w[i].position;
    }
    assert((w.back().position-o).norm()<1e-8);
  }
  Eigen::Vector3d home(1.,2.,-.12),current(20.,-3.,1.8);
  auto r=return_home(home,current,.7);
  assert(r.size()==1);
  assert((r[0].position.head<2>()-home.head<2>()).norm()<1e-8);
  assert(std::abs(r[0].position.z()-.28)<1e-8);
  current.z()=15.;r=return_home(home,current,.7);
  assert(r.size()==1 && std::abs(r[0].position.z()-.28)<1e-8);
  current.z()=.1;r=return_home(home,current,.7);
  assert(std::abs(r[0].position.z()-.28)<1e-8);
}
