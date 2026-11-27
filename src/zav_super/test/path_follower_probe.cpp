#include <zav_super/path_follower.hpp>
#include <iostream>
#include <iomanip>
int main(){
  size_t n,m;double chord;if(!(std::cin>>n>>m>>chord))return 2;
  std::vector<argo_super::Waypoint> points(n);
  for(auto& p:points)std::cin>>p.position.x()>>p.position.y()>>p.position.z()>>p.yaw;
  argo_super::PathFollower follower(points,chord);
  std::cout<<std::setprecision(17);
  for(size_t i=0;i<m;++i){Eigen::Vector3d p;std::cin>>p.x()>>p.y()>>p.z()>>follower.lookahead;
    const auto t=follower.update(p);std::cout<<follower.progress()<<' '<<follower.targetArc()<<' '<<t.position.transpose()<<' '<<t.yaw<<'\n';}
}
