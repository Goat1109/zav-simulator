#include <frontier_manager/frontier_manager.h>
#include <cassert>
#include <iostream>

// Existing friend hook used by independent measured-map regression drivers.
struct SurfaceVisibilityRegression {
 static void run(FrontierManager& m){
  m.vpp_.fixed_height_enabled_=true;m.vpp_.fixed_height_=1.4f;
  m.surface_continuity_=true;m.graph_->bubble_min_radius_=.85f;
  m.graph_->parallel_bubble_astar_=make_shared<ParallelBubbleAstar>();
  m.graph_->parallel_bubble_astar_->init(m.nh_,m.lidar_map_interface_);
  auto astar=m.graph_->parallel_bubble_astar_;
  astar->flight_min_z_=1.2;astar->flight_max_z_=1.6;astar->safe_distance_=.72;
  const float cell=m.frtp_.cell_size_;
  for(float x=-6.5f;x<-3.5f;x+=.1f)for(float y=-.5f;y<.5f;y+=.1f)for(float z=1.15f;z<1.65f;z+=.05f)
   m.observed_air_.insert((Eigen::Vector3f(x,y,z)/cell).array().floor().cast<int>());
  const auto feed=[&](bool obstruction){
   pcl::PointCloud<PointType> cloud;
   for(float x=-7.f;x<-3.f;x+=.1f)for(float y=-2.f;y<2.f;y+=.1f){
    cloud.emplace_back(x,y,.1f);cloud.emplace_back(x,y,2.7f);
   }
   if(obstruction)cloud.emplace_back(-5.f,.68f,1.6f);
   auto msg=make_shared<sensor_msgs::msg::PointCloud2>();pcl::toROSMsg(cloud,*msg);
   auto odom=make_shared<nav_msgs::msg::Odometry>();odom->pose.pose.position.x=-4.f;
   odom->pose.pose.position.z=1.4f;odom->pose.pose.orientation.w=1.;
   m.lidar_map_interface_->updateCloudMapOdometry(msg,odom);
  };
  const vector<Eigen::Vector3f> original{{-4,0,1.4f},{-4.5f,0,1.2f},{-5.5f,0,1.2f},{-6,0,1.4f}};
  m.surface_body_=original.front();feed(false);
  assert(m.certifyFlightGuide(original));
  auto level=original;m.preferLevelGuide(level);
  for(const auto& p:level)assert(std::abs(p.z()-1.4f)<1e-5f);
  assert(m.certifyFlightGuide(level));
  feed(true);assert(m.certifyFlightGuide(original));
  assert(!m.certifyFlightGuide(level));
  auto avoided=original;m.preferLevelGuide(avoided);
  for(size_t i=0;i<avoided.size();++i)assert((avoided[i]-original[i]).norm()<1e-5f);
  vector<Eigen::Vector3f> vertical{{-4,0,1.4f},{-4,0,1.25f}};
  m.preferLevelGuide(vertical);assert(vertical.back().z()<1.3f);
  std::cout<<"PASS nominal level chosen in observed clear air; obstacle avoidance and vertical return preserved\n";
 }
};
int main(int argc,char** argv){
 epic_ros::init(argc,argv,"level_guide_regression");epic_ros::NodeHandle nh;
 auto lio=make_shared<LIOInterface>();lio->init(nh);
 auto graph=make_shared<TopoGraph>();FrontierManager manager;manager.init(nh,lio,graph);
 SurfaceVisibilityRegression::run(manager);rclcpp::shutdown();
}
