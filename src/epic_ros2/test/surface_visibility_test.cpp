#include <epic_planner/corridor_pass.h>
#include <frontier_manager/frontier_manager.h>
#include <cassert>
#include <atomic>
#include <thread>
#include <pcl_conversions/pcl_conversions.h>

struct SurfaceVisibilityRegression {
  static void run(FrontierManager &m) {
    auto node=m.nh_.native();
    auto mode=[&](bool enabled) {
      node->set_parameter(rclcpp::Parameter("FrontierManager.measured_surface_visibility",enabled));
    };
    PointVector targets;
    for(int y=100;y<=101;++y)for(int z=7;z<=8;++z) {
      PointType p;m.idx2pos({100,y,z},p);targets.push_back(p);
      ByteArrayRaw key;m.pos2bytes(p,key);m.frtd_.label_map_[key]=FRONTIER_DIS;
      PointType front;m.idx2pos({101,y,z},front);
      ByteArrayRaw front_key;m.pos2bytes(front,front_key);m.frtd_.label_map_[front_key]=DENSE;
    }
    const float plane_x=targets.front().x+.5f*m.frtp_.cell_size_;
    auto add_plane=[&](float x) {
      pcl::PointCloud<PointType> cloud;
      for(float y=-1;y<2;y+=.1f)for(float z=0;z<3;z+=.1f)cloud.emplace_back(x,y,z);
      auto msg=make_shared<sensor_msgs::msg::PointCloud2>();pcl::toROSMsg(cloud,*msg);
      auto odom=make_shared<nav_msgs::msg::Odometry>();odom->pose.pose.position.x=5.2;
      odom->pose.pose.position.z=1.2;odom->pose.pose.orientation.w=1.;
      m.lidar_map_interface_->updateCloudMapOdometry(msg,odom);
    };
    add_plane(plane_x);
    auto cluster=make_shared<ClusterInfo>();
    m.compute_cluster_info(targets,vector<Eigen::Vector3f>(4,Eigen::Vector3f::UnitX()),cluster);
    m.graph_->odom_node_=make_shared<TopoNode>();
    m.graph_->odom_node_->center_=cluster->center_+Eigen::Vector3f(5,0,0);
    m.graph_->odom_node_->yaw_=0.;
    auto score=[&](float side) {
      cluster->is_dormant_=false;cluster->vp_clusters_.clear();
      ViewpointCluster group;group.center_=cluster->center_+Eigen::Vector3f(side*4,0,0);
      group.vps_.emplace_back(group.center_.x(),group.center_.y(),group.center_.z());group.distance_=4.;
      cluster->vp_clusters_.push_back(group);m.selectBestViewpoint(cluster);
      return cluster->is_reachable_;
    };
    mode(false);assert(!score(1)); // The same wall's adjacent voxel hides its own target.
    assert(score(-1)); // Legacy sign accepts the reverse side of that surface.
    mode(true);assert(score(1));
    auto &poses = cluster->vp_clusters_.front().vps_;
    const PointType first = poses.front();
    poses.emplace_back(first.x, first.y + .6f, first.z);
    poses.emplace_back(first.x, first.y - .6f, first.z);
    m.selectBestViewpoint(cluster);
    assert(cluster->is_reachable_ && !cluster->alternatives_.empty());
    for (const auto &view : cluster->alternatives_) {
      assert(view.visible_area > 0.f && std::isfinite(view.yaw));
      assert((view.position - cluster->best_vp_).norm() >= .2f);
    }
    assert(!score(-1));
    // A second physical plane inside the local voxel band must STILL occlude.
    add_plane(plane_x+.25f);
    assert(!score(1));
    // A level corridor must not invent a stair through an unseen ceiling.
    // A real, previously unvisited, rising volume needs traversing LiDAR rays.
    const Eigen::Vector3f origin(8.1f, 5.1f, 1.4f);
    PointVector rays;
    for (float y = -1.f; y <= 1.f; y += .1f) rays.emplace_back(14.1f, 5.1f + y, 1.4f);
    m.recordFreeRays(origin, rays);
    assert(m.risingOpenings(origin, .8f).empty());
    assert(!m.observedFree({15.f, 5.1f, 1.4f}));
    const auto allow = [](const Eigen::Vector3f &) {return false;};
    auto passage = m.levelPassage(origin,Eigen::Vector3f::UnitX(),.8f,allow);
    assert(passage.size()>2 && passage.back().x() > origin.x()+3.f);
    for (const auto &p : passage) assert(m.observedFree(p) && std::abs(p.z()-origin.z())<.01f);
    assert(m.levelPassage(origin,Eigen::Vector3f::UnitX(),.8f,
        [&](const Eigen::Vector3f &p) {return p.x()>origin.x()+.9f;}).empty());
    auto clipped=m.levelPassage(origin,Eigen::Vector3f::UnitX(),.8f,
        [&](const Eigen::Vector3f &p) {return p.x()>origin.x()+1.8f;});
    for(const auto &p:clipped) assert(p.x()<=origin.x()+1.8f);
    for (const auto &p : passage) m.observation_positions_.insert((p/m.frtp_.cell_size_).array().floor().cast<int>());
    assert(m.levelPassage(origin,Eigen::Vector3f::UnitX(),.8f,allow).empty());
    const Eigen::Vector3f upstairs = origin + Eigen::Vector3f(0,0,4);
    assert(m.landingPassages(upstairs,.8f).empty());
    PointVector upper_rays = rays;
    for (auto &ray : upper_rays) ray.z += 4;
    m.recordFreeRays(upstairs,upper_rays);
    assert(m.levelPassage(upstairs,Eigen::Vector3f::UnitX(),.8f,allow).size()>2);
    auto outlets=m.landingPassages(upstairs,.8f);
    assert(!outlets.empty());
    for(const auto &p:outlets) assert(m.observedFree(p) && p.z()>upstairs.z()-1.2f);
    // A long corridor may continue beyond the local clustering window.
    // A lower-floor visit alone cannot establish that upper connection.
    const Eigen::Vector3f distant=origin+Eigen::Vector3f(0,15,0);
    m.observation_positions_.insert((distant/m.frtp_.cell_size_).array().floor().cast<int>());
    auto has_distant = [&](const auto &candidates) {
      return std::any_of(candidates.begin(),candidates.end(),[&](const Eigen::Vector3f &p){
        return (p.head<2>()-distant.head<2>()).norm()<1.f;
      });
    };
    assert(!has_distant(m.landingPassages(upstairs,.8f)));
    PointVector down_hall;
    for (float dx=-.6f;dx<=.6f;dx+=.08f)
      down_hall.emplace_back(upstairs.x()+dx,upstairs.y()+17.f,upstairs.z());
    m.recordFreeRays(upstairs,down_hall);
    assert(has_distant(m.landingPassages(upstairs,.8f)));
    PointVector other_side;
    for (float y=-1.f;y<=1.f;y+=.1f) other_side.emplace_back(upstairs.x()-6.f,upstairs.y()+y,upstairs.z());
    m.recordFreeRays(upstairs,other_side);
    vector<Eigen::Vector3f> branches;
    const auto forward=m.levelPassage(upstairs,Eigen::Vector3f::UnitX(),.8f,allow,.9f,false,&branches);
    assert(forward.size()>2 && forward.back().x()>upstairs.x()+3.f);
    assert(std::any_of(branches.begin(),branches.end(),[&](const auto &p){return p.x()<upstairs.x()-3.f;}));
    rays.clear();
    for (float y = -1.5f; y <= 1.5f; y += .1f)
      for (float z = 2.f; z < 4.f; z += .1f) rays.emplace_back(14.1f, 5.1f + y, origin.z() + z);
    m.recordFreeRays(origin, rays);
    const auto openings = m.risingOpenings(origin, .8f);
    assert(!openings.empty());
    for (const auto &p : openings) assert(m.observedFree(p) && p.z() >= origin.z() + .8f);
    assert(!m.risingOpenings(origin + Eigen::Vector3f(0,-14,0), .8f).empty());
    // The centre of a measured 1.55 m doorway cannot admit a 0.80 m
    // planning radius. The finer pass must preserve the requested 0.70 m
    // clearance and reach the wider space on the far side.
    const Eigen::Vector3f entry(2.2f,14.2f,6.2f);
    pcl::PointCloud<PointType> doorway;
    for(float x=4.3f;x<=5.4f;x+=.08f) for(float z=4.8f;z<=7.6f;z+=.08f)
      for(float side:{-1.f,1.f}) doorway.emplace_back(x,entry.y()+side*.775f,z);
    auto cloud=make_shared<sensor_msgs::msg::PointCloud2>(); pcl::toROSMsg(doorway,*cloud);
    auto pose=make_shared<nav_msgs::msg::Odometry>(); pose->pose.pose.position.x=entry.x();
    pose->pose.pose.position.y=entry.y(); pose->pose.pose.position.z=entry.z(); pose->pose.pose.orientation.w=1.;
    m.lidar_map_interface_->updateCloudMapOdometry(cloud,pose);
    PointVector lane;
    for(float y=-.45f;y<=.45f;y+=.05f) for(float z=-.25f;z<=.25f;z+=.05f)
      lane.emplace_back(entry.x()+8.f,entry.y()+y,entry.z()+z);
    m.recordFreeRays(entry,lane);
    const auto wide=m.levelPassage(entry,Eigen::Vector3f::UnitX(),.8f,allow,.9f);
    assert(wide.empty() || wide.back().x()<4.4f);
    const auto narrow=m.levelPassage(entry,Eigen::Vector3f::UnitX(),.7f,allow,.9f,false,nullptr,true);
    assert(!narrow.empty() && narrow.back().x()>7.f);
    for(const auto &p:narrow) assert(m.observedFree(p) && m.lidar_map_interface_->getDisToOcc(p)>.7f);
    const Eigen::Vector3f inside(4.6f,14.2f,6.2f), across=inside+Eigen::Vector3f(0,1.6f,0);
    auto cell=[&](const Eigen::Vector3f &p)->Eigen::Vector3i {
      return (p/m.frtp_.cell_size_).array().floor().cast<int>();
    };
    m.observation_positions_.clear();
    m.observation_positions_.insert(cell(inside));
    for (float dy=0.f;dy<1.9f;dy+=.1f) m.observed_free_cells_.insert(cell(inside+Eigen::Vector3f(0,dy,0)));
    assert(m.recentlySurveyed(inside+Eigen::Vector3f(0,.3f,0),2.8f,1.f,.32f));
    assert(m.recentlySurveyed(across,2.8f,1.f));
    assert(!m.recentlySurveyed(across,2.8f,1.f,.32f));
    const Eigen::Vector3f near_surface=inside+Eigen::Vector3f(0,.60f,0);
    assert(!m.recentlySurveyed(near_surface,2.8f,1.f,.32f));
    assert(m.inspectedPassage(near_surface,.8f));
    assert(!m.inspectedPassage(inside+Eigen::Vector3f(0,1.12f,0),.8f));
    assert(!m.inspectedPassage(near_surface+Eigen::Vector3f(0,0,3.f),.8f));
    const Eigen::Vector3f measured(8.f,8.f,2.f),eye(4.f,8.f,2.f);
    PointVector measured_returns;measured_returns.emplace_back(measured.x(),measured.y(),measured.z());
    for(int k=0;k<4;++k)m.recordFreeRays(eye,measured_returns);
    assert(!m.adequatelyObservedSurface(measured,-Eigen::Vector3f::UnitX()));
    m.recordFreeRays(eye+Eigen::Vector3f(0,1.f,0),measured_returns);
    assert(m.adequatelyObservedSurface(measured,-Eigen::Vector3f::UnitX()));
    assert(!m.adequatelyObservedSurface(measured,Eigen::Vector3f::UnitX()));
    pcl::PointCloud<PointType> evidence_points;evidence_points.push_back(measured_returns.front());
    auto evidence_cloud=make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(evidence_points,*evidence_cloud);
    auto evidence_pose=make_shared<nav_msgs::msg::Odometry>();
    evidence_pose->pose.pose.position.x=eye.x();evidence_pose->pose.pose.position.y=eye.y();
    evidence_pose->pose.pose.position.z=eye.z();evidence_pose->pose.pose.orientation.w=1.;
    m.lidar_map_interface_->updateCloudMapOdometry(evidence_cloud,evidence_pose);
    assert(m.adequatelyObservedSurface(measured-Eigen::Vector3f(.15f,0,0),-Eigen::Vector3f::UnitX()));
    assert(!m.adequatelyObservedSurface(measured-Eigen::Vector3f(.35f,0,0),-Eigen::Vector3f::UnitX()));
    // Worker threads must see exactly the serial observation evidence while
    // concurrently populating a cold shared cache with different queries.
    vector<Eigen::Vector3f> queries;
    vector<std::array<bool,2>> expected;
    for(int i=0;i<128;++i) {
      queries.push_back(measured-Eigen::Vector3f(.4f*i/127.f,0,0));
      expected.push_back({m.adequatelyObservedSurface(queries.back(),-Eigen::Vector3f::UnitX()),
                          m.adequatelyObservedSurface(queries.back(),Eigen::Vector3f::UnitX())});
    }
    m.surface_completion_cache_.clear();
    std::atomic<size_t> mismatch{0};
    vector<std::thread> workers;
    for(int worker=0;worker<4;++worker) workers.emplace_back([&,worker] {
      for(int repeat=0;repeat<8;++repeat)for(size_t j=0;j<queries.size();++j) {
        const size_t i=(j+31*worker)%queries.size();
        if(m.adequatelyObservedSurface(queries[i],-Eigen::Vector3f::UnitX())!=expected[i][0])++mismatch;
        if(m.adequatelyObservedSurface(queries[i],Eigen::Vector3f::UnitX())!=expected[i][1])++mismatch;
      }
    });
    for(auto &worker:workers)worker.join();
    assert(mismatch==0);
    m.recordFreeRays(eye,measured_returns);
    auto verifier=make_shared<ParallelBubbleAstar>();
    verifier->init(m.nh_,m.lidar_map_interface_);
    for(float dx:{-1.f,-.45f,0.f,.2f,.65f,1.f})
      for(float dy:{-.7f,0.f,.3f,.9f}) for(float dz:{-.8f,0.f,.4f,.7f})
        for(float margin:{.65f,.8f,1.2f}) {
          const Eigen::Vector3f q=measured+Eigen::Vector3f(dx,dy,dz);
          assert(m.observationRayBlocked(q,margin)==(m.lidar_map_interface_->getDisToOcc(q)<.5f*margin));
          const Eigen::Vector3f next=q+Eigen::Vector3f(.10f,.05f,-.05f);
          const bool overlap=fast_planner::clearanceBallsOverlap((next-q).norm(),
              m.lidar_map_interface_->getDisToOcc(q),m.lidar_map_interface_->getDisToOcc(next),margin);
          if(overlap) {
            vector<Eigen::Vector3f> edge{q,next};
            assert(verifier->collisionCheck_shortenPath(edge,margin));
          }
        }
    assert(!m.adequatelyObservedSurface(measured+Eigen::Vector3f(0,0,3),-Eigen::Vector3f::UnitX()));
    // A lower-floor track is a suggestion, not an unexplored upper room.
    // With all measured air known, projecting the track upstairs must not
    // create another survey. Genuine newly exposed upper air reopens it.
    auto closed_lio=make_shared<LIOInterface>();closed_lio->init(m.nh_);
    auto closed_graph=make_shared<TopoGraph>();
    FrontierManager closed;closed.init(m.nh_,closed_lio,closed_graph);
    closed_lio->lp_->box_num_=1;closed_lio->lp_->dead_area_num_=0;
    closed_lio->lp_->global_box_min_boundary_vec_={{0,0,0}};
    closed_lio->lp_->global_box_max_boundary_vec_={{12,8,8}};
    const float grid=closed.frtp_.cell_size_;
    closed.observation_positions_.insert((Eigen::Vector3f(8.2f,4.2f,1.4f)/grid).array().floor().cast<int>());
    auto fill_air=[&](bool hidden) {
      closed.observed_free_cells_={};closed.view_gain_cache_.clear();
      for(int x=0;x<int(12/grid);++x)for(int y=0;y<int(8/grid);++y)for(int z=0;z<int(8/grid);++z)
        if(!hidden || (x+.5f)*grid<10.f)closed.observed_free_cells_.insert({x,y,z});
    };
    const Eigen::Vector3f upper_body(1.4f,4.2f,5.4f);
    fill_air(false);assert(closed.landingPassages(upper_body,.8f).empty());
    fill_air(true);assert(!closed.landingPassages(upper_body,.8f).empty());
    mode(false);
    std::cout<<"PASS: adjacent self-surface voxel, front/back sign, and genuine intervening wall\n";
  }
};

int main(int argc,char **argv) {
  epic_ros::init(argc,argv,"surface_visibility_regression");epic_ros::NodeHandle nh;
  auto lio=make_shared<LIOInterface>();lio->init(nh);
  auto graph=make_shared<TopoGraph>();FrontierManager manager;manager.init(nh,lio,graph);
  SurfaceVisibilityRegression::run(manager);rclcpp::shutdown();
}
