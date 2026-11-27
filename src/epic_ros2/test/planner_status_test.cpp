#include <epic_ros2/planner_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cassert>
#include <iostream>
#include <thread>
#include <atomic>
int main(int argc,char** argv){
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("offline_status_contract");
  epic_ros::PlannerStatus cache;
  assert(!cache.snapshot());
  visualization_msgs::msg::Marker marker;marker.text="PLAN_TRAJ";cache.update(marker);
  auto group=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto pub=node->create_publisher<visualization_msgs::msg::Marker>("/test/state",10);
  rclcpp::SubscriptionOptions options;options.callback_group=group;
  std::atomic<int> messages{0};std::atomic<bool> caution{false};double previous=0.,largest_gap=0.;
  auto sub=node->create_subscription<visualization_msgs::msg::Marker>("/test/state",10,[&](visualization_msgs::msg::Marker::SharedPtr m){
    const double t=rclcpp::Time(m->header.stamp).seconds();
    if(previous>0.)largest_gap=std::max(largest_gap,t-previous);
    previous=t;++messages;if(m->text=="CAUTION")caution=true;
  },options);
  auto status=node->create_wall_timer(std::chrono::milliseconds(100),[&](){
    auto snapshot=cache.snapshot();assert(snapshot);snapshot->header.stamp=node->now();pub->publish(*snapshot);
  },group);
  std::atomic<int> stage{0};
  rclcpp::TimerBase::SharedPtr blocking;
  blocking=node->create_wall_timer(std::chrono::milliseconds(200),[&](){
    if(stage==0){
      stage=1;std::this_thread::sleep_for(std::chrono::milliseconds(1800));
      assert(!caution && messages>=12);cache.update(marker);stage=2;
    }else if(stage==2){
      stage=3;std::this_thread::sleep_for(std::chrono::milliseconds(5400));
      stage=4;blocking->cancel();
    }
  });
  // Two threads, as in the hardware exploration node: one can be occupied by
  // graph search, the other services status/deadline callbacks and monitoring.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),2);
  executor.add_node(node);
  std::thread spin([&](){executor.spin();});
  const auto end=std::chrono::steady_clock::now()+std::chrono::seconds(10);
  while(stage<4 && std::chrono::steady_clock::now()<end)std::this_thread::sleep_for(std::chrono::milliseconds(20));
  executor.cancel();spin.join();
  assert(stage==4 && caution && messages>60 && largest_gap<.5);
  std::cout<<"PASS: status survives 1.8s busy planning, 5s stuck planner becomes CAUTION; largest status gap="<<largest_gap<<"s\n";
  rclcpp::shutdown();
}
