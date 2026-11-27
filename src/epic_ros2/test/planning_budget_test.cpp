#include <epic_ros2/planning_budget.hpp>
#include <cassert>
#include <thread>
#include <iostream>
int main() {
  using B=epic_ros::PlanningBudget;
  assert(!B::expired() && B::remaining(2.)==2.);
  {
    B::Scope tick(.06);auto outer=B::deadline();
    {B::Scope nested(2.);assert(B::deadline()==outer);}
    {B::Scope generation(.005);std::this_thread::sleep_for(std::chrono::milliseconds(8));assert(B::expired() && B::remaining(1.)==0.);}
    assert(!B::expired() && B::deadline()==outer);
    bool checked=false;
    std::thread worker([&]{B::Scope bind(outer);assert(B::deadline()==outer);checked=true;});worker.join();assert(checked);
  }
  assert(!B::expired() && B::remaining(2.)==2.);
  std::cout<<"PASS: nested/worker budgets share deadline; independent of ROS clock\n";
}
