#include <epic_planner/guide_continuity.h>
#include <cassert>
#include <iostream>
int main(){
 using Eigen::Vector3f;using Route=std::vector<Vector3f>;
 Route hook{{0,0,1},{-.2,0,1.1},{-.1,0,1.2},{.3,0,1.2},{2,0,1.2}};auto blocked=hook;
 assert(!fast_planner::forwardDeparture(blocked,Vector3f(1,0,0),[](const Route&){return false;}));
 assert(blocked.size()==hook.size());
 assert(fast_planner::forwardDeparture(hook,Vector3f(1,0,0),[](const Route& c){return c.back().x()>.2f;}));
 assert(hook[1].x()>.2f && hook.back().x()==2.f);
 Route genuine{{0,0,1},{-.2,0,1},{-2,0,1}};
 assert(!fast_planner::forwardDeparture(genuine,Vector3f(1,0,0),[](const Route&){return true;}));

 Route native{{29.201125f,-11.422495f,1.978949f},{27.147832f,-11.117373f,1.906636f},{27.02f,-10.95f,1.916f}};
 const auto original=native;
 assert((native[1]-native[0]).norm()>2.002f);
 fast_planner::boundGuideEdges(native);
 assert(native.size()==original.size()+1 && native.front()==original.front() && native.back()==original.back());
 assert(native[2]==original[1]); // Original turn preserved exactly.
 for(size_t i=1;i<native.size();++i)assert((native[i]-native[i-1]).norm()<=2.002f);
 assert((native[1]-.5f*(original[0]+original[1])).norm()<1.e-5f);
 std::cout<<"PASS: recorded 2.077m native handoff edge subdivided; corners/geometry unchanged\n";
 std::cout<<"PASS certified forward rejoin; blocked and genuine reversals preserved\n";
}
