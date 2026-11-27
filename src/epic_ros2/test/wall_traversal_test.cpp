#include <epic_ros2/wall_traversal.hpp>
#include <cassert>
#include <iostream>
int main(){
 using Journal=epic_ros::WallTraversal;using P=Journal::Point;using R=Journal::Route;
 Journal journal;R east{{0,0,1},{4,0,1}};R west{{4,0,1},{0,0,1}};
 journal.offer(east);assert(!journal.branches().empty());assert(journal.freshLength(east)>3.9f);
 journal.commit(east);assert(journal.freshLength(east)>3.9f); // planning is not visitation
 for(int i=0;i<=40;++i)journal.observe(P(i*.1f,0,1));
 assert(journal.freshLength(east)<.3f);assert(journal.freshLength(west)>3.9f); // opposite wall remains
 assert(journal.remaining(P(4,0,1)).empty());
 R short_branch{{4,0,1},{4,-1,1}};journal.offer(short_branch);assert(!journal.branches().empty());
 journal.commit(short_branch);assert(journal.remaining(P(4,-.4,1)).size()>1);
 // A nearby late segment cannot jump over the unflown turn of a U-shaped route.
 Journal u;u.commit(R{{0,0,1},{3,0,1},{3,.5,1},{0,.5,1}});
 auto remaining=u.remaining(P(.1,.15,1));assert(remaining.size()>=4);
 Journal reset;reset.observe(P(0,0,1));reset.observe(P(20,0,1));
 assert(reset.freshLength(R{{0,0,1},{20,0,1}})>19.f);
 for(float angle:{0.f,.7f,1.8f,3.1f}){
   Journal rotated;const P offset(10,-20,2),along(std::cos(angle),std::sin(angle),0);
   const R outbound{offset,offset+4*along},inbound{offset+4*along,offset};
   rotated.offer(outbound);
   for(int i=0;i<=40;++i)rotated.observe(offset+.1f*i*along);
   assert(rotated.freshLength(outbound)<.3f);assert(rotated.freshLength(inbound)>3.9f);
 }
 Journal entered;entered.proposeEntry(P(0,0,1),P(1,0,0));entered.commit(R{{0,0,1},{4,0,1}});
 assert(!entered.mayFollow(R{{2,0,1},{.1,0,1},{-.2,0,1},{-.8,0,1},{-2,0,1}}));
 assert(entered.mayFollow(R{{2,10,1},{-2,10,1}})); // another crossing is not this doorway
 entered.beginReturn();assert(entered.mayFollow(R{{2,0,1},{-2,0,1}}));
 std::cout<<"PASS directed wall visitation, short deferred branch, retained corner, odometry reset\n";
}
