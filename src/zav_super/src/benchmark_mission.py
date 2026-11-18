#!/usr/bin/env python3
"""Original SUPER benchmark waypoints, gated on OMMPC's completed takeoff."""
import math
import json
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


class PolylineFollower:
    """Ordered progress and an interpolated target; never globally snap to a path.

    Sequential segments preserve order at crossings and on an exact return leg.
    A reversal is a physical turn, so lookahead cannot jump it before arrival.
    """
    def __init__(self, points, lookahead=4., yaws=None, max_chord_error=None):
        if len(points)<2 or any(len(p)!=3 or not all(math.isfinite(v)for v in p)for p in points):
            raise ValueError('Path must contain at least two finite XYZ vertices')
        self.points=[tuple(map(float,points[0]))]
        if yaws is None:yaws=[None]*len(points)
        if len(yaws)!=len(points):raise ValueError('One optional yaw is required per path vertex')
        self.yaws=[yaws[0]]
        for p,yaw in zip(points[1:],yaws[1:]):
            if math.dist(p,self.points[-1])>1e-6:
                self.points.append(tuple(map(float,p)));self.yaws.append(yaw)
        if any(len(p)!=3 or not all(math.isfinite(v)for v in p)for p in self.points):
            raise ValueError('Path must contain finite XYZ vertices')
        if any(y is not None and not math.isfinite(y)for y in self.yaws):raise ValueError('Invalid path yaw')
        if len(self.points)<2:raise ValueError('Path requires two distinct points')
        self.lengths=[math.dist(a,b)for a,b in zip(self.points[:-1],self.points[1:])]
        self.arc=[0.]
        for length in self.lengths:self.arc.append(self.arc[-1]+length)
        self.segment=0;self.progress=0.;self.target_arc=0.;self.lookahead=float(lookahead)
        if not math.isfinite(self.lookahead) or self.lookahead<=0:raise ValueError('Invalid lookahead')
        self.max_chord_error=max_chord_error
        if max_chord_error is not None and (not math.isfinite(max_chord_error) or max_chord_error<=0):
            raise ValueError('Invalid path chord error')

    def point_at(self, arc):
        segment=min(next((i for i in range(len(self.lengths))if self.arc[i+1]>=arc),len(self.lengths)-1),len(self.lengths)-1)
        f=max(0.,min(1.,(arc-self.arc[segment])/self.lengths[segment]))
        return tuple(a+(b-a)*f for a,b in zip(self.points[segment],self.points[segment+1]))

    def correct_frame(self,R,t):
        if len(R)!=3 or any(len(r)!=3 for r in R) or len(t)!=3 or not all(math.isfinite(v)for r in R for v in r) or not all(math.isfinite(v)for v in t):raise ValueError('Invalid frame correction')
        if any(abs(sum(R[k][i]*R[k][j]for k in range(3))-(1. if i==j else 0.))>1e-5 for i in range(3)for j in range(3)):raise ValueError('Non-rigid frame correction')
        self.points=[tuple(sum(R[i][j]*p[j]for j in range(3))+t[i]for i in range(3))for p in self.points]
        self.yaws=[None if y is None else math.atan2(R[1][0]*math.cos(y)+R[1][1]*math.sin(y),R[0][0]*math.cos(y)+R[0][1]*math.sin(y))for y in self.yaws]

    def bounded_preview(self, target):
        # A distant endpoint alone can bypass a stair landing before the local
        # LiDAR has observed its floor/ceiling. Bound the chord against the 3D
        # upstream polyline, without using the offline map or stopping at every
        # collinear viewpoint. Apply the same rule to progress and goal preview.
        if self.max_chord_error is None:return target
        origin=self.point_at(self.progress)
        vertices=[(s,p)for s,p in zip(self.arc,self.points)if self.progress<s<target]
        def valid(end):
            point=self.point_at(end);delta=[b-a for a,b in zip(origin,point)]
            norm=sum(d*d for d in delta)
            for s,p in vertices:
                if s>=end:break
                f=max(0.,min(1.,sum((b-a)*d for a,b,d in zip(origin,p,delta))/norm))if norm>1e-12 else 0.
                if math.dist(p,[a+f*d for a,d in zip(origin,delta)])>self.max_chord_error:return False
            return True
        previous=self.progress
        for end in [s for s,_ in vertices]+[target]:
            if not valid(end):
                low,high=previous,end
                for _ in range(20):
                    middle=(low+high)/2.
                    if valid(middle):low=middle
                    else:high=middle
                return low
            previous=end
        return target

    def update(self, position):
        # Project onto a local forward arc window. Requiring proximity to
        # every vertex traps progress when a smooth executed curve rounds a
        # corner. Never search the whole path (especially its return overlap).
        for vertex in range(self.segment+1,len(self.points)-1):
            if self.arc[vertex]>self.progress+self.lookahead:break
            u=[(b-a)/self.lengths[vertex-1]for a,b in zip(self.points[vertex-1],self.points[vertex])]
            v=[(b-a)/self.lengths[vertex]for a,b in zip(self.points[vertex],self.points[vertex+1])]
            if sum(a*b for a,b in zip(u,v))<-.5 and math.dist(position,self.points[vertex])<.35:
                self.progress=max(self.progress,self.arc[vertex]);self.segment=vertex;break
        window_end=self.bounded_preview(min(self.arc[-1],self.progress+self.lookahead))
        for vertex in range(self.segment+1,len(self.points)-1):
            if self.arc[vertex]>window_end:break
            u=[(b-a)/self.lengths[vertex-1]for a,b in zip(self.points[vertex-1],self.points[vertex])]
            v=[(b-a)/self.lengths[vertex]for a,b in zip(self.points[vertex],self.points[vertex+1])]
            if sum(a*b for a,b in zip(u,v))<-.5 and math.dist(position,self.points[vertex])>=.35:
                window_end=min(window_end,self.arc[vertex]);break
        best=None
        for segment in range(self.segment,len(self.lengths)):
            if self.arc[segment]>window_end+1e-9:break
            a,b=self.points[segment:segment+2];length=self.lengths[segment]
            direction=[(y-x)/length for x,y in zip(a,b)]
            along=sum((p-x)*d for p,x,d in zip(position,a,direction))
            low=max(0.,self.progress-self.arc[segment])
            high=min(length,window_end-self.arc[segment])
            if low>high:continue
            along=max(low,min(high,along))
            projection=[x+along*d for x,d in zip(a,direction)]
            arc=self.arc[segment]+along
            # Prefer earliest progress in distance ties on overlapping legs.
            score=(round(math.dist(position,projection),8),arc)
            if best is None or score<best[0]:best=(score,arc)
        if best is not None:self.progress=max(self.progress,best[1])
        while self.segment<len(self.lengths)-1 and self.arc[self.segment+1]<=self.progress+1e-8:
            self.segment+=1
        target_arc=self.bounded_preview(min(self.arc[-1],self.progress+self.lookahead))
        for vertex in range(self.segment+1,len(self.points)-1):
            if self.arc[vertex]>target_arc:break
            u=[(b-a)/self.lengths[vertex-1]for a,b in zip(self.points[vertex-1],self.points[vertex])]
            v=[(b-a)/self.lengths[vertex]for a,b in zip(self.points[vertex],self.points[vertex+1])]
            if sum(a*b for a,b in zip(u,v))<-.5:
                target_arc=min(target_arc,self.arc[vertex]);break
        # Shrinking braking preview must not pull the goal backwards while the
        # vehicle is decelerating. Preview advances monotonically on this route.
        target_arc=max(self.target_arc,target_arc);self.target_arc=target_arc
        segment=self.segment
        while segment<len(self.lengths)-1 and self.arc[segment+1]<target_arc:segment+=1
        fraction=max(0.,min(1.,(target_arc-self.arc[segment])/self.lengths[segment]))
        a,b=self.yaws[segment:segment+2]
        self.target_yaw=None if a is None or b is None else a+math.atan2(math.sin(b-a),math.cos(b-a))*fraction
        return tuple(a+(b-a)*fraction for a,b in zip(self.points[segment],self.points[segment+1]))


def braking_lookahead(base, speed, acceleration, jerk, maximum):
    """Zero-initial-acceleration jerk-limited stopping distance plus margin.

    This changes spatial preview, not velocity/acceleration/jerk constraints.
    A half-second preview margin covers replanning and tracking response.
    """
    duration=(2*math.sqrt(speed/jerk) if speed<=acceleration*acceleration/jerk
              else speed/acceleration+acceleration/jerk)
    return min(maximum,base+.5*speed+.5*speed*duration)


class Mission(Node):
    def __init__(self):
        super().__init__('zav_super_benchmark')
        self.declare_parameter('execute', False)
        self.declare_parameter('repetitions', 1)
        self.declare_parameter('reach_radius', .8)
        self.declare_parameter('mission_file','')
        self.declare_parameter('route_source','bag')
        self.declare_parameter('path_lookahead_m',4.)
        self.declare_parameter('path_lookahead_max_m',15.)
        self.declare_parameter('path_chord_error_m',.15)
        self.declare_parameter('path_max_acc',6.)
        self.declare_parameter('path_max_jerk',10.)
        self.route_source=self.get_parameter('route_source').value
        if self.route_source not in ('bag','external'):raise ValueError('route_source must be bag or external')
        self.mission_file=self.get_parameter('mission_file').value
        self.goals=[(0.,50.,1.5),(0.,-50.,1.5)]*self.get_parameter('repetitions').value
        if self.mission_file:
            self.route=json.loads(Path(self.mission_file).read_text())
            self.goals=[tuple(map(float,p))for p in self.route['waypoints']]
            if not self.goals or any(len(p)!=3 or not all(math.isfinite(v)for v in p)for p in self.goals):
                raise ValueError('Mission must contain finite XYZ waypoints')
        self.follower=None;self.last_target=None
        self.accepted_points=None;self.accepted_yaws=None;self.route_signature=None;self.route_revision=0;self.path_dirty=True
        if self.route_source=='external':self.goals=[]
        elif self.mission_file and 'path_lookahead_m' in self.route:
            self.accepted_points=[self.route['takeoff'],*self.goals]
            self.accepted_yaws=self.route.get('yaw_waypoints',[None]*len(self.accepted_points))
            self.follower=PolylineFollower(self.accepted_points,self.route['path_lookahead_m'],self.accepted_yaws,
                                           self.get_parameter('path_chord_error_m').value)
        self.base_lookahead=self.follower.lookahead if self.follower else self.get_parameter('path_lookahead_m').value
        self.phase='WAITING';self.reason='';self.best_distance=float('inf');self.progress_stamp=None
        self.path_progress_stamp_sim=None
        self.mission_pub=self.create_publisher(String,'/zav/super/mission_status',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.path_pub=self.create_publisher(NavPath,'/mission/global_path',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.index=0;self.sent=False;self.ready=False;self.started=False;self.future=None
        self.position=None;self.velocity=None;self.rx=0.;self.status='';self.last_goal=0.
        self.publisher=self.create_publisher(PoseStamped,'/planning/click_goal',10)
        self.markers=self.create_publisher(MarkerArray,'/mission/mkr',10)
        self.start=self.create_client(Trigger,'/zav/super/start')
        self.land=self.create_client(Trigger,'/zav/super/land')
        self.hold=self.create_client(Trigger,'/zav/super/hold');self.hold_future=None
        self.create_subscription(Odometry,'/zav/super/odom',self.odom,10)
        self.create_subscription(String,'/zav/super/controller_status',self.state,10)
        self.last_frame_correction=0
        self.create_subscription(String,'/mission/bag_frame_correction',self.frame_correction,10)
        self.create_subscription(PoseStamped,'/goal_pose',self.click,10)
        self.create_subscription(NavPath,'/mission/exploration_path',self.exploration_path,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.2,self.tick)

    def exploration_path(self,msg):
        if self.route_source!='external':return
        try:
            if msg.header.frame_id!='world':raise ValueError('Exploration path must be in world')
            points=[];yaws=[]
            for pose in msg.poses:
                if pose.header.frame_id not in ('','world'):raise ValueError('Mixed pose frames')
                p=pose.pose.position;q=pose.pose.orientation
                points.append((p.x,p.y,p.z))
                norm=math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
                if norm<1e-9:yaws.append(None)
                elif not math.isfinite(norm) or abs(norm-1.)>.01:raise ValueError('Invalid orientation quaternion')
                else:yaws.append(math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
            candidate=PolylineFollower(points,self.get_parameter('path_lookahead_m').value,yaws,
                                        self.get_parameter('path_chord_error_m').value)
            signature=(tuple(candidate.points),tuple(candidate.yaws))
            if signature==self.route_signature:return
            if self.position is None:raise ValueError('Wait for body odometry before sending a path')
            position=(self.position.x,self.position.y,self.position.z)
            if math.dist(position,candidate.points[0])>2.:
                raise ValueError('A replacement path must start within 2m of the current body position')
            if self.phase=='ABORTED':raise ValueError('Mission is aborted; restart before replacing its route')
            self.follower=candidate;self.goals=candidate.points[1:]
            self.accepted_points=candidate.points;self.accepted_yaws=candidate.yaws
            self.route_signature=signature;self.route_revision+=1
            self.path_dirty=True
            self.index=0;self.last_target=None;self.best_distance=float('inf');self.progress_stamp=None
            self.path_progress_stamp_sim=None
            self.hold_future=None
            if self.started:self.phase='RUNNING'
            self.get_logger().info(f'Accepted exploration route {self.route_revision}: {len(self.goals)} waypoints, {candidate.arc[-1]:.2f}m')
        except (ValueError,IndexError,TypeError)as exc:
            self.get_logger().error(f'Rejected exploration path: {exc}')

    def odom(self,msg):
        self.position=msg.pose.pose.position;self.velocity=msg.twist.twist.linear;self.rx=time.monotonic()

    def frame_correction(self,msg):
        if not self.route.get('bag_map_alignment',False) or self.route_source!='external' or self.follower is None:return
        try:
            m=json.loads(msg.data)
            if m['revision']!=self.route_revision or m['sequence']<=self.last_frame_correction:return
            self.follower.correct_frame(m['rotation'],m['translation'])
            self.last_frame_correction=m['sequence'];self.goals=self.follower.points[1:]
            self.accepted_points=self.follower.points;self.accepted_yaws=self.follower.yaws
            self.route_signature=(tuple(self.accepted_points),tuple(self.accepted_yaws))
            self.last_target=None;self.path_dirty=True
        except (ValueError,KeyError,TypeError):self.get_logger().error('Rejected bag frame correction')

    def state(self,msg): self.status=msg.data

    def click(self,msg):
        # Interactive goals share the same frame as the planner.
        if self.route_source=='bag' and self.follower is None and msg.header.frame_id=='world':
            self.publisher.publish(msg)

    def report(self):
        self.mission_pub.publish(String(data=json.dumps({'phase':self.phase,'reason':self.reason,
            'reached_waypoints':self.index,'total_waypoints':len(self.goals),'mission_file':self.mission_file,
            'route_source':self.route_source,'route_revision':self.route_revision,
            'path_progress_stamp_sim':self.path_progress_stamp_sim,
            'path_progress_m':self.follower.progress if self.follower else None})))
        if self.path_dirty and (self.mission_file or self.accepted_points is not None):
            path=NavPath();path.header.frame_id='world';path.header.stamp=self.get_clock().now().to_msg()
            points=self.accepted_points if self.accepted_points is not None else self.route.get('reference_path',[self.route['takeoff'],*self.goals])
            yaws=self.accepted_yaws if self.accepted_yaws is not None else [None]*len(points)
            for goal,yaw in zip(points,yaws):
                pose=PoseStamped();pose.header=path.header
                if yaw is not None:pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2)
                pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=goal;path.poses.append(pose)
            self.path_pub.publish(path)
            self.path_dirty=False

    def abort(self,reason):
        self.phase='ABORTED';self.reason=reason;self.report()
        self.get_logger().error(f'Mission aborted: {reason}')
        if self.land.service_is_ready():self.land.call_async(Trigger.Request())
        self.set_parameters([rclpy.parameter.Parameter('execute',value=False)])

    def tick(self):
        self.report()
        if self.position is None or time.monotonic()-self.rx>1.: return
        if not self.get_parameter('execute').value: return
        if not self.started:
            if self.future is None and self.start.service_is_ready(): self.future=self.start.call_async(Trigger.Request())
            elif self.future is not None and self.future.done():
                self.started=self.future.result().success;self.future=None
                if self.started:self.phase='RUNNING'
            return
        if (self.mission_file or self.route_source=='external') and self.status.startswith(('FAULT','STOPPED')):
            self.abort('Controller reported '+self.status);return
        if self.status not in ('READY','TRACKING'): return
        if self.route_source=='external' and self.follower is None:return
        if self.index>=len(self.goals):
            if self.route_source=='external':self.phase='WAITING_PATH';return
            if self.land.service_is_ready():
                self.phase='COMPLETED';self.report()
                self.land.call_async(Trigger.Request());self.get_logger().info('Benchmark complete; OMMPC landing requested')
                self.set_parameters([rclpy.parameter.Parameter('execute',value=False)])
            return
        if self.follower is not None:
            self.follow_path();return
        x,y,z=self.goals[self.index]
        if not self.sent:
            msg=PoseStamped();msg.header.frame_id='world';msg.header.stamp=self.get_clock().now().to_msg()
            msg.pose.position.x=x;msg.pose.position.y=y;msg.pose.position.z=z;msg.pose.orientation.w=float('nan')
            self.publisher.publish(msg);self.sent=True;self.last_goal=time.monotonic()
            self.best_distance=float('inf');self.progress_stamp=self.get_clock().now().nanoseconds*1e-9
            self.get_logger().info(f'Waypoint {self.index+1}/{len(self.goals)}: {(x,y,z)}')
            markers=MarkerArray()
            for i,goal in enumerate(self.goals):
                marker=Marker();marker.header=msg.header;marker.ns='benchmark';marker.id=i;marker.type=Marker.SPHERE
                marker.pose.position.x,marker.pose.position.y,marker.pose.position.z=goal;marker.pose.orientation.w=1.
                marker.scale.x=marker.scale.y=marker.scale.z=.4
                marker.color.g=1.;marker.color.a=.8;markers.markers.append(marker)
            self.markers.publish(markers)
        distance=math.dist((self.position.x,self.position.y,self.position.z),(x,y,z))
        now=self.get_clock().now().nanoseconds*1e-9
        if distance<self.best_distance-.15:self.best_distance=distance;self.progress_stamp=now
        if self.mission_file and self.progress_stamp is not None and now-self.progress_stamp>60.:
            self.abort('No waypoint progress for 60 simulation seconds');return
        terminal=self.index==len(self.goals)-1
        radius=.15 if self.mission_file and terminal else self.get_parameter('reach_radius').value
        if distance<radius:
            speed=math.sqrt(self.velocity.x**2+self.velocity.y**2+self.velocity.z**2)
            if (self.mission_file and not terminal) or speed<.7:
                self.index+=1;self.sent=False

    def follow_path(self):
        if self.hold_future is not None:
            if self.hold_future.done():
                if self.hold_future.result().success:self.index=len(self.goals)
                self.hold_future=None
            return
        position=(self.position.x,self.position.y,self.position.z)
        speed=math.sqrt(self.velocity.x**2+self.velocity.y**2+self.velocity.z**2)
        self.follower.lookahead=braking_lookahead(self.base_lookahead,speed,
            self.get_parameter('path_max_acc').value,self.get_parameter('path_max_jerk').value,
            self.get_parameter('path_lookahead_max_m').value)
        target=self.follower.update(position)
        self.index=self.follower.segment
        now=self.get_clock().now().nanoseconds*1e-9
        self.path_progress_stamp_sim=now
        remaining=self.follower.arc[-1]-self.follower.progress
        if self.progress_stamp is None or remaining<self.best_distance-.15:
            self.best_distance=remaining;self.progress_stamp=now
        if now-self.progress_stamp>60.:
            self.abort('No path progress for 60 simulation seconds');return
        yaw=self.follower.target_yaw
        previous_yaw=getattr(self,'last_yaw',None)
        yaw_changed=(yaw is None)!=(previous_yaw is None) or (yaw is not None and previous_yaw is not None and abs(math.atan2(math.sin(yaw-previous_yaw),math.cos(yaw-previous_yaw)))>.1)
        # A corner can bound preview before the endpoint even when the remaining
        # arc is shorter than braking lookahead. Do not clear the last published
        # target there: it needlessly restarts planning on every mission tick.
        endpoint_pending=math.dist(target,self.goals[-1])<1e-6 and self.last_target!=self.goals[-1]
        if endpoint_pending:target=self.goals[-1]
        if self.last_target is None or math.dist(target,self.last_target)>.3 or yaw_changed or endpoint_pending or (speed<.3 and now-getattr(self,'last_path_goal_sim',-math.inf)>1.):
            msg=PoseStamped();msg.header.frame_id='world';msg.header.stamp=self.get_clock().now().to_msg()
            msg.pose.position.x,msg.pose.position.y,msg.pose.position.z=target
            if yaw is None:msg.pose.orientation.w=float('nan')
            else:msg.pose.orientation.z=math.sin(yaw/2);msg.pose.orientation.w=math.cos(yaw/2)
            self.publisher.publish(msg);self.last_target=target;self.last_yaw=yaw;self.last_path_goal_sim=now
            self.get_logger().info(f'Path progress {self.follower.progress:.2f}/{self.follower.arc[-1]:.2f}m; lookahead {target}')
        speed=math.sqrt(self.velocity.x**2+self.velocity.y**2+self.velocity.z**2)
        if self.follower.segment==len(self.follower.lengths)-1 and math.dist(position,self.goals[-1])<.15:
            if self.route_source=='external':
                if speed<.3 and self.hold.service_is_ready():
                    self.hold_future=self.hold.call_async(Trigger.Request())
            elif speed<.7:self.index=len(self.goals)



def main():
    rclpy.init();node=Mission()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
if __name__=='__main__': main()
