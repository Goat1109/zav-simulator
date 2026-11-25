#!/usr/bin/env python3
"""Device-free synthetic state-machine integration (not a flight dynamics test)."""
import os,math,json,time,socket,subprocess,tempfile
from pathlib import Path
assert os.environ.get('ROS_DOMAIN_ID')=='218' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
assert all(name=='lo' for _,name in socket.if_nameindex())
assert not list(Path('/dev').glob('ttyACM*'))+list(Path('/dev').glob('ttyUSB*'))
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from zav_super.srv import GoForward
from mars_quadrotor_msgs.msg import PolynomialTrajectory as Poly


def run():
    rclpy.init();node=rclpy.create_node('synthetic_super_mission_validation')
    odom=node.create_publisher(Odometry,'/argo/super/odom',10)
    cloud=node.create_publisher(PointCloud2,'/argo/super/cloud_world',1)
    bridge=node.create_publisher(String,'/zav_real/status',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
    poly=node.create_publisher(Poly,'/planning_cmd/poly_traj',10)
    status=[];goals=[]
    node.create_subscription(String,'/zav/super/controller_status',lambda m:status.append(json.loads(m.data)),10)
    node.create_subscription(PoseStamped,'/zav/super/goal',goals.append,10)
    clients={k:node.create_client(Trigger,'/zav/super/'+k) for k in ('patrol','return_home')}
    forward=node.create_client(GoForward,'/zav/super/go_forward')
    state={'p':[1.,2.,.1],'yaw':.3,'v':0.,'armed':False,'landed':True,'mode':[3,0]}
    last_heartbeat=0.
    def publish():
        nonlocal last_heartbeat
        stamp=node.get_clock().now().to_msg();m=Odometry();m.header.frame_id='world';m.child_frame_id='argo_body';m.header.stamp=stamp
        m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=state['p']
        m.pose.pose.orientation.w=math.cos(state['yaw']/2);m.pose.pose.orientation.z=math.sin(state['yaw']/2)
        m.twist.twist.linear.x=state['v'];odom.publish(m)
        scan=PointCloud2();scan.header.frame_id='world';scan.header.stamp=stamp;scan.height=scan.width=1;cloud.publish(scan)
        bridge.publish(String(data=json.dumps({'ready':True,'latched':False,'armed':state['armed'],
            'landed':state['landed'],'landed_fresh':True,'offboard':{'source':'super','stopped':False,
            'fc_mode':state['mode'],'auto_land_disarm_s':2.}})))
        if time.monotonic()-last_heartbeat>.1:
            beat=Poly();beat.header.frame_id='world';beat.header.stamp=stamp;beat.type=Poly.HEART_BEAT;poly.publish(beat);last_heartbeat=time.monotonic()
    node.create_timer(.02,publish)
    def wait(predicate,seconds=8.):
        end=time.monotonic()+seconds
        while time.monotonic()<end and not predicate():rclpy.spin_once(node,timeout_sec=.02)
        assert predicate(), ('Timed out',status[-1] if status else None)
        assert not status or status[-1]['phase'] not in ('SOLVER_FAILURE','ODOMETRY_JUMP'),status[-1]
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    def call(name):
        future=clients[name].call_async(Trigger.Request());wait(future.done);return future.result()
    def plan(point,accepted_goal=None,request_stamp=None):
        m=Poly();m.header.frame_id='world';m.header.stamp=node.get_clock().now().to_msg()
        m.type=Poly.POSITION_TRAJ|Poly.YAW_TRAJ|Poly.HEART_BEAT;m.order_pos=m.order_yaw=7
        m.piece_num_pos=m.piece_num_yaw=1;m.time_pos=m.time_yaw=[.2]
        m.start_wt_pos=m.start_wt_yaw=node.get_clock().now().nanoseconds/1e9
        m.coef_pos_x=[0.]*7+[point[0]];m.coef_pos_y=[0.]*7+[point[1]];m.coef_pos_z=[0.]*7+[point[2]]
        m.coef_yaw=[0.]*7+[state['yaw']]
        stamp=goals[-1].header.stamp
        m.debug_info=json.dumps({'goal_request_stamp_ns':stamp.sec*10**9+stamp.nanosec if request_stamp is None else request_stamp,
                                'accepted_goal':point if accepted_goal is None else accepted_goal})
        poly.publish(m)
    def arrive(point):
        initial=state['p'][:]
        # Incremental positions avoid synthetic reset/jump; velocity is kept zero
        # because this checks supervisor transitions, not plant tracking quality.
        for j in range(1,41):
            state['p']=[a+(b-a)*j/40 for a,b in zip(initial,point)];spin(.02)
        spin(.3)
    binary=str(Path(get_package_prefix('zav_super'))/'lib/zav_super/super_ommpc')
    with tempfile.TemporaryFile(mode='w+') as log:
        process=subprocess.Popen([binary,'--ros-args','-p','execute:=true'],stdout=log,stderr=subprocess.STDOUT)
        try:
            wait(lambda:status and status[-1]['phase']=='PRESTREAM' and all(c.service_is_ready() for c in clients.values()))
            spin(.3);assert not call('return_home').success
            state['armed']=True;spin(.3);state['landed']=False;arrive([1.,2.,1.]);state['mode']=[6,0]
            wait(lambda:status[-1]['phase']=='HOVER' and status[-1]['mission']['home_valid'])
            home=status[-1]['mission']['home'];assert abs(home[2]-.1)<1e-6
            assert call('patrol').success
            expected_yaws=[.3+math.pi/2,.3,.3-math.pi/2,.3+math.pi]
            for leg in range(4):
                wait(lambda:status[-1]['phase']=='PATROL_TURN')
                state['yaw']=math.atan2(math.sin(expected_yaws[leg]),math.cos(expected_yaws[leg]))
                wait(lambda:len(goals)>=leg+1)
                goal=goals[-1];p=[goal.pose.position.x,goal.pose.position.y,goal.pose.position.z]
                error=math.atan2(goal.pose.orientation.z,goal.pose.orientation.w)*2-expected_yaws[leg]
                assert abs(math.atan2(math.sin(error),math.cos(error)))<1e-6
                if leg==0:
                    plan(p,request_stamp=-1);spin(.12)
                    assert status[-1]['phase']=='PLANNING','Unrelated goal request was accepted'
                    plan(p,accepted_goal=[p[0]+1.,p[1],p[2]]);spin(.12)
                    assert status[-1]['phase']=='PLANNING','Out-of-range goal snap was accepted'
                    accepted=[p[0]+.15,p[1],p[2]]
                    plan(accepted);arrive([accepted[0]+.12,accepted[1],accepted[2]])
                    assert status[-1]['phase']=='ENDPOINT_SETTLING',status[-1]
                    assert status[-1]['mission']['leg']==1 and not status[-1]['mission']['complete']
                    arrive([accepted[0]+.08,accepted[1],accepted[2]])
                    assert math.dist(status[-1]['goal'],accepted)<1e-9
                    print('PASS: stale goal request and excessive snap rejected; arrival uses accepted goal and requires <10cm')
                else:
                    plan(p);arrive(p)
                if leg<3:wait(lambda:status[-1]['phase']=='PATROL_TURN')
            wait(lambda:status[-1]['mission']['complete'])
            assert math.dist(state['p'][:2],home[:2])<1e-6
            # A new forward request is allowed at the prior endpoint.
            future=forward.call_async(GoForward.Request(distance_m=3.));wait(future.done);assert future.result().success
            wait(lambda:len(goals)==5);p=[goals[-1].pose.position.x,goals[-1].pose.position.y,goals[-1].pose.position.z]
            plan(p);spin(.2);state['v']=.8;state['p'][0]+=.2;spin(.1)
            assert call('return_home').success  # preempt while moving
            state['v']=0.;wait(lambda:len(goals)>=6)
            p=[goals[-1].pose.position.x,goals[-1].pose.position.y,goals[-1].pose.position.z]
            assert abs(p[2]-(home[2]+.4))<1e-6 and math.dist(p[:2],home[:2])<1e-6
            assert status[-1]['mission']['legs']==1
            plan(p);arrive(p);wait(lambda:status[-1]['phase']=='RTH_LANDING')
            assert len(goals)==6  # no intermediate Home-XY cruise waypoint
            state['mode']=[4,6];spin(.3);state['p']=[float(x) for x in home];state['landed']=True;state['armed']=False
            wait(lambda:status[-1]['phase']=='RTH_LANDED')
            print('PASS: ground Home, 4 patrol legs/outward yaw, endpoint re-use, moving RTH preemption, single 3D Home+0.4m goal, land handoff, disarm observation')
        except Exception:
            log.seek(0);print(log.read()[-12000:]);raise
        finally:
            process.terminate();process.wait(timeout=5)
    # The automatic takeoff controller is also exercised with synthetic FC state.
    state.update(p=[1.,2.,.1],yaw=.3,v=0.,armed=False,landed=True,mode=[3,0]);spin(.3);status.clear();goals.clear()
    with tempfile.TemporaryFile(mode='w+') as log:
        process=subprocess.Popen([binary,'--ros-args','-p','execute:=true','-p','auto_takeoff:=true'],stdout=log,stderr=subprocess.STDOUT)
        try:
            wait(lambda:status and status[-1]['phase']=='TAKEOFF_WAIT')
            spin(.3);state['mode']=[6,0];spin(.3);state['armed']=True
            wait(lambda:status[-1]['phase']=='TAKEOFF_CLIMB')
            assert abs(status[-1]['goal'][2]-.5)<1e-6
            state['landed']=False;arrive([1.,2.,.5])
            wait(lambda:status[-1]['phase']=='HOVER' and status[-1]['mission']['complete'])
            assert not goals,'Vertical takeoff must not publish planner goals'
            print('PASS: automatic takeoff ground anchor +0.4m, local bounded vertical profile, completed hover')
        except Exception:
            log.seek(0);print(log.read()[-12000:]);raise
        finally:
            process.terminate();process.wait(timeout=5)
    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':run()
