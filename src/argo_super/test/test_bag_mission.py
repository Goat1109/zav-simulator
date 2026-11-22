#!/usr/bin/env python3
"""Regression: moving intermediate goals, precise stopped return and abort.

An isolated ROS domain and fake landing client keep this test away from flight.
"""
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest
os.environ['ROS_DOMAIN_ID']='223'
import rclpy
from geometry_msgs.msg import Point,Vector3,PoseStamped
from nav_msgs.msg import Path as NavPath
import math

spec=importlib.util.spec_from_file_location('bag_mission',Path(__file__).resolve().parents[1]/'src/benchmark_mission.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
spec=importlib.util.spec_from_file_location('bag_publisher',Path(__file__).resolve().parents[3]/'simulator/tools/super/publish_bag_route.py')
publisher_module=importlib.util.module_from_spec(spec);spec.loader.exec_module(publisher_module)


class Landing:
    def __init__(self):self.calls=0
    def service_is_ready(self):return True
    def call_async(self,_):
        self.calls+=1
        from types import SimpleNamespace
        return SimpleNamespace(done=lambda:True,result=lambda:SimpleNamespace(success=True))


class PathProgress(unittest.TestCase):
    def test_rigid_correction_preserves_progress_and_preview(self):
        f=module.PolylineFollower([(0.,0.,1.),(0.,10.,1.),(0.,0.,1.)],4.,[0.,0.,None])
        f.update((0.,4.,1.));before=(f.progress,f.segment,f.target_arc,list(f.arc))
        f.correct_frame([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]],[2.,3.,0.])
        self.assertEqual((f.progress,f.segment,f.target_arc,f.arc),before)
        self.assertEqual(f.points[1],(-8.,3.,1.));self.assertAlmostEqual(f.yaws[0],math.pi/2)
        self.assertEqual(f.update((-2.,3.,1.)),(-6.,3.,1.))
        with self.assertRaises(ValueError):f.correct_frame([[2.,0.,0.],[0.,1.,0.],[0.,0.,1.]],[0.,0.,0.])
    def test_blocked_loop_fallback_joins_return_without_skipping_a_floor(self):
        route=[(0.,0.,1.),(0.,5.,1.),(0.,10.,1.),(0.,5.,1.),(0.,0.,1.)]
        index,removed=publisher_module.find_return_join(route,4.5,(0.,4.5,1.))
        self.assertEqual(index,3);self.assertAlmostEqual(removed,10.)
        multi_floor=[(0.,0.,1.),(0.,6.,1.),(0.,6.,5.),(0.,5.,5.),(0.,5.,1.),(0.,0.,1.)]
        self.assertIsNone(publisher_module.find_return_join(multi_floor,0.,(0.,0.,1.)))

    def test_preview_cannot_cut_stair_landing_but_straight_path_keeps_preview(self):
        f=module.PolylineFollower([(0.,0.,1.),(3.,0.,1.),(3.,3.,3.)],12.,max_chord_error=.15)
        target=f.update((0.,0.,1.))
        self.assertLess(math.dist(target,(3.,0.,1.)),.16)
        before=f.target_arc
        target=f.update((2.9,0.,1.))
        self.assertGreaterEqual(f.target_arc,before)
        self.assertGreater(target[2],2.)
        straight=module.PolylineFollower([(0.,0.,1.),(3.,0.,2.),(6.,0.,3.)],4.,max_chord_error=.15)
        self.assertAlmostEqual(math.dist(straight.update((0.,0.,1.)),(0.,0.,1.)),4.)

    def test_preview_uses_existing_braking_limits_without_changing_them(self):
        self.assertEqual(module.braking_lookahead(4.,0.,6.,10.,15.),4.)
        self.assertGreater(module.braking_lookahead(4.,5.,6.,10.,15.),8.)
        self.assertEqual(module.braking_lookahead(4.,20.,6.,10.,15.),15.)
    def test_dense_waypoints_interpolate_ahead_without_arrival_stop(self):
        f=module.PolylineFollower([(i*.5,0.,1.5)for i in range(21)],4.)
        self.assertEqual(f.update((0.,0.,1.5)),(4.,0.,1.5))
        self.assertEqual(f.update((.6,0.,1.5)),(4.6,0.,1.5))
        self.assertEqual(f.segment,1)

    def test_braking_preview_cannot_retract_a_published_target(self):
        f=module.PolylineFollower([(0.,0.,0.),(30.,0.,0.)],12.)
        first=f.update((5.,0.,0.));f.lookahead=4.
        second=f.update((6.,0.,0.))
        self.assertGreaterEqual(second[0],first[0])

    def test_exact_return_cannot_skip_outbound_or_reverse_early(self):
        f=module.PolylineFollower([(0.,0.,0.),(10.,0.,0.),(0.,0.,0.)],4.)
        self.assertEqual(f.update((0.,0.,0.)),(4.,0.,0.))
        f.update((4.,0.,0.))
        self.assertEqual(f.update((8.,0.,0.)),(10.,0.,0.))
        self.assertEqual(f.segment,0)
        self.assertLess(f.update((9.8,0.,0.))[0],9.8)
        self.assertEqual(f.segment,1)
        progress=f.progress
        f.update((9.9,0.,0.));self.assertGreaterEqual(f.progress,progress)

    def test_rounding_a_vertical_vertex_does_not_freeze_progress(self):
        f=module.PolylineFollower([(0.,0.,1.8),(2.,0.,2.3),(4.,0.,1.8),(8.,0.,1.8)],4.)
        f.update((0.,0.,1.8))
        f.update((2.6,0.,1.8))
        self.assertGreater(f.progress,2.5)
        self.assertEqual(f.segment,1)

    def test_targets_follow_ordered_corner_segments(self):
        f=module.PolylineFollower([(0.,0.,0.),(3.,0.,0.),(3.,5.,0.)],4.)
        self.assertEqual(f.update((0.,0.,0.)),(3.,1.,0.))
        self.assertEqual(f.update((3.,.2,0.)),(3.,4.2,0.))
        self.assertEqual(f.segment,1)

    def test_yaw_uses_shortest_wrap_independently_of_heading(self):
        f=module.PolylineFollower([(0.,0.,0.),(10.,0.,0.)],5.,[math.radians(170),math.radians(-170)])
        f.update((0.,0.,0.));self.assertAlmostEqual(f.target_yaw,math.pi)

    def test_invalid_vertex_is_not_silently_removed(self):
        with self.assertRaises(ValueError):module.PolylineFollower([(0.,0.,0.),(float('nan'),0.,0.),(2.,0.,0.)])


class BagMission(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();path=Path(self.tmp.name)/'mission.json'
        path.write_text(json.dumps({'takeoff':[0.,0.,1.5],'waypoints':[[0.,0.,1.5],[2.,0.,1.5]]}))
        rclpy.init(args=['--ros-args','-p',f'mission_file:={path}','-p','execute:=true'])
        self.node=module.Mission();self.node.started=True;self.node.status='READY';self.node.sent=True
        self.node.land=Landing();self.node.hold=Landing();self.node.rx=__import__('time').monotonic()
        self.node.position=Point(x=0.,y=0.,z=1.5);self.node.velocity=Vector3(x=3.,y=0.,z=0.)
    def tearDown(self):
        self.node.destroy_node();rclpy.shutdown();self.tmp.cleanup()
    def test_terminal_requires_stopped_precise_return(self):
        n=self.node;n.tick();self.assertEqual(n.index,1)
        n.sent=True;n.position.x=2.;n.velocity.x=3.;n.tick();self.assertEqual(n.index,1)
        n.velocity.x=0.;n.position.y=.6;n.tick();self.assertEqual(n.index,1)
        n.position.y=.05;n.tick();self.assertEqual(n.index,2)
        n.tick();self.assertEqual(n.phase,'COMPLETED');self.assertEqual(n.land.calls,1)
        self.assertFalse(n.get_parameter('execute').value)
    def test_stagnation_lands_without_completion(self):
        n=self.node;n.position.x=1.;n.velocity.x=0.;n.best_distance=1.
        n.progress_stamp=n.get_clock().now().nanoseconds*1e-9-61.
        n.tick();self.assertEqual(n.phase,'ABORTED');self.assertEqual(n.index,0)
        self.assertEqual(n.land.calls,1)
    def test_unchanged_path_goal_is_retried_after_transient_rejection(self):
        from types import SimpleNamespace
        n=self.node;sent=[];n.publisher=SimpleNamespace(publish=sent.append);n.velocity.x=0.
        n.follower=module.PolylineFollower([(0.,0.,1.5),(10.,0.,1.5)])
        n.goals=[(10.,0.,1.5)];n.base_lookahead=4.
        n.follow_path();self.assertEqual(len(sent),1)
        n.last_path_goal_sim=n.get_clock().now().nanoseconds*1e-9-2.
        n.follow_path();self.assertEqual(len(sent),2)
        self.assertEqual(sent[0].pose.position,sent[1].pose.position)
    def test_controller_fault_is_failure(self):
        n=self.node;n.status='FAULT: test';n.tick()
        self.assertEqual(n.phase,'ABORTED');self.assertEqual(n.land.calls,1)

    def test_bounded_corner_goal_is_not_republished_when_unchanged(self):
        from types import SimpleNamespace
        n=self.node;sent=[];n.publisher=SimpleNamespace(publish=sent.append);n.velocity.x=1.
        points=[(0.,0.,1.5),(2.,0.,1.5),(2.,2.,1.5)]
        n.follower=module.PolylineFollower(points,max_chord_error=.15)
        n.goals=points[1:];n.base_lookahead=4.
        n.follow_path();n.follow_path()
        self.assertEqual(len(sent),1)
        self.assertGreater(math.dist(n.last_target,points[-1]),1.)

    def test_exact_endpoint_is_sent_even_below_goal_update_spacing(self):
        from types import SimpleNamespace
        n=self.node;sent=[];n.publisher=SimpleNamespace(publish=sent.append);n.velocity.x=1.
        points=[(0.,0.,1.5),(2.,0.,1.5)]
        n.follower=module.PolylineFollower(points);n.goals=points[1:];n.base_lookahead=4.
        n.last_target=(1.8,0.,1.5)
        n.follow_path()
        self.assertEqual(len(sent),1)
        self.assertEqual(sent[0].pose.position.x,2.)

    def exploration_message(self):
        msg=NavPath();msg.header.frame_id='world'
        for x in [0.,2.,5.]:
            p=PoseStamped();p.pose.position.x=x;p.pose.position.z=1.5
            p.pose.orientation.z=math.sin(math.pi/4);p.pose.orientation.w=math.cos(math.pi/4)
            msg.poses.append(p)
        return msg

    def test_live_path_repeat_preserves_progress_and_yaw(self):
        n=self.node;n.route_source='external';msg=self.exploration_message();n.exploration_path(msg)
        self.assertEqual(n.route_revision,1);self.assertEqual(len(n.goals),2)
        n.follower.update((2.2,0.,1.5));progress=n.follower.progress
        self.assertAlmostEqual(n.follower.target_yaw,math.pi/2)
        follower=n.follower;target_arc=follower.target_arc
        msg.header.stamp.sec=123  # Heartbeat re-publication, same ordered path.
        n.exploration_path(msg)
        self.assertEqual(n.route_revision,1);self.assertEqual(n.follower.progress,progress)
        self.assertIs(n.follower,follower);self.assertEqual(n.follower.target_arc,target_arc)
        replacement=self.exploration_message();replacement.poses[-1].pose.position.y=1.
        n.exploration_path(replacement);self.assertEqual(n.route_revision,2)

    def test_stair_viewpoints_preserve_height_and_explicit_yaw(self):
        n=self.node;n.route_source='external';msg=NavPath();msg.header.frame_id='world'
        points=[(0.,0.,1.5,0.),(1.,0.,2.5,0.),(1.,1.,3.5,math.pi/2),(2.,1.,4.5,math.pi)]
        for x,y,z,yaw in points:
            pose=PoseStamped();pose.pose.position.x=x;pose.pose.position.y=y;pose.pose.position.z=z
            pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2)
            msg.poses.append(pose)
        n.exploration_path(msg)
        self.assertEqual(n.accepted_points,[p[:3]for p in points])
        for actual,expected in zip(n.accepted_yaws,[p[3]for p in points]):self.assertAlmostEqual(actual,expected)
        target=n.follower.update((.5,0.,2.))
        self.assertGreater(target[2],2.5)
        self.assertIsNotNone(n.follower.target_yaw)

    def test_invalid_live_route_keeps_previous_route(self):
        n=self.node;n.route_source='external';msg=self.exploration_message();n.exploration_path(msg)
        follower=n.follower;msg.header.frame_id='map';n.exploration_path(msg)
        self.assertIs(n.follower,follower)
        msg.header.frame_id='world';msg.poses[1].pose.position.x=float('nan');n.exploration_path(msg)
        self.assertIs(n.follower,follower)
        msg=self.exploration_message();msg.poses[0].pose.position.x=10.;n.exploration_path(msg)
        self.assertIs(n.follower,follower)

    def test_external_route_end_waits_without_landing(self):
        n=self.node;n.route_source='external';n.exploration_path(self.exploration_message())
        n.follower.progress=n.follower.arc[-1];n.follower.segment=len(n.follower.lengths)-1
        n.position.x=5.;n.velocity.x=0.;n.tick();n.tick();n.tick()
        self.assertEqual(n.phase,'WAITING_PATH');self.assertEqual(n.land.calls,0)
        self.assertEqual(n.hold.calls,1)
        self.assertTrue(n.get_parameter('execute').value)

    def test_continuous_stair_and_corner_vertices_do_not_request_hold(self):
        n=self.node;n.route_source='external';msg=NavPath();msg.header.frame_id='world'
        points=[(0.,0.,1.5),(1.,0.,2.5),(1.,2.,3.5),(3.,2.,3.5),(8.,2.,3.5)]
        for x,y,z in points:
            pose=PoseStamped();pose.pose.position.x=x;pose.pose.position.y=y;pose.pose.position.z=z
            pose.pose.orientation.w=1.;msg.poses.append(pose)
        n.exploration_path(msg);n.velocity.x=0.
        for x,y,z in points[1:-1]:
            n.position=Point(x=x,y=y,z=z)
            for _ in range(3):n.tick()
            self.assertEqual(n.hold.calls,0)
            self.assertNotEqual(n.phase,'WAITING_PATH')
        n.position=Point(x=8.,y=2.,z=3.5)
        for _ in range(4):n.tick()
        self.assertEqual(n.hold.calls,1);self.assertEqual(n.phase,'WAITING_PATH')
        self.assertEqual(n.land.calls,0)


if __name__=='__main__':unittest.main()
