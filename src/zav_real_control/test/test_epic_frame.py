import math,struct
import pytest
from zav_real_control.epic_frame import ProfileFrame
from zav_real_control.geometry import rotate


def test_first_floor_anchor_inverse_and_heading_alignment():
    reference=[-3.26924,-.05305,.18920];heading=-math.pi
    f=ProfileFrame(reference,heading,[1.,2.,.1],.4)
    assert f.forward((1.,2.,.1))==pytest.approx(reference)
    for point in ([0.,0.,0.],[2.,-5.,1.2],[-9.,6.,3.]):
        assert f.inverse(f.forward(point))==pytest.approx(point)
    q=f.quaternion((math.cos(.2),0.,0.,math.sin(.2)))
    assert rotate(q,(1.,0.,0.))==pytest.approx((math.cos(heading),math.sin(heading),0.))
    assert f.quaternion(q,inverse=True)==pytest.approx((math.cos(.2),0.,0.,math.sin(.2)))


def test_odometry_and_guide_use_inverse_frames_and_world_velocity():
    from nav_msgs.msg import Odometry,Path
    from geometry_msgs.msg import PoseStamped
    f=ProfileFrame([3.,4.,.2],math.pi/2,[1.,2.,.1],0.)
    o=Odometry();o.header.frame_id='world';o.child_frame_id='argo_body';o.pose.pose.orientation.w=1.
    o.pose.pose.position.x,o.pose.pose.position.y,o.pose.pose.position.z=1.,2.,.1
    o.twist.twist.linear.x=2.;o.twist.twist.angular.x=.3
    converted=f.odometry(o)
    assert (converted.pose.pose.position.x,converted.pose.pose.position.y,converted.pose.pose.position.z)==pytest.approx([3.,4.,.2])
    assert (converted.twist.twist.linear.x,converted.twist.twist.linear.y)==pytest.approx([0.,2.])
    assert converted.twist.twist.angular.x==.3
    p=PoseStamped();p.pose=converted.pose.pose
    m=Path();m.poses=[p];out=f.path(m).poses[0].pose
    assert (out.position.x,out.position.y,out.position.z)==pytest.approx([1.,2.,.1])
    assert out.orientation.w==pytest.approx(1.)


@pytest.mark.parametrize('big',[False,True])
def test_cloud_transform_preserves_padding_and_extra_fields(big):
    from sensor_msgs.msg import PointCloud2,PointField
    f=ProfileFrame([3.,4.,.2],math.pi/2,[1.,2.,.1],0.)
    endian='>' if big else '<'
    msg=PointCloud2(height=2,width=1,point_step=16,row_step=20,is_bigendian=big,
        data=struct.pack(endian+'ffff',1.,2.,.1,42.)+b'PAD!'+struct.pack(endian+'ffff',2.,2.,1.1,99.)+b'PAD!')
    msg.fields=[PointField(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate(('x','y','z','intensity'))]
    out=f.cloud(msg);data=bytes(out.data)
    assert struct.unpack_from(endian+'ffff',data,0)==pytest.approx([3.,4.,.2,42.])
    assert struct.unpack_from(endian+'ffff',data,20)==pytest.approx([3.,5.,1.2,99.])
    assert data[16:20]==data[36:40]==b'PAD!'
    assert bytes(msg.data)!=data
