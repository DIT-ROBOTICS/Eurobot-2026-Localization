#!/usr/bin/env python3
import math
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q
class TestBroadcaster(Node):
    def __init__(self):
        super().__init__('ekf_br')
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.parent_frame_id = 'robot/map'
        self.child_frame_id = 'robot/base_footprint'
        
        self.X = np.array([0.5, 0.4, 0.0])

        self.ekf_pose = self.create_publisher(Odometry, 'final_pose', 10)
        self.footprint_publish()
    
    def footprint_publish(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame_id
        t.child_frame_id = self.child_frame_id
        t.transform.translation.x = self.X[0]
        t.transform.translation.y = self.X[1]
        t.transform.translation.z = 0.0
        quat = quaternion_from_euler(0, 0, self.X[2])
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_static_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    br = TestBroadcaster()
    rclpy.spin(br)  # Keep the node running
    rclpy.shutdown()  # Shut down the ROS 2 client

if __name__ == '__main__':
    main()
