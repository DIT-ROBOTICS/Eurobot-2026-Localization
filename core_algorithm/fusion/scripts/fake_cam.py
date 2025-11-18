#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('fake_cam_broadcast')
        self.camera_publisher = self.create_publisher(Odometry, 'camera', 10)
        self.get_logger().info("cam publisher init")
        self.publish_camera()
    
    def publish_camera(self):
        data_list = [
            {"time": 50.0, "position": (0.4806712233883248, 1.0932451234202643, 0), "orientation": (0, 0, 0.9999016629254438, -0.014023711310927924)},
            {"time": 50.5, "position": (0.4806712233883248, 1.0932451234202643, 0), "orientation": (0, 0, 0.9999016629254438, -0.014023711310927924)},
            {"time": 51, "position": (0.4806712233883248, 1.0932451234202643, 0), "orientation": (0, 0, 0.9999016629254438, -0.014023711310927924)},
        ]

        clock = self.get_clock()
        start_time = clock.now().seconds_nanoseconds()[0]  # Seconds since epoch
        for data in data_list:
            target_time = start_time + data["time"]
            self.get_logger().info(f"Start time: {start_time}, Target time: {target_time}")

            current_time = clock.now().seconds_nanoseconds()[0]
            delay = target_time - current_time

            if delay > 0:
                self.get_logger().info(f"Sleeping for {delay} seconds to match target time")
                self.create_timer(delay, lambda: None)

            odom_msg = Odometry()
            odom_msg.header.stamp = Time(sec=int(target_time), nanosec=int((target_time % 1) * 1e9))
            odom_msg.header.frame_id = "robot/map"

            # Set position and orientation
            odom_msg.pose.pose.position.x = data["position"][0]
            odom_msg.pose.pose.position.y = data["position"][1]
            odom_msg.pose.pose.position.z = data["position"][2]
            odom_msg.pose.pose.orientation.x = data["orientation"][0]
            odom_msg.pose.pose.orientation.y = data["orientation"][1]
            odom_msg.pose.pose.orientation.z = data["orientation"][2]
            odom_msg.pose.pose.orientation.w = data["orientation"][3]

            # Publish message
            self.camera_pub.publish(odom_msg)
            self.get_logger().info(f"Published camera data at target time {target_time}")

        self.get_logger().info("Finished publishing camera data")

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()