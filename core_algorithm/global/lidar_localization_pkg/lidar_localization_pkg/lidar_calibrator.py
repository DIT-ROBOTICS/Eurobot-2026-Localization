import rclpy
from rclpy.node import Node 
from obstacle_detector.msg import Obstacles
import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64

class LidarCalibrator(Node):
    def __init__(self):
        super().__init__('lidar_calibrator')
        self.pose_sub = self.create_subscription(
            Point,
            'pose',
            self.pose_callback,
            10)

        self.obstacle_sub = self.create_subscription(
            Obstacles, 
            'raw_obstacles', 
            self.obstacle_callback, 
            10)
        
        self.distance_pub = self.create_publisher(
            Twist,
            'distance',
            10)

        self.ready_sub = self.create_subscriber(
            Int64,
            'record',
            self.ready_callback,
            10
        )

        self.marker_pub = self.create_publisher(
            Marker,
            'nearest_obstacle',
            10)

        self.target_pose = None  
        self.dis = Twist()
        self.pose = np.array([2.83, 0.03])
        self.create_timer(1, self.distance_publisher)

    def pose_callback(self, msg):
        self.target_pose = np.array([msg.x, msg.y])
        self.get_logger().info(f'Target pose set to x: {msg.x:.2f}, y: {msg.y:.2f}')

    def obstacle_callback(self, msg):
        if self.target_pose is None:
            self.get_logger().warn("No target pose received yet. Skipping obstacle processing.")
            return

        if msg.circles:  
            distances = [
                np.linalg.norm(np.array([obs.center.x, obs.center.y]) - self.target_pose)
                for obs in msg.circles
            ]
            min_index = np.argmin(distances)  
            nearest_obs = msg.circles[min_index]
            distance = distances[min_index] 

            self.publish_marker(nearest_obs.center.x, nearest_obs.center.y, nearest_obs.radius)

            ideal_vector = np.array([self.target_pose[0] - self.pose[0], self.target_pose[1] - self.pose[1]])
            nearest_obs_vector = np.array([nearest_obs.center.x - self.pose[0] , nearest_obs.center.y - self.pose[1]])
            theta = np.arctan2(ideal_vector[1], ideal_vector[0])
            self.dis.linear.x = ideal_vector[0]
            self.dis.linear.y = ideal_vector[1]
            self.dis.linear.z = nearest_obs_vector[0]
            self.dis.angular.x = nearest_obs_vector[1]
            self.dis.angular.y = ideal_vector[0]-nearest_obs_vector[0]
            self.dis.angular.z = ideal_vector[1]-nearest_obs_vector[1]
            self.get_logger().info(f"wanted obs:{self.target_pose}, nearest_obs:{nearest_obs.center.x, nearest_obs.center.y}")
            self.get_logger().info(f"ideal_vector: {ideal_vector}, nearest_obs_vector: {nearest_obs_vector}")
            self.get_logger().info(f"theta: {theta}")

    def ready_callback(self, msg):
        if msg.data==1:
            self.ready=1
        else:
            self.ready=0

    def publish_marker(self, x, y, radius):
        marker = Marker()
        marker.header.frame_id = "map"  
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "nearest_obstacle"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.scale.x = radius * 2  # 確保顯示的圓球與實際障礙物大小一致
        marker.scale.y = radius * 2
        marker.scale.z = radius * 2
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  
        marker.lifetime.sec = 1  

        self.marker_pub.publish(marker)

    def distance_publisher(self):
        if self.ready:
            self.distance_pub.publish(self.dis)

def main(args=None):
    rclpy.init(args=args)
    node = LidarCalibrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
