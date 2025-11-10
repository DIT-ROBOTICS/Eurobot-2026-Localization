import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point
from sensor_msgs.msg import Imu
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math
import numpy as np
from datetime import datetime  # Import for date and time
import os  # Import for file operations
import json  # Import for JSON operations
from healthcheck.ready_signal_template import ReadySignal

def rpy_from_quaternion(x, y, z, w):
    # yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return yaw

class HealthCheckNode(Node):
    def __init__(self):
        super().__init__('healthcheck_node')
        
        # Parameters V
        self.declare_parameter('robot_frame_id', 'base_footprint')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('rival_frame_id', 'rival/base_footprint')
        self.declare_parameter('lidar_frame_id', 'laser')
        # lidar param: [P_pred_linear, P_pred_angular, likelihood_threshold]
        self.declare_parameter('lidar_param_position', [0.2, 20.0, 0.8]) # [34cm, 170deg, 80%] take position only, set rotation free. we might not need this now
        self.declare_parameter('lidar_param_running', [0.1, 1.0, 0.8]) # [20cm, 35deg, 80%]
        self.declare_parameter('lidar_param_rotate', [0.5, 1.0, 0.8]) # [40cm, 35deg, 80%] take rotation only, set position free.

        self.p_robot_frame_id = self.get_parameter('robot_frame_id').get_parameter_value().string_value
        self.p_map_frame_id = self.get_parameter('map_frame_id').get_parameter_value().string_value
        self.p_rival_frame_id = self.get_parameter('rival_frame_id').get_parameter_value().string_value
        self.p_lidar_frame_id = self.get_parameter('lidar_frame_id').get_parameter_value().string_value
        self.p_lidar_param_rotate = self.get_parameter('lidar_param_rotate').get_parameter_value().double_array_value
        self.p_lidar_param_running = self.get_parameter('lidar_param_running').get_parameter_value().double_array_value
        self.p_lidar_param_position = self.get_parameter('lidar_param_position').get_parameter_value().double_array_value

        self.get_init = False
        self.odom_init = False
        self.prev_camera_pose = None

        # Subscribers: final_pose, local_filter, lidar_pose, imu/data_cov, odom2map, rival/final_pose V
        self.subscription = self.create_subscription(
            PoseStamped,
            'odom2map',
            self.odom2map_callback,
            10
        )
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'lidar_pose',
            self.lidar_pose_callback,
            10
        )
        self.subscription =self.create_subscription(
            PoseStamped,
            '/vision/aruco/robot/single/average_pose',
            self.camera_pose_callback,
            10
        )
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'initialpose',
            self.init_pose_callback,
            10
        )
        self.subscription = self.create_subscription(
            Imu,
            'imu/data_cov',
            self.imu_cov_callback,
            10
        )
        self.subscription # prevent unused variable warning

        self.init_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            'initial_pose',
            10
        )
        self.camera_pose_pub = self.create_publisher(
            PoseStamped,
            'camera_pose',
            10
        )
        self.lidar_param_pub = self.create_publisher(
            Point,
            'lidar_param_update',
            10
        )
        # TF buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Read the user input from button file
        self.read_button()

        # # Create health report file V
        self.create_health_report_file()

        # for main communication
        self.ready_signal = ReadySignal()

        self.check_localization_ok()

        # Timer for health check (3 seconds interval) V
        self.timer = self.create_timer(3.0, self.health_check_timer_callback)
        self.timer2 = self.create_timer(1.0, self.check_final_pose)
        self.timer3 = self.create_timer(0.1, self.sensor_check)
                                                                                
        self.timer3 = self.create_timer(0.1, self.sensor_check)
                                                                                
        self.wheel_slip_first = True

        # for slip estimation
        self.slip_values = []
        self.new_lidar = False
        self.new_odom = False

        self.point_msg = Point()

    def read_button(self):
        """
        Read /home/user/share/data/button.json, detect which start-button is
        pressed, and publish the corresponding initial pose.
        """
        button_file_path = '/home/user/share/data/button.json'

        # ---------- 1. Load the file ----------
        try:
            with open(button_file_path, 'r') as fp:
                btn_data = json.load(fp)
        except (OSError, json.JSONDecodeError) as e:
            self.get_logger().error(f"[read_button] Cannot read {button_file_path}: {e}")
            return

        states = btn_data.get("states", {})
        if not states:
            self.get_logger().warn("[read_button] No 'states' field in JSON")
            return

        # ---------- 2. Find the first button that is True ----------
        pressed_id = next((int(k) for k, v in states.items() if v), None)
        if pressed_id is None:
            self.get_logger().info("[read_button] No button is pressed. waiting for initial pose...")
            return

        # ---------- 3. Map button id → (x, y, yaw) ----------
        # mind blue/yellow, panda or raccoon
        start_lookup = { # x, y, z, x, y, z, w
            0:   (1.20, 0.20, 0.00, 0.00, 0.00, 0.707, 0.707),   
            1:   (0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 1.00),     
            10:  (0.35, 1.70, 0.00, 0.00, 0.00, 0.707, -0.707),
            11:  (2.70, 0.90, 0.00, 0.00, 0.00, 1.00, 0.00),
            13:  (1.28, 0.30, 0.00, 0.00, 0.00, 0.707, 0.707),   
            # team blue
            19:  (2.65, 1.70, 0.00, 0.00, 0.00, 0.707, -0.707),  
            15:  (0.30, 0.90, 0.00, 0.00, 0.00, 0.00, 1.00),     
            17:  (1.72, 0.30, 0.00, 0.00, 0.00, 0.707, 0.707),  
        }
        if pressed_id not in start_lookup:
            self.get_logger().error(f"[read_button] Button {pressed_id} not in lookup table")
            return
        
        # wait until the ekf is launched
        self.check_tf_ok()
        self.get_init = False
        self.odom_init = False
        # ---------- 4. Build & publish initial pose ----------
        init_msg = PoseWithCovarianceStamped()
        init_msg.header.stamp = self.get_clock().now().to_msg()
        init_msg.header.frame_id = self.p_map_frame_id
        init_msg.pose.pose.position.x = start_lookup[pressed_id][0]
        init_msg.pose.pose.position.y = start_lookup[pressed_id][1]
        init_msg.pose.pose.position.z = 0.0
        init_msg.pose.pose.orientation.x = start_lookup[pressed_id][3]
        init_msg.pose.pose.orientation.y = start_lookup[pressed_id][4]
        init_msg.pose.pose.orientation.z = start_lookup[pressed_id][5]
        init_msg.pose.pose.orientation.w = start_lookup[pressed_id][6]

        self.init_pub.publish(init_msg) # should be published after ekf is launched
        self.initial_pose = init_msg

        yaw = rpy_from_quaternion(
            start_lookup[pressed_id][3],
            start_lookup[pressed_id][4],
            start_lookup[pressed_id][5],
            start_lookup[pressed_id][6]
        )
        self.get_logger().info(f"[read_button] Init pose published from button {pressed_id} "
                            f"→ ({start_lookup[pressed_id][0]:.2f}, {start_lookup[pressed_id][1]:.2f}, {yaw:.2f} rad)")

        # Kick-start the localisation pipeline
        self.check_localization_ok()

    def create_health_report_file(self):
        # Generate the filename based on the current date and timeV
        now = datetime.now()
        filename = now.strftime("%Y-%m-%d_%H-%M-%S_health_report.txt")
        report_dir = '/home/user/localization-ws/src/localization-devel-ws/healthcheck/report'

        # Ensure the directory exists
        os.makedirs(report_dir, exist_ok=True)

        # Full path to the report file
        self.report_file_path = os.path.join(report_dir, filename)

        # Create the file and write the header
        with open(self.report_file_path, 'w') as file:
            file.write("Health Report\n")
            file.write(f"Generated on: {now.strftime('%Y-%m-%d %H:%M:%S')}\n")
            file.write("=" * 40 + "\n")
        self.get_logger().info(f"Health report file created: {self.report_file_path}")

    def check_localization_ok(self): #onlyOnce 
        # Conditions to satisfy for localization ok
        # 1. TF is published and without error; base_footprint and rival/base_footprint are published
        self.check_tf_ok()
        # 2. local_filter, odom2map and imu/data_cov are published(what;s the difference oddom2map and local_filter? can they be merged?)
        if hasattr(self, 'odom2map') and hasattr(self, 'local_filter') and hasattr(self, 'imu_cov'):
            self.get_logger().info("odom2map local_filter, and imu are published")
        # 3. either initial pose or camera pose is published
        if not hasattr(self, 'initial_pose') and not hasattr(self, 'camera_pose'):
            self.get_logger().warn("need inital or camera pose to initialize...")
            return False
        if not hasattr(self, 'lidar_pose'):
            # self.get_logger().warn("lidar_pose not available")
            return False
        # 4. lidar_pose is published and agree with either initial pose or camera pose (TODO)
        if hasattr(self, 'lidar_pose') and hasattr(self, 'initial_pose'):
            if np.linalg.norm(
                np.array([self.lidar_pose.pose.pose.position.x - self.initial_pose.pose.pose.position.x,
                          self.lidar_pose.pose.pose.position.y - self.initial_pose.pose.pose.position.y])
            ) >0.5:
                #recall lidar localization to calculate
                self.get_logger().warn("lidar_pose and initial pose have a large difference, republishing initial pose")
                self.init_pub.publish(self.initial_pose)
                return False
        elif hasattr(self, 'lidar_pose') and hasattr(self, 'camera_pose'):
            if np.linalg.norm(
                np.array([self.lidar_pose.pose.pose.position.x - self.camera_pose.pose.pose.position.x,
                          self.lidar_pose.pose.pose.position.y - self.camera_pose.pose.pose.position.y])
            ) > 0.2:
               #recall lidar localization to calculate
                self.get_logger().warn("lidar_pose and camera pose have a large difference, republishing camera pose")
                self.camera_pose_pub.publish(self.camera_pose)
                return False
        self.get_init = True
        #4. if localization ok, publish the initial pose from lidar_pose
        if self.get_init and not self.odom_init:
            self.init_pub.publish(self.lidar_pose)
            self.odom2map.pose.pose.position = self.lidar_pose.pose.pose.position
            self.odom2map.pose.pose.orientation = self.lidar_pose.pose.pose.orientation
            self.get_logger().info("Initial pose published from lidar_pose")
            self.odom_init = True
        # 5. if localization ok, set the lidar_param to 'running'
        self.point_msg.x = self.p_lidar_param_running[0]
        self.point_msg.y = self.p_lidar_param_running[1]
        self.point_msg.z = self.p_lidar_param_running[2]
        self.lidar_param_pub.publish(self.point_msg)
        self.get_logger().info("Lidar parameters set to running")
        # 6. response to main (a service?)
        self.ready_signal.sendReadySignal(4, 3)
        return True
    
    def health_check_timer_callback(self):
        self.dead_wheel_slip_estimation()
        # self.check_lidar_delay()

    def dead_wheel_slip_estimation(self): #V
        # Check for dead wheel slip estimation
        # Check the availability of the odom2map and lidar_pose
        if not self.get_init or not self.odom_init:
            return False
    
        if not hasattr(self, 'odom2map') or not hasattr(self, 'lidar_pose'):
            self.get_logger().warn("odom2map or lidar_pose not available")
            return False

        if not self.new_odom or not self.new_lidar:
            self.get_logger().warn("odom2map or lidar_pose not updated")
            return False
        
        # Record the timestamp of the latest lidar_pose
        lidar_time = self.lidar_pose.header.stamp

        # Use lookup_transform to get the transform between odom2map at the lidar_pose timestamp and now
        odom_tf = self.tf_buffer.lookup_transform(
            self.p_map_frame_id,
            self.p_robot_frame_id,
            lidar_time
        )
            
        if not self.wheel_slip_first:
            try:

                # Calculate the displacement from the transform
                odom_displacement_x = odom_tf.transform.translation.x - self.odom_x_prev
                odom_displacement_y = odom_tf.transform.translation.y - self.odom_y_prev

                # Calculate the displacement from lidar_pose
                lidar_displacement_x = self.lidar_pose.pose.pose.position.x - self.lidar_x_prev
                lidar_displacement_y = self.lidar_pose.pose.pose.position.y - self.lidar_y_prev

                # Compare the displacements
                slip_x = abs(odom_displacement_x - lidar_displacement_x)
                slip_y = abs(odom_displacement_y - lidar_displacement_y)
                slip_magnitude = math.sqrt(slip_x**2 + slip_y**2) # for analysis
                self.slip_values.append(slip_magnitude)

                # self.get_logger().info(f"Slip X: {slip_x}, Slip Y: {slip_y}, Magnitude: {slip_magnitude}")

                if slip_x > 0.03 or slip_y > 0.03: # TODO: more test on this, it shouldn't be so frequent!
                    self.get_logger().warn(f"Dead wheel slip detected! Slip X: {slip_x}, Slip Y: {slip_y}")
                    with open(self.report_file_path, 'a') as file:
                        file.write(f"Dead wheel slip detected! Slip X: {slip_x}, Slip Y: {slip_y}\n")


            except Exception as e:
                self.get_logger().error(f"Error during slip estimation: {e}")

        # Update previous positions
        self.lidar_x_prev = self.lidar_pose.pose.pose.position.x
        self.lidar_y_prev = self.lidar_pose.pose.pose.position.y
        self.odom_x_prev = odom_tf.transform.translation.x
        self.odom_y_prev = odom_tf.transform.translation.y
        self.wheel_slip_first = False
        self.new_odom = False
        self.new_lidar = False
        return True
    

    def check_tf_ok(self): #V
        tf_retry_count = 0
        while rclpy.ok(): # is it safe to use while loop? it is a blocking function
            tf_retry_count += 1
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))

            tf_ok = True

            # check /base_footprint, /laser, /rival/base_footprint
            try:
                self.tf_buffer.can_transform(
                    self.p_robot_frame_id,
                    self.p_map_frame_id,
                    rclpy.time.Time()
                )
            except Exception as e:
                tf_ok = False
                self.get_logger().warn(f"TF lookup failed: {e}")
            try:
                self.tf_buffer.can_transform(

                    self.p_rival_frame_id,
                    self.p_map_frame_id,
                    rclpy.time.Time()
                )
            except Exception as e:
                tf_ok = False
                self.get_logger().warn(f"TF lookup failed: {e}")

            if tf_ok:
                return True

            self.get_logger().warn("[Lidar Localization]: TF not OK")

            if tf_retry_count % 20 == 0:
                self.get_logger().error(
                    f"[Lidar Localization]: TF error after retry {tf_retry_count} times"
                )

        return False
    
    # def check_lidar_delay(self):
    #     # Check delay time for lidar, 
    #     # compare the trend of orientation because that is the most obvoius and stable data

    #     # Initialize a list to store the latest 50 orientation z data
    #     if not hasattr(self, 'orientation_z_data'):
    #         self.orientation_z_data = []

    #     # Append the latest orientation z data
    #     self.orientation_z_data.append(self.lidar_pose.pose.pose.orientation.z)

    #     # Ensure the list only keeps the latest 50 entries
    #     if len(self.orientation_z_data) > 50:
    #         self.orientation_z_data.pop(0)

    #     # Example: Log the orientation data for debugging
    #     self.get_logger().info(f"Latest orientation z data: {self.orientation_z_data}")
    #     return True

    def check_final_pose(self): #V
        if not self.get_init or not self.odom_init:
            return False
        # compare odom2map, lidar_pose and camera_pose
        # warn if any one of them has a large difference
        if not hasattr(self, 'odom2map') or not (hasattr(self, 'lidar_pose') or hasattr(self, 'camera_pose')):
            self.get_logger().warn("odom2map or (both lidar_pose and camera_pose) not available")
            return False
        
        current_time = self.get_clock().now().nanoseconds / 1e9  
        tolerance = 0.1

        def is_valid_stamp(stamp):
            t = stamp.sec + stamp.nanosec / 1e9
            return abs(current_time - t) < tolerance and t > 1e-3

        if not is_valid_stamp(self.odom2map.header.stamp):
            self.get_logger().warn("odom2map timestamp invalid or too old")
            return False

        if not is_valid_stamp(self.lidar_pose.header.stamp):
            self.get_logger().warn("lidar_pose timestamp invalid or too old")
            return False

        lidar_yaw = rpy_from_quaternion(
            self.lidar_pose.pose.pose.orientation.x,
            self.lidar_pose.pose.pose.orientation.y,
            self.lidar_pose.pose.pose.orientation.z,
            self.lidar_pose.pose.pose.orientation.w
        )
        odom_yaw = rpy_from_quaternion(
            self.odom2map.pose.pose.orientation.x,
            self.odom2map.pose.pose.orientation.y,
            self.odom2map.pose.pose.orientation.z,
            self.odom2map.pose.pose.orientation.w
        )
        # 1. compare lidar and odom2map, if they agree, fine!
        angle_diff = abs(math.atan2(math.sin(odom_yaw - lidar_yaw), math.cos(odom_yaw - lidar_yaw)))

        if np.linalg.norm(
            np.array([self.odom2map.pose.pose.position.x - self.lidar_pose.pose.pose.position.x,
                      self.odom2map.pose.pose.position.y - self.lidar_pose.pose.pose.position.y])
        ) < 0.15 and abs(angle_diff) < 0.4:
            self.point_msg.x = self.p_lidar_param_running[0]
            self.point_msg.y = self.p_lidar_param_running[1]
            self.point_msg.z = self.p_lidar_param_running[2]
            self.lidar_param_pub.publish(self.point_msg)
            self.get_logger().info("Lidar parameters set to running")
            return True 
        self.get_logger().warn("odom2map and lidar_pose have a large difference") 

        # 2. if there's no camera, just initialpub odom2map, and have lidar to try again
        if not hasattr(self, 'camera_pose'):
            self.get_logger().warn("camera_pose not available") 
            # self.init_pub.publish(self.odom2map) 
            return False
        # 3. if there's a camera, and odom2map agree with cameram, we suspect that lidar is broken
        if not is_valid_stamp(self.camera_pose.header.stamp):
            self.get_logger().warn("camera_pose timestamp invalid or too old")
            return False

        if np.linalg.norm(
            np.array([self.odom2map.pose.pose.position.x - self.camera_pose.pose.pose.pose.position.x,
                      self.odom2map.pose.pose.position.y - self.camera_pose.pose.pose.pose.position.y])
        ) < 0.05:
            self.get_logger().warn("odom2map and camera_pose are identical, republishing camera_pose")
            msg = PoseStamped()
            msg.header.stamp = self.camera_pose.header.stamp
            msg.header.frame_id = self.p_map_frame_id
            msg.pose.position = self.camera_pose.pose.pose.position
            msg.pose.orientation = self.camera_pose.pose.pose.orientation
            self.camera_pose_publication.publish(msg)
            # and set lidar param to fix rotation but ignore position
            self.point_msg.x = self.p_lidar_param_running[0]
            self.point_msg.y = self.p_lidar_param_running[1]
            self.point_msg.z = self.p_lidar_param_running[2]
            self.lidar_param_pub.publish(self.point_msg)
            # self.get_logger().info("Lidar parameters set to running")
            return False
        else:
            self.get_logger().warn("odom2map and camera_pose have a large difference")
        # 4. if there's a camera, and lidar and camera agree, we suspect that odom2map is broken
        if np.linalg.norm(
            np.array([self.lidar_pose.pose.pose.position.x - self.camera_pose.pose.pose.position.x,
                      self.lidar_pose.pose.pose.position.y - self.camera_pose.pose.pose.position.y])
        ) < 0.05:
            self.get_logger().warn("lidar_pose and camera_pose are identical, republishing camera as initial")
            msg = PoseStamped()
            msg.header.stamp = self.camera_pose.header.stamp
            msg.header.frame_id = self.p_map_frame_id
            msg.pose.position = self.camera_pose.pose.pose.position
            msg.pose.orientation = self.camera_pose.pose.pose.orientation
            self.camera_pose_pub.publish(self.camera_pose)
            # save the broken odom information in the report file
            with open(self.report_file_path, 'a') as file:
                file.write(f"odom2map: {self.odom2map.pose.position.x}, {self.odom2map.pose.position.y}\n")
                file.write(f"lidar_pose: {self.lidar_pose.pose.pose.position.x}, {self.lidar_pose.pose.pose.position.y}\n")
                file.write(f"camera_pose: {self.camera_pose.pose.pose.position.x}, {self.camera_pose.pose.pose.position.y}\n")
            return False
        else:
            self.get_logger().warn("lidar_pose, odom2map and camera_pose are all different, Splendid!")
            self.init_pub.publish(self.odom2map)
            # all three are wrong, save the information in the report file
            with open(self.report_file_path, 'a') as file:
                file.write(f"odom2map: {self.odom2map.pose.position.x}, {self.odom2map.pose.position.y}\n")
                file.write(f"lidar_pose: {self.lidar_pose.pose.pose.position.x}, {self.lidar_pose.pose.pose.position.y}\n")
                file.write(f"camera_pose: {self.camera_pose.pose.pose.position.x}, {self.camera_pose.pose.pose.position.y}\n")
            self.get_logger().warn("All three poses doesn't agree")
            return False
        
    def sensor_check(self):

        if not hasattr(self, 'imu_cov'):
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        # if current_time - self.start_time > 5:
        #     return
        def is_valid_stamp(stamp, tolerance):
            t = stamp.sec + stamp.nanosec / 1e9
            return abs(current_time - t) < tolerance and t > 1e-3
        
        if not is_valid_stamp(self.imu_cov.header.stamp, 1e-2):
            self.get_logger().warn("imu_cov timestamp invalid or too old")
            # write in the report file
            with open(self.report_file_path, 'a') as file:
                # Record the time difference between current time and imu_cov stamp
                imu_time = self.imu_cov.header.stamp.sec + self.imu_cov.header.stamp.nanosec / 1e9
                time_diff = imu_time - current_time
                file.write(f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} - Time difference current to imu_cov: {time_diff} seconds\n")
            # return False
        
        if not hasattr(self, 'odom2map'):
            return
        if not is_valid_stamp(self.odom2map.header.stamp, 1e-2):
            self.get_logger().warn("[sensor check] odom2map timestamp invalid or too old")
            with open(self.report_file_path, 'a') as file:
                # also record the time difference between imu_cov and odom2map
                odom_time = self.odom2map.header.stamp.sec + self.odom2map.header.stamp.nanosec / 1e9
                imu_time = self.imu_cov.header.stamp.sec + self.imu_cov.header.stamp.nanosec / 1e9
                time_diff = odom_time - imu_time
                file.write(f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} - Time difference imu to odom: {time_diff} seconds\n")                
            # return False
        
    def odom2map_callback(self, msg):
        self.odom2map = PoseWithCovarianceStamped()
        self.odom2map.header.stamp = msg.header.stamp
        self.odom2map.header.frame_id = self.p_map_frame_id
        self.odom2map.pose.pose.position = msg.pose.position
        self.odom2map.pose.pose.orientation = msg.pose.orientation
        self.new_odom = True

    def lidar_pose_callback(self, msg):
        self.lidar_pose = msg
        self.new_lidar = True
        if not self.get_init:
            self.check_localization_ok()

    def camera_pose_callback(self, msg):
        self.camera_pose = PoseWithCovarianceStamped()
        self.camera_pose.header.stamp = msg.header.stamp
        self.camera_pose.header.frame_id = self.p_map_frame_id
        self.camera_pose.pose.pose.position = msg.pose.position
        self.camera_pose.pose.pose.orientation = msg.pose.orientation
        if not hasattr(self, 'initial_pose') and not self.get_init:
            self.camera_pose_pub.publish(msg)
            self.check_localization_ok()

    def init_pose_callback(self, msg):
        self.get_init = False
        self.odom_init = False
        self.initial_pose = msg
        self.init_pub.publish(msg)
        self.get_logger().info("Initial pose published")
        self.check_localization_ok()

    def imu_cov_callback(self, msg):
        self.imu_cov = msg

def main(args=None):
    rclpy.init(args=args)
    node = HealthCheckNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()