import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, Point
from obstacle_detector.msg import Obstacles
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, String
from nav_msgs.msg import Odometry

import numpy as np

class LidarLocalization(Node): # inherit from Node

    def __init__(self):
        super().__init__('lidar_localization_node')

        # Declare parameters
        self.declare_parameter('side', 0)
        self.declare_parameter('debug_mode', False)
        self.declare_parameter('use_two_beacons', False)
        self.declare_parameter('visualize_candidate', True)
        self.declare_parameter('likelihood_threshold', 0.001)
        self.declare_parameter('consistency_threshold', 0.9)
        self.declare_parameter('robot_frame_id', 'base_footprint')
        self.declare_parameter('robot_parent_frame_id', 'map')
        self.declare_parameter('lidar_multiplier', 0.987)
        self.declare_parameter('likelihood_threshold_two', 0.95)
        self.declare_parameter('consistency_threshold_two', 0.99)

        # Get parameters
        self.side = self.get_parameter('side').get_parameter_value().integer_value
        self.debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value
        self.p_use_two_beacons = self.get_parameter('use_two_beacons').get_parameter_value().bool_value
        self.visualize_true = self.get_parameter('visualize_candidate').get_parameter_value().bool_value
        self.likelihood_threshold = self.get_parameter('likelihood_threshold').get_parameter_value().double_value
        self.consistency_threshold = self.get_parameter('consistency_threshold').get_parameter_value().double_value
        self.robot_frame_id = self.get_parameter('robot_frame_id').get_parameter_value().string_value
        self.robot_parent_frame_id = self.get_parameter('robot_parent_frame_id').get_parameter_value().string_value
        self.lidar_multiplier = self.get_parameter('lidar_multiplier').get_parameter_value().double_value
        self.likelihood_threshold_two = self.get_parameter('likelihood_threshold_two').get_parameter_value().double_value
        self.consistency_threshold_two = self.get_parameter('consistency_threshold_two').get_parameter_value().double_value

        # Set the landmarks map based on the side
        if self.side == 0:
            self.landmarks_map = [
                np.array([-0.094, 0.052]),
                np.array([-0.094, 1.948]),
                np.array([3.094, 1.0])
            ]
        elif self.side == 1:
            self.landmarks_map = [
                np.array([3.094, 0.052]),
                np.array([3.094, 1.948]),
                np.array([-0.094, 1.0])
            ]

        # set debug mode
        self.beacon_no = 0

        # ros settings
        self.lidar_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'lidar_pose', 10)
        self.beacons_pub = self.create_publisher(PoseArray, '/beacons_guaguagua', 10)

        if self.visualize_true:
            self.marker_array = MarkerArray()
            self.marker_num_pre = np.array([0, 0, 0])
            self.marker_id = 0
            self.circles_pub = self.create_publisher(MarkerArray, 'candidates', 10)

        self.subscription = self.create_subscription(
            Obstacles,
            'raw_obstacles',
            self.obstacle_callback,
            10)
        self.subscription = self.create_subscription( # if TF is not available
            Odometry, 
            'final_pose',
            self.pred_pose_callback,
            10
        )
        self.subscription = self.create_subscription(
            String,
            'set_lidar_side',
            self.set_lidar_side_callback,
            10
        )
        self.subscription = self.create_subscription(
            Odometry,
            'local_filter',
            self.local_callback,
            1
        )
        self.subscription = self.create_subscription(
            Point,
            'lidar_param_update',
            self.param_update_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # tf2 buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ros debug logger
        self.get_logger().debug('Lidar Localization Node has been initialized')

        self.init_landmarks_map(self.landmarks_map)
        self.robot_pose = []
        self.landmarks_set = []
        self.P_pred = np.array([[0.05**2, 0.0, 0.0], [0.0, 0.05**2, 0.0], [0.0, 0.0, 0.1]]) # TODO: tune the value, fixed for now
        self.newPose = False
        self.R = np.array([[0.001, 0.0], [0.0, 0.001]]) # measurement noise; TODO: tune the value
        self.lidar_pose_msg = PoseWithCovarianceStamped()
        self.predict_transform = None

        self.P_pred_linear = 0.8 # starting mode [0.8, 1.0, 0.8] -> [70cm, 35 deg, 80%]
        self.P_pred_angular = 1.0
        self.use_two_beacons = self.p_use_two_beacons
    
    def obstacle_callback(self, msg): # main
        self.get_logger().debug('obstacle detected')
        self.obs_raw = []
        for obs in msg.circles:
            self.obs_raw.append(np.array([obs.center.x, obs.center.y]))
        self.obs_time = msg.header.stamp
        # check if TF is available. If true, use the TF. If false, use the latest topic
        try:
            self.predict_transform = self.tf_buffer.lookup_transform(
                self.robot_parent_frame_id,
                self.robot_frame_id,
                self.obs_time
            )
            self.robot_pose = np.array([
                self.predict_transform.transform.translation.x,
                self.predict_transform.transform.translation.y,
                euler_from_quaternion(
                    self.predict_transform.transform.rotation.x,
                    self.predict_transform.transform.rotation.y,
                    self.predict_transform.transform.rotation.z,
                    self.predict_transform.transform.rotation.w
                )
            ])
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f'Could not transform {self.robot_parent_frame_id} to {self.robot_frame_id}: {e}')
            self.get_logger().debug("now try to use the latest topic")
            if self.newPose == False: 
                self.get_logger().error("no new predict topic, skip.")
                return
            
        # main: data processing
        self.landmarks_candidate = self.get_landmarks_candidate(self.landmarks_map, self.obs_raw)

        # if each of the three beacon has at least one candidate
        landmarks_with_candidates = 0
        for i in range(3):
            if len(self.landmarks_candidate[i]['obs_candidates']) > 0:
                landmarks_with_candidates += 1
        if landmarks_with_candidates == 3:
            self.landmarks_set = self.get_landmarks_set(self.landmarks_candidate)
        elif landmarks_with_candidates == 2:
            # self.get_logger().info("candidates only two beacons")
            self.get_two_beacons()
            return
        else:
            # self.get_logger().warn("less than two landmarks")
            return

        # for the normal case, complete three landmarks
        if len(self.landmarks_set) == 0: # when fail when checking consistency and likelihood
            self.get_logger().debug("empty landmarks set")
            self.get_two_beacons()
            return

        self.get_lidar_pose(self.landmarks_set, self.landmarks_map)

        # clear used data
        self.clear_data()
    
    def param_update_callback(self, msg):
        self.P_pred_linear = msg.x
        self.P_pred_angular = msg.y
        self.likelihood_threshold = msg.z

    def pred_pose_callback(self, msg):
        self.newPose = True
        orientation = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) # raw, pitch, *yaw
        # check orientation range
        if orientation < 0:
            orientation += 2 * np.pi
        self.robot_pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, orientation])
        self.P_pred = np.array([
            [self.P_pred_linear, 0, 0],
            [0, self.P_pred_linear, 0],
            [0, 0, self.P_pred_angular]
        ])
    
    def local_callback(self, msg):
        # get robot speed
        self.robot_speed = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z])
        if not self.p_use_two_beacons: return
        # if robot spin larger than 0.3, do not use two beacons
        if abs(self.robot_speed[2]) > 0.3:
            self.use_two_beacons = False
        else:
            self.use_two_beacons = True


    def set_lidar_side_callback(self, msg):
        side = msg.data.lower()
        if side in ['0', '1', 'yellow', 'blue']:
            if side == '0' or side == 'yellow':
                self.side = 0
                self.landmarks_map = [
                    np.array([-0.094, 0.052]),
                    np.array([-0.094, 1.948]),
                    np.array([3.094, 1.0])
                ]
            elif side == '1' or side == 'blue':
                self.side = 1
                self.landmarks_map = [
                    np.array([3.094, 0.052]),
                    np.array([3.094, 1.948]),
                    np.array([-0.094, 1.0])
                ]
            self.init_landmarks_map(self.landmarks_map)
            self.get_logger().debug(f"Set lidar side to {self.side}")
        else:
            self.get_logger().warn("Invalid side value")

    def init_landmarks_map(self, landmarks_map):
        self.landmarks_map = landmarks_map
        # calculate the geometry description of landmarks
        self.geometry_description_map = {}
        NUM_LANDMARKS = len(landmarks_map)
        for i in range(NUM_LANDMARKS):
            for j in range(i + 1, NUM_LANDMARKS):
                if i == j:
                    continue
                d_ij = np.linalg.norm(landmarks_map[i] - landmarks_map[j])
                self.geometry_description_map[(i, j)] = d_ij
                
    def clear_data(self):
        self.obs_raw = []
        self.robot_pose = np.array([])
        self.landmarks_candidate = []
        self.landmarks_set = []
        self.newPose = False
        self.predict_transform = None

    def get_obs_candidate(self, landmark, obs_raw):
        obs_candidates = []
        x_r, y_r, phi_r = self.robot_pose
        x_o, y_o = landmark
        r_prime = np.sqrt((x_o - x_r) ** 2 + (y_o - y_r) ** 2)
        # theta_rob = np.arctan2(
        temp = np.arctan2(y_o - y_r, x_o - x_r)
        theta_prime = angle_limit_checking(temp - phi_r)

        H = np.array([
            [-(x_o - x_r) / r_prime, -(y_o - y_r) / r_prime, 0],
            [(y_o - y_r) / r_prime ** 2, -(x_o - x_r) / r_prime ** 2, -1]
        ])
        S = H @ self.P_pred @ H.T + self.R
        S_inv = np.linalg.inv(S)

        for obs in obs_raw:
            r_z = np.sqrt(obs[0] ** 2 + obs[1] ** 2)
            theta_z = np.arctan2(obs[1], obs[0])
            y = np.array([r_z - r_prime, angle_limit_checking(theta_z - theta_prime)])
            di_square = y.T @ S_inv @ y
            likelihood = np.exp(-0.5 * di_square)
            if likelihood > self.likelihood_threshold:
                obs[0] = 0.987*obs[0]
                obs[1] = 0.987*obs[1]
                if likelihood > self.likelihood_threshold:
                    obs_candidates.append({'position': obs, 'probability': likelihood})
                else:
                    self.get_logger.info("calibrated obs likelihood very bad")
                if self.visualize_true:
                    self.visualize_candidates(obs, likelihood)

        return obs_candidates

    def get_landmarks_candidate(self, landmarks_map, obs_raw):
        landmarks_candidate = []
        self.beacon_no = 0
        for landmark in landmarks_map:
            self.beacon_no += 1
            self.marker_id = 0
            candidate = {
                'landmark': landmark,
                'obs_candidates': self.get_obs_candidate(landmark, obs_raw)
            }
            landmarks_candidate.append(candidate)

        if self.visualize_true:
            self.remove_old_markers()
            self.circles_pub.publish(self.marker_array)
            # self.get_logger().debug("Published marker array")
            self.marker_array.markers.clear() # clean up (is this enough?)

        return landmarks_candidate

    def get_landmarks_set(self, landmarks_candidate):
        landmarks_set = []
        for i in range(len(landmarks_candidate[0]['obs_candidates'])):
            for j in range(len(landmarks_candidate[1]['obs_candidates'])):
                for k in range(len(landmarks_candidate[2]['obs_candidates'])):
                    set = {
                        'beacons': {
                            0: landmarks_candidate[0]['obs_candidates'][i]['position'],
                            1: landmarks_candidate[1]['obs_candidates'][j]['position'],
                            2: landmarks_candidate[2]['obs_candidates'][k]['position']
                        }
                    }
                    # consistency of the set
                    set['consistency'] = self.get_geometry_consistency(set['beacons'])
                    if set['consistency'] < self.consistency_threshold:
                        self.get_logger().debug(f"Geometry consistency is less than {self.consistency_threshold}: {set['consistency']}")
                        continue
                    # probability of the set
                    set['probability_set'] = landmarks_candidate[0]['obs_candidates'][i]['probability'] * landmarks_candidate[1]['obs_candidates'][j]['probability'] * landmarks_candidate[2]['obs_candidates'][k]['probability']
                    landmarks_set.append(set)


        return landmarks_set

    def get_lidar_pose(self, landmarks_set, landmarks_map):
        if not landmarks_set:
            raise ValueError("landmarks_set is empty")
        # prefer the set with more beacons
        landmarks_set = sorted(landmarks_set, key=lambda x: len(x['beacons']), reverse=True)
        # with the most beacon possible, prefer the set with the highest probability_set; TODO: better way to sort?
        max_likelihood = max(set['probability_set'] for set in landmarks_set)
        max_likelihood_idx = next(i for i, set in enumerate(landmarks_set) if set['probability_set'] == max_likelihood)

        lidar_pose = np.zeros(3)
        lidar_cov = np.diag([0.05**2, 0.05**2, 0.05**2]) # what should the optimal value be?

        # If the most likely set has at least 3 beacons
        if len(landmarks_set[max_likelihood_idx]['beacons']) >= 3:
            beacons = [landmarks_set[max_likelihood_idx]['beacons'][i] for i in range(3)]
            A = np.zeros((2, 2))
            b = np.zeros(2)
            dist_beacon_robot = [np.linalg.norm(beacon) for beacon in beacons]

            A[0, 0] = 2 * (landmarks_map[0][0] - landmarks_map[2][0])
            A[0, 1] = 2 * (landmarks_map[0][1] - landmarks_map[2][1])
            A[1, 0] = 2 * (landmarks_map[1][0] - landmarks_map[2][0])
            A[1, 1] = 2 * (landmarks_map[1][1] - landmarks_map[2][1])

            b[0] = (landmarks_map[0][0]**2 - landmarks_map[2][0]**2) + (landmarks_map[0][1]**2 - landmarks_map[2][1]**2) + (dist_beacon_robot[2]**2 - dist_beacon_robot[0]**2)
            b[1] = (landmarks_map[1][0]**2 - landmarks_map[2][0]**2) + (landmarks_map[1][1]**2 - landmarks_map[2][1]**2) + (dist_beacon_robot[2]**2 - dist_beacon_robot[1]**2)

            try:
                X = np.linalg.solve(A.T @ A, A.T @ b)
                if X[0] < 0 or X[0] > 3 or X[1] < 0 or X[1] > 2:
                    return
                lidar_pose[0] = X[0]
                lidar_pose[1] = X[1]

                robot_sin = 0
                robot_cos = 0

                for i in range(3):
                    theta = angle_limit_checking(np.arctan2(landmarks_map[i][1] - lidar_pose[1], landmarks_map[i][0] - lidar_pose[0]) - np.arctan2(beacons[i][1], beacons[i][0]))
                    robot_sin += np.sin(theta)
                    robot_cos += np.cos(theta)

                    lidar_pose[2] = angle_limit_checking(np.arctan2(robot_sin, robot_cos))

                lidar_pose = self.pose_compensation(lidar_pose)

                lidar_cov[0, 0] /= (max_likelihood/1.1)
                lidar_cov[1, 1] /= (max_likelihood/1.1)
                lidar_cov[2, 2] /= (max_likelihood/1.1)
                # publish the lidar pose
                self.pub_lidar_pose(lidar_pose, lidar_cov)
                self.publish_beacons(beacons)

            except np.linalg.LinAlgError as e:
                self.get_logger().warn("Linear algebra error: {}".format(e))
            
            # use markerarray to show the landmarks it used
            if self.visualize_true:
                self.visualize_sets(beacons, max_likelihood, landmarks_set[max_likelihood_idx]['consistency'])
            
        else:
            self.get_logger().debug("not enough beacons")

        return lidar_pose, lidar_cov

    def get_two_beacons(self):

        if self.use_two_beacons == False:
            self.get_logger().debug("use_two_beacons is set to false")
            return

        indices = []
        self.get_logger().debug("get two beacons")

        # get the index of the two beacons with candidates and obstacle probability larger than 0.8
        for i in range(3):
            if any(candidate['probability'] > self.likelihood_threshold_two for candidate in self.landmarks_candidate[i]['obs_candidates']):
                indices.append(i)

        self.get_logger().debug(f"indices: {indices}")

        if len(indices) == 2:
            two_index = [indices[0], indices[1]]
            self.get_set_two(two_index)
        elif len(indices) == 3:
            for i in range(3): # 01, 12, 20
                two_index = [indices[i], indices[(i + 1) % 3]]
                self.get_set_two(two_index)

        # check if there is valid set
        if len(self.landmarks_set) == 0:
            self.get_logger().warn("no valid set")
            return

        # use the set with the highest probability
        max_likelihood = max(set['probability_set'] for set in self.landmarks_set)
        max_likelihood_idx = next(i for i, set in enumerate(self.landmarks_set) if set['probability_set'] == max_likelihood)
        beacons = [self.landmarks_set[max_likelihood_idx]['beacons'][0], self.landmarks_set[max_likelihood_idx]['beacons'][1]]
        # calculate the lidar pose
        lidar_pose = self.get_lidar_pose_two(
            indices[0],
            indices[1],
            beacons[0][0], beacons[0][1],
            beacons[1][0], beacons[1][1]
        )
        # check if the lidar pose is in the map
        if lidar_pose[0] < 0 or lidar_pose[0] > 3 or lidar_pose[1] < 0 or lidar_pose[1] > 2:
            self.get_logger().debug("lidar pose is out of map")
            return
        lidar_cov = np.diag([0.05**2, 0.05**2, 0.05**2])
        lidar_cov[0, 0] /= max_likelihood/10
        lidar_cov[1, 1] /= max_likelihood/10
        lidar_cov[2, 2] /= max_likelihood/10
        # publish the lidar pose
        self.pub_lidar_pose(lidar_pose, lidar_cov)
        if self.visualize_true:
            self.visualize_sets(beacons, max_likelihood, self.landmarks_set[max_likelihood_idx]['consistency'])
            
        # also print the likelihood
        self.get_logger().debug(f"lidar_pose (two): {lidar_pose}")
        self.get_logger().debug(f"lidar_pose (two) likelihood: {max_likelihood}")

        self.clear_data()

    def get_set_two(self, two_index):

        # nominal geometry
        nominal_distance = np.linalg.norm(self.landmarks_map[two_index[0]] - self.landmarks_map[two_index[1]])
        self.get_logger().debug(f"nominal distance: {nominal_distance:.2f}")
        angle1 = np.arctan2(self.landmarks_map[two_index[1]][1] - self.robot_pose[1], self.landmarks_map[two_index[1]][0] - self.robot_pose[0])
        angle0 = np.arctan2(self.landmarks_map[two_index[0]][1] - self.robot_pose[1], self.landmarks_map[two_index[0]][0] - self.robot_pose[0]) 
        nominal_angle = angle_limit_checking(angle1 - angle0)
        # print nominal angle, display in degree
        self.get_logger().debug(f"nominal angle: {nominal_angle * 180 / np.pi:.2f} degree")

        # get the sets
        for i in range(len(self.landmarks_candidate[two_index[0]]['obs_candidates'])):
            for j in range(len(self.landmarks_candidate[two_index[1]]['obs_candidates'])):
                set = {
                    'beacons': {
                        0: self.landmarks_candidate[two_index[0]]['obs_candidates'][i]['position'],
                        1: self.landmarks_candidate[two_index[1]]['obs_candidates'][j]['position']
                    }
                }
                # geometry consistency
                # 1. check the cross product: 'robot to beacon 0' cross 'robot to beacon 1'
                cross = np.cross(set['beacons'][0], set['beacons'][1]) # TODO: check if beacon 0 and 1 will be in the order of beacon a, b and c
                if self.side == 0: # yellow is clockwise(negative)
                    if cross > 0:
                        # self.get_logger().debug("cross product is positive")
                        continue
                elif self.side == 1: # blue is counter-clockwise(positive)
                    if cross < 0:
                        # self.get_logger().debug("cross product is negative")
                        continue
                # 2. check the distance between the two beacons
                set['consistency'] = 1 - abs(np.linalg.norm(set['beacons'][0] - set['beacons'][1]) - nominal_distance) / nominal_distance
                if set['consistency'] < self.consistency_threshold_two:
                    self.get_logger().debug(f"Geometry consistency is less than {self.consistency_threshold_two}: {set['consistency']}")
                    continue
                # 3. check the angle between the two beacons
                angle1 = np.arctan2(set['beacons'][1][1], set['beacons'][1][0])
                angle0 = np.arctan2(set['beacons'][0][1], set['beacons'][0][0])
                angle = angle_limit_checking(angle1 - angle0)
                if abs(angle_limit_checking(angle - nominal_angle)) > np.pi / 18: # 10 degree
                    self.get_logger().debug(f"Angle difference is too large: {angle * 180 / np.pi:.2f} degree")
                    continue
                # self.get_logger().info(f"angle1: {angle1 * 180 / np.pi:.2f} degree")
                # self.get_logger().info(f"angle0: {angle0 * 180 / np.pi:.2f} degree")
                # self.get_logger().info(f"cadidate angle: {angle * 180 / np.pi:.2f} degree")
                # probability of the set
                set['probability_set'] = self.landmarks_candidate[two_index[0]]['obs_candidates'][i]['probability'] * self.landmarks_candidate[two_index[1]]['obs_candidates'][j]['probability']
                if set['probability_set'] < self.likelihood_threshold_two:
                    self.get_logger().debug(f"Probability is less than {self.likelihood_threshold_two}: {set['probability_set']}")
                    continue
                self.landmarks_set.append(set)

    def get_lidar_pose_two(self, landmark_index_1, landmark_index_2, bxr, byr, cxr, cyr):

        lidar_pose = np.zeros(3)
        # the beacons position w.r.t. map
        bxm = self.landmarks_map[landmark_index_1][0]
        bym = self.landmarks_map[landmark_index_1][1]
        cxm = self.landmarks_map[landmark_index_2][0]
        cym = self.landmarks_map[landmark_index_2][1]
        x = cxm - bxm
        y = cym - bym
        xr = cxr - bxr
        yr = cyr - byr
        # calculate the angle from robot to map
        costheta = (xr*x + yr*y) / (xr**2 + yr**2)
        sintheta = (xr*y - yr*x) / (xr**2 + yr**2)
        lidar_pose[2] = np.arctan2(sintheta, costheta)
        # find the linear translation from robot to map
        # 1. transform beacon positions from w.r.t. robot to w.r.t. map
        bx = costheta * bxr - sintheta * byr
        by = sintheta * bxr + costheta * byr
        cx = costheta * cxr - sintheta * cyr
        cy = sintheta * cxr + costheta * cyr
        # 2. linear translation from robot to map, take the average of the result from the two beacons
        lidar_pose[0] = (bxm - bx)/2 + (cxm - cx)/2
        lidar_pose[1] = (bym - by)/2 + (cym - cy)/2

        return lidar_pose

    def visualize_sets(self, beacons, max_likelihood, consistency):
        marker_array = MarkerArray()
        for i, beacon in enumerate(beacons):
            marker = Marker()
            marker.header.frame_id = "base_footprint"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "chosen_landmarks"
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.01
            marker.pose.position.x = beacon[0]
            marker.pose.position.y = beacon[1]
            marker.pose.position.z = -0.1
            if len(beacons) == 3:
                marker.color = ColorRGBA(r=0.0, g=0.5, b=0.5, a=1.0)
            elif len(beacons) == 2:
                marker.color = ColorRGBA(r=0.5, g=0.5, b=0.0, a=1.0)
            marker.id = i
            marker_array.markers.append(marker)
        
        if len(beacons) == 2:
            # delete marker id 3
            marker_delete = Marker()
            marker_delete.header.frame_id = "base_footprint"
            marker_delete.ns = "chosen_landmarks"
            marker_delete.id = 3
            marker_delete.type = Marker.SPHERE
            marker_delete.action = Marker.DELETE
            marker_array.markers.append(marker_delete)

        # add the max_likelihood, consistency to the marker array
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "set_param"
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.scale.z = 0.1
        marker.pose.position.x = 3.5
        marker.pose.position.y = 2.5
        marker.pose.position.z = 0.5
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        marker.text = f"max_likelihood: {max_likelihood:.2f}, consistency: {consistency:.2f}"
        marker.id = 10
        marker_array.markers.append(marker)
        # publish the marker array
        self.circles_pub.publish(marker_array)
        self.get_logger().debug("Published marker array")
        # clean up
        marker_array.markers.clear()

    def visualize_candidates(self, obs, likelihood):

        self.marker_id += 1

        # circles
        marker = Marker()
        marker.header.frame_id = "base_footprint"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"candidates_circle{self.beacon_no}"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.01

        marker.pose.position.x = obs[0]
        marker.pose.position.y = obs[1]
        marker.pose.position.z = 0.0
        if self.beacon_no == 1:
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=likelihood)  # Red for beacon 1
        elif self.beacon_no == 2:
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=likelihood)  # Green for beacon 2
        elif self.beacon_no == 3:
            marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=likelihood)  # Blue for beacon 3
        marker.id = self.marker_id
        self.marker_array.markers.append(marker)

        # texts
        text_marker = Marker()
        text_marker.header.frame_id = "base_footprint"
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = f"candidates_text{self.beacon_no}"
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.scale.z = 0.1
        text_marker.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.2)  # White text

        text_marker.pose.position.x = obs[0] + 0.2
        text_marker.pose.position.y = obs[1]
        text_marker.pose.position.z = 0.1
        text_marker.text = f"{likelihood:.2f}"
        text_marker.id = self.marker_id
        self.marker_array.markers.append(text_marker)

    def remove_old_markers(self):
        # remove the old markers
        num_old_markers = self.marker_num_pre[self.beacon_no - 1] - self.marker_id
        for i in range(num_old_markers):
            old_marker = Marker()
            old_marker.header.frame_id = "base_footprint"
            old_marker.header.stamp = self.get_clock().now().to_msg()
            old_marker.ns = f"candidates_circle{self.beacon_no}"
            old_marker.type = Marker.SPHERE
            old_marker.action = Marker.DELETE
            old_marker.id = i + 1
            self.marker_array.markers.append(old_marker)

            old_text_marker = Marker()
            old_text_marker.header.frame_id = "base_footprint"
            old_text_marker.header.stamp = self.get_clock().now().to_msg()
            old_text_marker.ns = f"candidates_text{self.beacon_no}"
            old_text_marker.type = Marker.TEXT_VIEW_FACING
            old_text_marker.action = Marker.DELETE
            old_text_marker.id = i + 1
            self.marker_array.markers.append(old_text_marker)
        # update the marker number
        self.marker_num_pre[self.beacon_no - 1] = self.marker_id

    def get_geometry_consistency(self, beacons):
        geometry_description = {}
        consistency = 1.0
        lenB = len(beacons)

        # lenB can be 2, 3 or 4
        # use the index of the beacons to calculate the distance between them
        for i in beacons:
            for j in beacons:
                if i == j:
                    continue
                geometry_description[(i, j)] = np.linalg.norm(beacons[i] - beacons[j])
                # self.get_logger().debug(f"Beacon {i} to Beacon {j} distance: {geometry_description[(i, j)]}")
                if (i, j) in self.geometry_description_map:
                    expected_distance = self.geometry_description_map[(i, j)]
                    consistency *= 1 - np.abs(geometry_description[(i, j)] - expected_distance) / expected_distance
                # if the index is not found in map, it is probably on the lower triangle of the matrix
        
        # check the landmark sequence is correct, clockwise for yellow, counter-clockwise for blue
        if self.side == 0:
            if np.cross(beacons[1] - beacons[0], beacons[2] - beacons[0]) > 0:
                consistency = 0
        elif self.side == 1:
            if np.cross(beacons[1] - beacons[0], beacons[2] - beacons[0]) < 0:
                consistency = 0

        return consistency
    
    def pose_compensation(self, lidar_pose):
        # Use robot speed to compensate for the pose
        try:
            v_x, v_y, w = self.robot_speed
            dt = (self.get_clock().now() - rclpy.time.Time.from_msg(self.obs_time)).nanoseconds * 1e-9
            theta = lidar_pose[2]
            c_theta = np.cos(theta)
            s_theta = np.sin(theta)
            c_delta = np.cos(w * dt)
            s_delta = np.sin(w * dt)

            if abs(w) > 1e-3:
                lidar_pose[0] += (c_theta * s_delta - s_theta * (c_delta - 1)) * v_x / w - (s_theta * s_delta - c_theta * (c_delta - 1)) * v_y / w
                lidar_pose[1] += (s_theta * s_delta - c_theta * (c_delta - 1)) * v_x / w + (c_theta * s_delta - s_theta * (c_delta - 1)) * v_y / w
            else:
                lidar_pose[0] += v_x * dt * np.cos(theta + w * dt) - v_y * dt * np.sin(theta + w * dt)
                lidar_pose[1] += v_x * dt * np.sin(theta + w * dt) + v_y * dt * np.cos(theta + w * dt)

            lidar_pose[2] += w * dt
            return lidar_pose
        except Exception as e:
            self.get_logger().error(f"Error during pose compensation: {e}")
            return lidar_pose
        
    def publish_beacons(self, beacons):
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "base_footprint"  # Adjust the frame_id as needed

        for beacon in beacons:
            pose = Pose()
            pose.position.x = beacon[0]
            pose.position.y = beacon[1]
            pose.position.z = 0.0
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

        self.beacons_pub.publish(pose_array)
        self.get_logger().debug("Published beacons to /beacons_guaguagua")
    
    def pub_lidar_pose(self, lidar_pose, lidar_cov):
        # publish the lidar pose
        self.lidar_pose_msg.header.stamp = self.get_clock().now().to_msg()
        self.lidar_pose_msg.header.frame_id = 'map' #TODO: param
        self.lidar_pose_msg.pose.pose.position.x = lidar_pose[0]
        self.lidar_pose_msg.pose.pose.position.y = lidar_pose[1]
        self.lidar_pose_msg.pose.pose.position.z = 0.0
        self.lidar_pose_msg.pose.pose.orientation.x = 0.0
        self.lidar_pose_msg.pose.pose.orientation.y = 0.0
        self.lidar_pose_msg.pose.pose.orientation.z = np.sin(lidar_pose[2] / 2)
        self.lidar_pose_msg.pose.pose.orientation.w = np.cos(lidar_pose[2] / 2)
        self.lidar_pose_msg.pose.covariance = [
            lidar_cov[0, 0], 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, lidar_cov[1, 1], 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, lidar_cov[2, 2]
        ]
        self.lidar_pose_pub.publish(self.lidar_pose_msg)


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = np.cos(ai)
    si = np.sin(ai)
    cj = np.cos(aj)
    sj = np.sin(aj)
    ck = np.cos(ak)
    sk = np.sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk
    q = np.empty((4,))
    q[0] = cj * sc - sj * cs
    q[1] = cj * ss + sj * cc
    q[2] = cj * cs - sj * sc
    q[3] = cj * cc + sj * ss
    return q

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.arctan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = np.arcsin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.arctan2(t3, t4)
    return yaw_z  # in radians

def angle_limit_checking(theta):
    while theta > np.pi:
        theta -= 2 * np.pi
    while theta <= -np.pi:
        theta += 2 * np.pi
    return theta

def main(args=None):
    rclpy.init(args=args)

    lidar_localization = LidarLocalization()

    rclpy.spin(lidar_localization)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lidar_localization.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()