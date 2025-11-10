#include "rival_localization/rival_localization.h"

Rival::Rival() : Node("rival_localization"){

    IMM imm;
    model = imm;
    initial = true;

    initialize();
}

void Rival::initialize() {

    this->declare_parameter<std::string>("robot_name", "robot");
    this->declare_parameter<std::string>("rival_name", "rival");
    this->declare_parameter<double>("frequency", 10.);
    // for play area
    this->declare_parameter<double>("x_max", 3.);
    this->declare_parameter<double>("x_min", 0.);
    this->declare_parameter<double>("y_max", 2.);
    this->declare_parameter<double>("y_min", 0.);
    // for obstacle tracking (within_lock)
    this->declare_parameter<double>("vel_lpf_gain", 0.9);
    this->declare_parameter<double>("locking_rad", 0.3);
    this->declare_parameter<double>("lockrad_growing_rate", 0.3);
    // for is_me
    this->declare_parameter<double>("is_me", 0.3);
    // weights for the three sensors (the weight will be normalized depending on the combination)
    this->declare_parameter<double>("cam_weight", 2.0);
    this->declare_parameter<double>("obs_weight", 6.0);
    this->declare_parameter<double>("side_weight", 2.0);
    // threshold for comparing poses
    this->declare_parameter<double>("cam_side_threshold", 0.2);
    this->declare_parameter<double>("side_obs_threshold", 0.2);
    this->declare_parameter<double>("obs_cam_threshold", 0.2);
    // a list of crossed areas, each area as [x_min, x_max, y_min, y_max]
    // This is stored as a flat vector, but you can interpret it as an n x 4 matrix in your code
    this->declare_parameter<std::vector<double>>(
        "crossed_areas", 
        {2.25, 3.0, 0.0, 0.15,
         2.55, 3.0, 0.65, 1.1}
    );

    
    robot_name           = this->get_parameter("robot_name").get_value<std::string>();
    rival_name           = this->get_parameter("rival_name").as_string();
    freq                 = this->get_parameter("frequency").as_double();
    // for play area
    x_max                = this->get_parameter("x_max").as_double();
    x_min                = this->get_parameter("x_min").as_double();
    y_max                = this->get_parameter("y_max").as_double();
    y_min                = this->get_parameter("y_min").as_double();
    // for obstacle tracking (within_lock)
    vel_lpf_gain         = this->get_parameter("vel_lpf_gain").as_double();
    p_locking_rad        = this->get_parameter("locking_rad").as_double(); // but what if rival is moving?? should increase if rival's moving!
    lockrad_growing_rate = this->get_parameter("lockrad_growing_rate").as_double(); // 5e-2 meter per second
    // for is_me
    p_is_me              = this->get_parameter("is_me").as_double();
    // weights for the three sensors (the weight will be normalized depending on the combination)
    cam_weight           = this->get_parameter("cam_weight").as_double();
    obs_weight           = this->get_parameter("obs_weight").as_double();
    side_weight          = this->get_parameter("side_weight").as_double();
    // threshold for comparing poses
    cam_side_threshold   = this->get_parameter("cam_side_threshold").as_double();
    side_obs_threshold   = this->get_parameter("side_obs_threshold").as_double();
    obs_cam_threshold    = this->get_parameter("obs_cam_threshold").as_double();
    // a list of crossed areas, each area as [x_min, x_max, y_min, y_max]
    crossed_areas = this->get_parameter("crossed_areas").as_double_array();
    
    // RCLCPP_INFO(this->get_logger(),"robot_name: %s, rival_name: %s", robot_name.c_str(), rival_name.c_str());

    obstacles_sub = this->create_subscription<obstacle_detector::msg::Obstacles>("obstacles_to_map", 10, std::bind(&Rival::obstacles_callback, this, _1));
    cam_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/ceiling_rival/pose", 10, std::bind(&Rival::cam_callback, this, _1));
    side_obstacle_sub = this->create_subscription<obstacle_detector::msg::Obstacles>("/side/side_obstacles_to_map", 10, std::bind(&Rival::side_obstacles_callback, this, _1));
    robot_pose_sub = this->create_subscription<nav_msgs::msg::Odometry>("final_pose", 10, std::bind(&Rival::robot_pose_callback, this, _1));
    rival_raw_pub = this->create_publisher<nav_msgs::msg::Odometry>("raw_pose", 10);
    rival_final_pub = this->create_publisher<nav_msgs::msg::Odometry>("rhino_pose", 10);

    br = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / freq), std::bind(&Rival::timerCallback, this));
  
    obstacle_ok = false;
    locking_rad = p_locking_rad;
    my_pose.x = 10, my_pose.y = 10;
}        

bool Rival::in_playArea_obs(geometry_msgs::msg::Point center) {

    bool ok = true;

    if (center.x > x_max || center.x < x_min) ok = false;
    if (center.y > y_max || center.y < y_min) ok = false;

    return ok;
}

bool Rival::in_crossed_area(geometry_msgs::msg::Point center) {

    bool tf = false;
    // given a n * 4 matrix, it represents the crossed area of rectangle areas
    // if the center is in any of the areas, return false
    for (size_t i = 0; i < crossed_areas.size(); i += 4) {
        if (center.x > crossed_areas[i] && center.x < crossed_areas[i + 1] && center.y > crossed_areas[i + 2] && center.y < crossed_areas[i + 3]) {
            tf = true;
            break;
        }
    }

    return tf;
}

bool Rival::within_lock(geometry_msgs::msg::Point pre, geometry_msgs::msg::Point cur, double dt) {

    bool ok = true;

    locking_rad = locking_rad + sqrt(pow(rival_final_vel.x, 2) + pow(rival_final_vel.y, 2)) * dt;
    double distance = sqrt(pow((pre.x - cur.x), 2) + pow((pre.y - cur.y), 2));

    if (distance > locking_rad) ok = false;

    return ok;
}

bool Rival::is_me(geometry_msgs::msg::Point center) {

    bool tf = false;
    if (sqrt(pow((center.x - my_pose.x), 2) + pow((center.y - my_pose.y), 2)) < p_is_me) tf = true;

    return tf;
}

geometry_msgs::msg::Vector3 Rival::lpf(double gain, geometry_msgs::msg::Vector3 pre, geometry_msgs::msg::Vector3 cur) {

    geometry_msgs::msg::Vector3 out;
    out.x = gain * cur.x + (1.0 - gain) * pre.x;
    out.y = gain * cur.y + (1.0 - gain) * pre.y;

    return out;
}

void Rival::imm_filter() {

    double px, py, vx, vy;
    
    px = rival_raw_pose.x;
    py = rival_raw_pose.y;
    vx = rival_raw_vel.x;
    vy = rival_raw_vel.y;
    
    if (!initial) {

        Eigen::VectorXd z;

        z.resize(4);

        z << px, py, vx, vy;

        model.updateOnce(rival_stamp.seconds(), &z);
    }
    else {

        Eigen::VectorXd x;
        ModelGenerator gen;

        x.resize(6);

        //initial state
        x << px, py, vx, vy, 0, 0;

        gen.generateIMMModel(rival_stamp.seconds(), x, model);

        initial = false;
    }

    rival_final_pose.x = model.x_[0];
    rival_final_pose.y = model.x_[1];
    rival_final_vel.x  = model.x_[2];
    rival_final_vel.y  = model.x_[3];
}

void Rival::cam_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {

    cam_rival_pose.x = msg->pose.position.x;
    cam_rival_pose.y = msg->pose.position.y;
    cam_rival_pose.z = msg->pose.position.z;
    cam_stamp = msg->header.stamp;
}

void Rival::obstacles_callback(const obstacle_detector::msg::Obstacles::SharedPtr msg) {

    static bool first = false;
    static geometry_msgs::msg::Point obstacle_pose_pre;
    static geometry_msgs::msg::Vector3 obstacle_vel_pre;
    static rclcpp::Time obstacle_stamp_pre;
    static rclcpp::Time Obstacles_stamp_now;
    Obstacles_stamp_now = msg->header.stamp;
    double dt = Obstacles_stamp_now.seconds() - obstacle_stamp_pre.seconds();
    double dt_cam = Obstacles_stamp_now.seconds() - cam_stamp.seconds();
    double min_distance = 400;
                                                             
    for (const obstacle_detector::msg::CircleObstacle& circle : msg->circles) {

        if (!in_playArea_obs(circle.center)) continue;

        if (!within_lock(obstacle_pose_pre, circle.center, dt)) continue;

        if (!first){

            obstacle_stamp = msg->header.stamp;
            obstacle_pose = circle.center;
            first = true;
            continue;
        }
        
        double distance_;
        if (dt_cam > 1) { // if camera is unavailable, use the rival's previous pose
            distance_ = sqrt(pow((circle.center.x - rival_final_pose.x), 2) + pow((circle.center.y - rival_final_pose.y), 2));
        } else { // if camera is available, find the one closest to the rival
            distance_ = sqrt(pow((circle.center.x - cam_rival_pose.x), 2) + pow((circle.center.y - cam_rival_pose.y), 2));
            camera_ok = true;
        }

        obstacle_ok = true;

        if (distance_ <= min_distance) {

            min_distance = distance_;
            obstacle_stamp = msg->header.stamp;
            obstacle_pose = circle.center;
        }
    }

    if(obstacle_stamp.seconds() != obstacle_stamp_pre.seconds()){

        obstacle_vel.x = (obstacle_pose.x - obstacle_pose_pre.x) / (obstacle_stamp.seconds() - obstacle_stamp_pre.seconds());
        obstacle_vel.y = (obstacle_pose.y - obstacle_pose_pre.y) / (obstacle_stamp.seconds() - obstacle_stamp_pre.seconds());
        obstacle_pose_pre = obstacle_pose;
        obstacle_vel_pre = obstacle_vel;
        obstacle_stamp_pre = obstacle_stamp;
    }

    if (!obstacle_ok){

        if (dt == 0) return;
        if (locking_rad > 3) return;
        locking_rad += locking_rad + dt * lockrad_growing_rate;
    }
    else 
        locking_rad = p_locking_rad;
    
    return;
}

void Rival::side_obstacles_callback(const obstacle_detector::msg::Obstacles::SharedPtr msg) {

    double max_radius = 0.0;

    for (const obstacle_detector::msg::CircleObstacle& circle : msg->circles) {

        if (!in_playArea_obs(circle.center)) continue; // check if the obstacle is in the play area
        // RCLCPP_INFO(this->get_logger(),"in play area: %f, %f", circle.center.x, circle.center.y);
        if (is_me(circle.center)) continue; // ignore the one closest to our robot
        // RCLCPP_INFO(this->get_logger(),"is not me: %f, %f", circle.center.x, circle.center.y);
        if (in_crossed_area(circle.center)) continue; // ignore the ones in columns area, or other specified areas
        // RCLCPP_INFO(this->get_logger(),"not in crossed area: %f, %f", circle.center.x, circle.center.y);
        if (circle.radius > max_radius) { // find the obstacle with the largest radius
            max_radius = circle.radius;
            side_obstacle_pose = circle.center;
        }
    }
    if (max_radius < 0.15) return; // no obstacle found, maybe the rival is blocked by something
    // RCLCPP_INFO(this->get_logger(),"max radius: %f", max_radius);
    // RCLCPP_INFO(this->get_logger(),"side_obstacle: %f, %f", side_obstacle_pose.x, side_obstacle_pose.y);
    side_obstacle_ok = true;
}

void Rival::robot_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    my_pose.x = msg->pose.pose.position.x;
    my_pose.y = msg->pose.pose.position.y;
}

void Rival::fusion() {

    rival_ok = false;

    double distance_cam_obs = 10;
    double distance_side_obs = 10;
    double distance_cam_side = 10;

    // compare the three poses, if two of them are ok
    if(camera_ok && side_obstacle_ok){
        distance_cam_side = sqrt(pow((side_obstacle_pose.x - cam_rival_pose.x), 2) + pow((side_obstacle_pose.y - cam_rival_pose.y), 2));
    }
    if(camera_ok && obstacle_ok){
        distance_cam_obs = sqrt(pow((obstacle_pose.x - cam_rival_pose.x), 2) + pow((obstacle_pose.y - cam_rival_pose.y), 2));
    }
    if(side_obstacle_ok && obstacle_ok){
        distance_side_obs = sqrt(pow((side_obstacle_pose.x - obstacle_pose.x), 2) + pow((side_obstacle_pose.y - obstacle_pose.y), 2));
    }

    // param for fusing the three
    if(distance_cam_side < cam_side_threshold && distance_cam_obs < obs_cam_threshold && distance_side_obs < side_obs_threshold){ // if all of them agree
        double total_weight = cam_weight + obs_weight + side_weight; // normalize the weights
        rival_raw_pose.x = cam_rival_pose.x * (cam_weight / total_weight) + obstacle_pose.x * (obs_weight / total_weight) + side_obstacle_pose.x * (side_weight / total_weight);
        rival_raw_pose.y = cam_rival_pose.y * (cam_weight / total_weight) + obstacle_pose.y * (obs_weight / total_weight) + side_obstacle_pose.y * (side_weight / total_weight);
        rival_ok = true;
    }
    else if(distance_cam_obs < obs_cam_threshold){
        double total_weight = cam_weight + obs_weight;
        rival_raw_pose.x = cam_rival_pose.x * (cam_weight / total_weight) + obstacle_pose.x * (obs_weight / total_weight);
        rival_raw_pose.y = cam_rival_pose.y * (cam_weight / total_weight) + obstacle_pose.y * (obs_weight / total_weight);
        rival_ok = true;
    }
    else if(distance_side_obs < side_obs_threshold){
        double total_weight = obs_weight + side_weight;
        rival_raw_pose.x = obstacle_pose.x * (obs_weight / total_weight) + side_obstacle_pose.x * (side_weight / total_weight);
        rival_raw_pose.y = obstacle_pose.y * (obs_weight / total_weight) + side_obstacle_pose.y * (side_weight / total_weight);
        rival_ok = true;
    }
    else if(distance_cam_side < cam_side_threshold){
        double total_weight = cam_weight + side_weight;
        rival_raw_pose.x = cam_rival_pose.x * (cam_weight / total_weight) + side_obstacle_pose.x * (side_weight / total_weight);
        rival_raw_pose.y = cam_rival_pose.y * (cam_weight / total_weight) + side_obstacle_pose.y * (side_weight / total_weight);
        rival_ok = true;
    }
    else{ // if none of them agree, use single one: cam -> side -> obs (what should be the priority?)
        if(camera_ok){
            rival_raw_pose = cam_rival_pose;
            // RCLCPP_INFO(this->get_logger(),"cam ok");
            rival_ok = true;
        }
        else if(side_obstacle_ok){
            rival_raw_pose = side_obstacle_pose;
            // RCLCPP_INFO(this->get_logger(),"side_obstacle ok");
            rival_ok = true;
        }
        else if(obstacle_ok){
            rival_raw_pose = obstacle_pose;
            // RCLCPP_INFO(this->get_logger(),"obstacle ok");
            rival_ok = true;
        }
    }
    // reset the flags
    obstacle_ok = false;
    camera_ok = false;
    side_obstacle_ok = false;
}


void Rival::timerCallback() {

    static geometry_msgs::msg::Point rival_pose_pre;
    static geometry_msgs::msg::Vector3 rival_vel_pre;

    fusion();

    if(rival_ok){
        // RCLCPP_INFO(this->get_logger(),"rival ok");
        rival_stamp = clock.now();

        rival_raw_vel = lpf(vel_lpf_gain, rival_vel_pre, rival_raw_vel);

        // 比較是否有 imm filter 的差異
        publish_rival_raw();
        publish_rival_final();

        broadcast_rival_tf();
        rival_pose_pre = rival_final_pose;
        rival_vel_pre = rival_final_vel;
        rival_ok = false;
    }
}

void Rival::publish_rival_raw() {

    rival_output.header.stamp = rival_stamp;
    rival_output.header.frame_id = "/map";
    rival_output.child_frame_id = rival_name + "/raw_pose";
    rival_output.pose.pose.position = rival_raw_pose;
    rival_output.pose.pose.orientation.w = 1;
    rival_output.twist.twist.linear = rival_raw_vel;

    rival_raw_pub->publish(rival_output);

    // RCLCPP_INFO(this->get_logger(),"raw Publish:");
    // RCLCPP_INFO(this->get_logger(),"center:( %f , %f )",rival_raw_pose.x, rival_raw_pose.y);
    // RCLCPP_INFO(this->get_logger(),"velocity:( %f , %f )", rival_raw_vel.x, rival_raw_vel.y);
    // RCLCPP_INFO(this->get_logger(),"time stamp: %f", rival_stamp.seconds());
    // RCLCPP_INFO(this->get_logger(),"-------------");
}

void Rival::publish_rival_final() {                    

    imm_filter();

    rival_output.header.stamp = rival_stamp;
    rival_output.header.frame_id = "/map";
    rival_output.child_frame_id = rival_name + "/final_pose";
    rival_output.pose.pose.position = rival_final_pose;
    rival_output.pose.pose.orientation.w = 1;
    rival_output.twist.twist.linear = rival_final_vel;

    rival_final_pub->publish(rival_output);

    // RCLCPP_INFO(this->get_logger(),"final Publish:");
    // RCLCPP_INFO(this->get_logger(),"center:( %f , %f )",rival_final_pose.x, rival_final_pose.y);
    // RCLCPP_INFO(this->get_logger(),"velocity:( %f , %f )", rival_final_vel.x, rival_final_vel.y);
    // RCLCPP_INFO(this->get_logger(),"time stamp: %f", rival_stamp.seconds());
    // RCLCPP_INFO(this->get_logger(),"-------------");

}

void Rival::broadcast_rival_tf() {

    static rclcpp::Time stamp_prev;

    if (stamp_prev.nanoseconds() != rival_stamp.nanoseconds()) {

        geometry_msgs::msg::TransformStamped transformStamped;

        transformStamped.header.stamp = rival_stamp;
        transformStamped.header.frame_id = "/map";
        transformStamped.child_frame_id = rival_name + "/base_footprint";
        transformStamped.transform.translation.x = rival_final_pose.x;
        transformStamped.transform.translation.y = rival_final_pose.y;
        transformStamped.transform.translation.z = rival_final_pose.z;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0); // Roll, Pitch, Yaw
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        br->sendTransform(transformStamped);
    }
    stamp_prev = rival_stamp;
}

int main(int argc, char * argv[]){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Rival>());
    rclcpp::shutdown();

    return 0;
}