#include "fusion/fusion_node.hpp"

#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


double qua2yaw(double x, double y, double z, double w){
    tf2::Quaternion qua(x, y, z, w);
    double _, yaw;
    tf2::Matrix3x3(qua).getRPY(_, _, yaw);
    return yaw;
}

geometry_msgs::msg::Quaternion yaw2qua(double yaw){
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
}

FusionNode::FusionNode() : Node("fusion_node"){
    this->declare_parameter("global_frame_id", "map");
    this->global_frame = this->get_parameter("global_frame_id").as_string();

    this->declare_parameter("robot_parent_frame_id", "odom");
    this->robot_parent_frame = this->get_parameter("robot_parent_frame_id").as_string();

    this->declare_parameter("robot_frame_id", "base_footprint");
    this->robot_frame = this->get_parameter("robot_frame_id").as_string();

    this->declare_parameter("compensation", "false");
    this->if_comp = this->get_parameter("compensation").as_bool();

    for(int i=0; i<9; i++){
        std::string str;
        switch(i){
            case 0: str = "x";
            case 4: str = "y";
            case 8: str = "yaw";
                this->declare_parameter("cb_cov" + str, 4e-4);
                this->cbcam_cov(i) = this->get_parameter("cb_cov"+str).as_double();
            break;
            default:
                this->cbcam_cov(i) = 0;
            break;
        }
    }

    for(int i=0; i<9; i++){
        std::string str;
        switch(i){
            case 0: str = "x";
            case 4: str = "y";
            case 8: str = "yaw";
                this->declare_parameter("ob_cov" + str, 4e-4);
                this->obcam_cov(i) = this->get_parameter("ob_cov"+str).as_double();
            break;
            default:
                this->obcam_cov(i) = 0;
            break;
        }
    }


    this->init_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovariance>(
        "initial_pose", 10, std::bind(&FusionNode::initCallback, this, std::placeholders::_1));

    this->local_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "local_pose", 10, std::bind(&FusionNode::localCallback, this, std::placeholders::_1));

    this->global_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "lidar_pose", 10, std::bind(&FusionNode::globalCallback, this, std::placeholders::_1));

    this->cbcam_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "cb_camera_pose", 10, std::bind(&FusionNode::cbCameraCallback, this, std::placeholders::_1));

    this->obcam_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "ob_camera_pose", 10, std::bind(&FusionNode::obCameraCallback, this, std::placeholders::_1));


    this->final_pose_pub = this->create_publisher<nav_msgs::msg::Odometry>("final_pose", 10);
}


void FusionNode::initCallback(const geometry_msgs::msg::PoseWithCovariance & init_msg){
    rclcpp::Clock clock;
    rclcpp::Time stamp = clock.now();


    Eigen::Vector3d init_pose;

    double init_yaw = qua2yaw(init_msg.pose.orientation.x,
                              init_msg.pose.orientation.y,
                              init_msg.pose.orientation.z,
                              init_msg.pose.orientation.w);

    init_pose << init_msg.pose.position.x, init_msg.pose.position.y, init_yaw;

    Eigen::Matrix3d init_cov;
    init_cov << init_msg.covariance[0], 0, 0,
                0, init_msg.covariance[7], 0,
                0, 0, init_msg.covariance[35];
    
    ekf.init(init_pose, init_cov);


    geometry_msgs::msg::Transform init_tf_msg;

    init_tf_msg.translation.x = 0;
    init_tf_msg.translation.y = 0;
    init_tf_msg.translation.z = 0;
    init_tf_msg.rotation.x = 0;
    init_tf_msg.rotation.y = 0;
    init_tf_msg.rotation.z = 0;
    init_tf_msg.rotation.w = 1;

    updateTransform(stamp, this->global_frame, this->robot_parent_frame, init_tf_msg);
}

void FusionNode::localCallback(const nav_msgs::msg::Odometry & local_msg){
    rclcpp::Time stamp = local_msg.header.stamp;


    Eigen::Vector3d local_pose;

    double local_yaw = qua2yaw(local_msg.pose.pose.orientation.x,
                               local_msg.pose.pose.orientation.y,
                               local_msg.pose.pose.orientation.z,
                               local_msg.pose.pose.orientation.w);

    local_pose << local_msg.pose.pose.position.x, local_msg.pose.pose.position.y, local_yaw;

    Eigen::Matrix3d local_cov;
    local_cov << local_msg.pose.covariance[0], 0, 0,
                 0, local_msg.pose.covariance[7], 0,
                 0, 0, local_msg.pose.covariance[35];

    ekf.prediction(local_pose, local_cov);
    
    
    Eigen::Vector3d robot_pose = ekf.getPose();
    Eigen::Matrix3d robot_cov = ekf.getCov();

    pubFinalPose(stamp, robot_pose, robot_cov);


    this->prev_pred_stamp = stamp;
}

void FusionNode::globalCallback(const geometry_msgs::msg::PoseWithCovarianceStamped & global_msg){
    rclcpp::Time stamp = global_msg.header.stamp;
    

    Eigen::Vector3d global_pose;

    double global_yaw = qua2yaw(global_msg.pose.pose.orientation.x,
                                global_msg.pose.pose.orientation.y,
                                global_msg.pose.pose.orientation.z,
                                global_msg.pose.pose.orientation.w);

    global_pose << global_msg.pose.pose.position.x, global_msg.pose.pose.position.y, global_yaw;
    
    Eigen::Matrix3d global_cov;
    global_cov << global_msg.pose.covariance[0], 0, 0,
                  0, global_msg.pose.covariance[7], 0,
                  0, 0, global_msg.pose.covariance[35];
    
    ekf.correction(global_pose, global_cov);
    

    Eigen::Vector3d robot_pose = ekf.getPose();
    Eigen::Matrix3d robot_cov = ekf.getCov();
    geometry_msgs::msg::Transform robot_tf_msg;

    pubFinalPose(stamp, robot_pose, robot_cov);

    robot_tf_msg.translation.x = robot_pose(0);
    robot_tf_msg.translation.y = robot_pose(1);
    robot_tf_msg.translation.z = 0;
    robot_tf_msg.rotation = yaw2qua(robot_pose(2));

    updateTransform(stamp, this->global_frame, this->robot_parent_frame, robot_tf_msg);
}

// @brief central beacon camera pose
void FusionNode::cbCameraCallback(const geometry_msgs::msg::PoseStamped & cbcam_msg){
    rclcpp::Time stamp = cbcam_msg.header.stamp;
    

    Eigen::Vector3d cbcam_pose;

    double cbcam_yaw = qua2yaw(cbcam_msg.pose.orientation.x,
                               cbcam_msg.pose.orientation.y,
                               cbcam_msg.pose.orientation.z,
                               cbcam_msg.pose.orientation.w);

    cbcam_pose << cbcam_msg.pose.position.x, cbcam_msg.pose.position.y, cbcam_yaw;
    
    ekf.correction(cbcam_pose, this->cbcam_cov);


    Eigen::Vector3d robot_pose = ekf.getPose();
    Eigen::Matrix3d robot_cov = ekf.getCov();
    geometry_msgs::msg::Transform robot_tf_msg;

    pubFinalPose(stamp, robot_pose, robot_cov);

    robot_tf_msg.translation.x = robot_pose(0);
    robot_tf_msg.translation.y = robot_pose(1);
    robot_tf_msg.translation.z = 0;
    robot_tf_msg.rotation = yaw2qua(robot_pose(2));

    updateTransform(stamp, this->global_frame, this->robot_parent_frame, robot_tf_msg);
}

// @brief onboard camera pose
void FusionNode::obCameraCallback(const geometry_msgs::msg::PoseStamped & obcam_msg){
    rclcpp::Time stamp = obcam_msg.header.stamp;
    

    Eigen::Vector3d obcam_pose;

    double obcam_yaw = qua2yaw(obcam_msg.pose.orientation.x,
                               obcam_msg.pose.orientation.y,
                               obcam_msg.pose.orientation.z,
                               obcam_msg.pose.orientation.w);

    obcam_pose << obcam_msg.pose.position.x, obcam_msg.pose.position.y, obcam_yaw;
    
    ekf.correction(obcam_pose, this->obcam_cov);


    Eigen::Vector3d robot_pose = ekf.getPose();
    Eigen::Matrix3d robot_cov = ekf.getCov();
    geometry_msgs::msg::Transform robot_tf_msg;

    pubFinalPose(stamp, robot_pose, robot_cov);

    robot_tf_msg.translation.x = robot_pose(0);
    robot_tf_msg.translation.y = robot_pose(1);
    robot_tf_msg.translation.z = 0;
    robot_tf_msg.rotation = yaw2qua(robot_pose(2));

    updateTransform(stamp, this->global_frame, this->robot_parent_frame, robot_tf_msg);
}

void FusionNode::pubFinalPose(rclcpp::Time stamp, Eigen::Vector3d pose, Eigen::Matrix3d cov){
    nav_msgs::msg::Odometry final_pose_msg;

    final_pose_msg.header.stamp = stamp;
    final_pose_msg.child_frame_id = robot_frame;
    final_pose_msg.pose.pose.position.x = pose(0);
    final_pose_msg.pose.pose.position.y = pose(1);
    final_pose_msg.pose.pose.position.z = 0;
    final_pose_msg.pose.pose.orientation = yaw2qua(pose(2));
    final_pose_msg.pose.covariance[0] = cov(0);
    final_pose_msg.pose.covariance[7] = cov(4);
    final_pose_msg.pose.covariance[35] = cov(8);

    this->final_pose_pub->publish(final_pose_msg);
}

// @brief directly update transform between two frames
void FusionNode::updateTransform(rclcpp::Time stamp, std::string from_frame, std::string to_frame, geometry_msgs::msg::Transform tf_msg){
    geometry_msgs::msg::TransformStamped pub_tf_msg;

    pub_tf_msg.header.stamp = stamp;
    pub_tf_msg.header.frame_id = from_frame;
    pub_tf_msg.child_frame_id = to_frame;
    pub_tf_msg.transform = tf_msg;

    this->tf_broadcaster->sendTransform(pub_tf_msg);
}

/*
 * @brief indirectly update transform.
 * TF tree: A->B->C. Update TF of AB using AB = AC * BC^-1.
 * A: from_frame, B: to_frame, C: by_frame, TF of AC: tf_msg.
 */
void FusionNode::updateTransform(rclcpp::Time stamp, std::string from_frame, std::string to_frame, std::string by_frame, geometry_msgs::msg::Transform tf_msg){
    geometry_msgs::msg::TransformStamped from2to_tf_msg;

    from2to_tf_msg.header.stamp = stamp;
    from2to_tf_msg.header.frame_id = from_frame;
    from2to_tf_msg.child_frame_id = to_frame;


    tf2::Transform from2by_tf;
    tf2::fromMsg(tf_msg, from2by_tf);

    tf2::Transform to2by_tf;
    geometry_msgs::msg::TransformStamped to2by_tf_msg;

    try{
        to2by_tf_msg = this->tf_buffer->lookupTransform(by_frame, to_frame, stamp);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            by_frame.c_str(), to_frame.c_str(), ex.what());
        return;
    }
    tf2::fromMsg(to2by_tf_msg.transform, to2by_tf);


    tf2::Transform from2to_tf;
    from2to_tf = from2by_tf*to2by_tf.inverse();

    from2to_tf_msg.transform = tf2::toMsg(from2to_tf);


    this->tf_broadcaster->sendTransform(from2to_tf_msg);
}

int main(int argc, char * argv[]){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FusionNode>());
    rclcpp::shutdown();

    return 0;
}