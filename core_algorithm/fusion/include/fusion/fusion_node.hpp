#ifndef FUSION_NODE_HPP_
#define FUSION_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

#include <eigen3/Eigen/Dense>

#include "fusion/ekf.hpp"

class FusionNode : public rclcpp::Node{
    public:
    FusionNode();

    void initCallback(const geometry_msgs::msg::PoseWithCovariance &);
    void localCallback(const nav_msgs::msg::Odometry &);
    void globalCallback(const geometry_msgs::msg::PoseWithCovarianceStamped &);
    void cbCameraCallback(const geometry_msgs::msg::PoseStamped &);
    void obCameraCallback(const geometry_msgs::msg::PoseStamped &);
    
    void pubFinalPose(rclcpp::Time, Eigen::Vector3d, Eigen::Matrix3d);
    void updateTransform(rclcpp::Time, std::string, std::string, geometry_msgs::msg::Transform);
    void updateTransform(rclcpp::Time, std::string, std::string, std::string, geometry_msgs::msg::Transform);

    void compensation();

    private:
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovariance>::SharedPtr init_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr global_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cbcam_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr obcam_sub;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr final_pose_pub;
    
    // record last prediction stamp for compensation
    rclcpp::Time prev_pred_stamp;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

    std::string global_frame;
    std::string robot_parent_frame;
    std::string robot_frame;

    Eigen::Matrix3d cbcam_cov;
    Eigen::Matrix3d obcam_cov;

    bool if_comp;

    EKF ekf;
};

#endif // FUSION_NODE_HPP_