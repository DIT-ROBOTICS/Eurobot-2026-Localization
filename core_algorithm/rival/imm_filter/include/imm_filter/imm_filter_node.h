#ifndef RIVAL_LOCALIZATION_H_
#define RIVAL_LOCALIZATION_H_
#include "imm_filter/IMM.h"
#include "imm_filter/EKF.h"
#include "imm_filter/ModelGenerator.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::placeholders;

class filter  : public rclcpp::Node{
    
    public:
        filter();

        void obstacles_callback(const nav_msgs::msg::Odometry::ConstPtr& msg);
        void IMM_publisher(double point_x, double point_y, double velocity_x, double velocity_y, rclcpp::Time time);
        void broadcast_rival_tf();

        bool first_;

        double sub_px_;
        double sub_py_;
        double sub_vx_;
        double sub_vy_;

    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr imm_sub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr imm_pub;
        rclcpp::Time rival_stamp;

        std::string robot_name;
        std::string rival_name;

        geometry_msgs::msg::TransformStamped rival_tf;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> imm_br;

        IMM model_;
};
#endif