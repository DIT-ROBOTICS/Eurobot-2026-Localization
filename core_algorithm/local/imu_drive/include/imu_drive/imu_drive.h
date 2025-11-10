#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rcl/time.h"

#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"

using std::placeholders::_1;

class IMU : public rclcpp::Node{

public :

    IMU() : Node("imu_drive_node", rclcpp::NodeOptions()){

        Initialize();
    }

private :

    /* Function - for initialize params */
    void Initialize();

    /* Function - for update params */
    void UpdateParams();

    /* Function - for sensor_msgs::Imu ( /imu/data )*/
    void IMUdataCallback(const sensor_msgs::msg::Imu::ConstPtr &msg);

    /* Function publish sth we need */
    void publish();

    /** -- Advertise -- **/
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    /** -- Msgs to pub -- **/
    sensor_msgs::msg::Imu imu_output_;
    sensor_msgs::msg::Imu imu_output_backup_;

    /** -- For low pass filter -- **/
    geometry_msgs::msg::Vector3 prev_angular_velocity;
    geometry_msgs::msg::Vector3 prev_linear_acceleration;

	/* Parameters */
    bool p_active_;
	bool p_publish_;
    bool p_update_params_; 

    double p_covariance_;
	double p_cov_multi_vel_;
	double p_cov_multi_acl_;
    double p_filter_prev_;

    std::string p_frame_;

    std::string p_imu_sub_topic_;
    std::string p_imu_pub_topic_;
};