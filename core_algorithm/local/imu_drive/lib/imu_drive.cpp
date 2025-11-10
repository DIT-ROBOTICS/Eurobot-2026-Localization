#include "imu_drive/imu_drive.h"

void IMU::Initialize(){

    RCLCPP_INFO(this->get_logger(), "inactive node");

    this->declare_parameter<bool>("active", false);
    this->declare_parameter<bool>("publish", false);
    this->declare_parameter<double>("covariance_vx", 0);
    this->declare_parameter<double>("covariance_vy", 0);
    this->declare_parameter<double>("covariance_vz", 0);
    this->declare_parameter<double>("covariance_ax", 0);
    this->declare_parameter<double>("covariance_ay", 0);
    this->declare_parameter<double>("covariance_az", 0);
    this->declare_parameter<double>("cov_multi_vel", 0);
    this->declare_parameter<double>("cov_multi_acl", 0);
    this->declare_parameter<double>("filter_prev", 0);

    UpdateParams();
}

void IMU::UpdateParams(){

    RCLCPP_INFO(this->get_logger(), "start to update parameters");

    /* get param */
    p_active_ = this->get_parameter("active").get_value<bool>();
    RCLCPP_INFO(this->get_logger(), "active set to %d", p_active_);

    p_publish_ = this->get_parameter("publish").get_value<bool>();
    RCLCPP_INFO(this->get_logger(), "publish set to %d", p_publish_);

    p_imu_sub_topic_ = "sub_topic";
    RCLCPP_INFO(this->get_logger(), "Current subscribe topic [ %s ]", p_imu_sub_topic_.c_str());

    p_imu_pub_topic_ = "pub_topic";
    RCLCPP_INFO(this->get_logger(), "Current publish topic [ %s ]", p_imu_pub_topic_.c_str());

    p_covariance_ = this->get_parameter("covariance_vx").get_value<double>();
    RCLCPP_INFO(this->get_logger(), "vx covariance set to %f", p_covariance_);
    this->imu_output_.angular_velocity_covariance[0] = p_covariance_;

    p_covariance_ = this->get_parameter("covariance_vy").get_value<double>();
    RCLCPP_INFO(this->get_logger(), "vy covariance set to %f", p_covariance_);
    this->imu_output_.angular_velocity_covariance[4] = p_covariance_;

    p_covariance_ = this->get_parameter("covariance_vz").get_value<double>();
    RCLCPP_INFO(this->get_logger(), "vz covariance set to %f", p_covariance_);
    this->imu_output_.angular_velocity_covariance[8] = p_covariance_;
    
    p_covariance_ = this->get_parameter("covariance_ax").get_value<double>();
    RCLCPP_INFO(this->get_logger(), "ax covariance set to %f", p_covariance_);
    this->imu_output_.linear_acceleration_covariance[0] = p_covariance_;
    
    p_covariance_ = this->get_parameter("covariance_ay").get_value<double>();
    RCLCPP_INFO(this->get_logger(), "ay covariance set to %f", p_covariance_);
    this->imu_output_.linear_acceleration_covariance[4] = p_covariance_;

    p_covariance_ = this->get_parameter("covariance_az").get_value<double>();
    RCLCPP_INFO(this->get_logger(), "az covariance set to %f", p_covariance_);
    this->imu_output_.linear_acceleration_covariance[8] = p_covariance_;

    p_cov_multi_vel_ = this->get_parameter("cov_multi_vel").get_value<double>();
    RCLCPP_INFO(this->get_logger(), "[Odometry] : gyroscope \"a\" is set to %f", p_cov_multi_vel_);

    p_cov_multi_acl_ = this->get_parameter("cov_multi_acl").get_value<double>();
    RCLCPP_INFO(this->get_logger(), "[Odometry] : accel \"a\" is set to %f", p_cov_multi_acl_);

    p_filter_prev_ = this->get_parameter("filter_prev").get_value<double>();
    RCLCPP_INFO(this->get_logger(), "[Odometry] : low pass filter constant is set to %f", p_filter_prev_);

    RCLCPP_INFO(this->get_logger(), "active node");

    this->imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(p_imu_sub_topic_, 10, std::bind(&IMU::IMUdataCallback, this, _1));
    this->imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(p_imu_pub_topic_, 10);
    
    /* -- Backup covariance -- */
	this->imu_output_backup_ = this->imu_output_;
		
    /* -- Set basic variables -- */
    this->imu_output_.header.frame_id = this->p_frame_;
}

void IMU::IMUdataCallback(const sensor_msgs::msg::Imu::ConstPtr &msg){  //  from /imu/data

    rclcpp::Clock clock;

    this->imu_output_.header.stamp = clock.now();

    this->imu_output_.angular_velocity = msg->angular_velocity;

    this->imu_output_.linear_acceleration = msg->linear_acceleration;

    this->imu_output_.angular_velocity=msg->angular_velocity; /* <!-- ADD --> */

    this->prev_angular_velocity = this->imu_output_.angular_velocity;

    if(this->p_publish_) 
        publish();
}

void IMU::publish(){

    this->imu_pub_->publish(this->imu_output_);
}