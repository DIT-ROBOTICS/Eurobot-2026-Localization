#include "rclcpp/rclcpp.hpp"
// msg
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
// matrix calulation
#include <eigen3/Eigen/Dense>
#include <math.h>
// tf2
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/msg/point_stamped.hpp"
#include <tf2/LinearMath/Transform.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
struct RobotState
{
    Eigen::Vector3d mu;
    Eigen::Matrix3d sigma;
};

class GlobalFilterNode {
public:
    GlobalFilterNode(std::shared_ptr<rclcpp::Node> nh) :
    nh_(nh){
        // Initialize filter coefficients and initial values
        rclcpp::Clock clock;
        rclcpp::Time now=clock.now();
        prev_stamp_=now;
        linear_x_ = 0.0;
        linear_y_ = 0.0;
        angular_z_ = 0.0;
        robotstate_.mu[0] = 0;
        robotstate_.mu[1] = 0;
        robotstate_.mu[2] = 0;
        robotstate_.sigma.setZero();
        robotstate_.sigma(0, 0) = 0.1;   // x-x
        robotstate_.sigma(1, 1) = 0.1;   // y-y
        robotstate_.sigma(2, 2) = 0.1;   // theta-theta

        nh_->declare_parameter("LPF_alpha_x", 0.5); // filter coefficient
        alpha_x=nh_->get_parameter("LPF_alpha_x").as_double();

        nh_->declare_parameter("LPF_alpha_y", 0.5); // filter coefficient
        alpha_y=nh_->get_parameter("LPF_alpha_y").as_double();

        nh_->declare_parameter("linear_cov_max", 0.1);
        linear_cov_max_=nh_->get_parameter("linear_cov_max").as_double();

        nh_->declare_parameter("angular_cov_max", 0.05);
        angular_cov_max_=nh_->get_parameter("angular_cov_max").as_double();

        for(int i=0;i<3;i++){
            std::string str;
            switch(i){
                case 0: str="vx"; break;
                case 1: str="vy"; break;
                case 2: str="vz"; break;
                default: break;
            }
            nh_->declare_parameter("covariance_"+str, 0.);
            cov_backup_[i]=nh_->get_parameter("covariance_"+str).as_double();
        }

        for(int i=0;i<3;i++){
            std::string str;
            switch(i){
                case 1: str="vx"; break;
                case 2: str="vy"; break;
                case 3: str="vz"; break;
                default: break;
            }
            nh_->declare_parameter("covariance_multi_"+str, 0.);
            cov_multi_[i]=nh_->get_parameter("covariance_multi_"+str).as_double();
        }

        setpose_sub_ = nh_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initial_pose", 50, std::bind(&GlobalFilterNode::setposeCallback, this, std::placeholders::_1));

        odom_sub_ = nh_->create_subscription<geometry_msgs::msg::Twist>(
            "odoo_googoogoo", 10, std::bind(&GlobalFilterNode::odomCallback, this, std::placeholders::_1));

        imu_sub_ = nh_->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data_cov", 10, std::bind(&GlobalFilterNode::imuCallback, this, std::placeholders::_1));

        final_pose_sub_ = nh_->create_subscription<nav_msgs::msg::Odometry>(
            "final_pose", 10, std::bind(&GlobalFilterNode::finalCallback, this, std::placeholders::_1));

        global_filter_pub_ = nh_->create_publisher<nav_msgs::msg::Odometry>("local_filter", 10);
        odom2map_pub_=nh_->create_publisher<geometry_msgs::msg::PoseStamped>("odom2map", 10);
    }

    void diff_model(double v, double w, double dt)
    {
        double theta = robotstate_.mu(2);
        double s = sin(theta); double s_ = sin(theta + w * dt);
        double c = cos(theta); double c_ = cos(theta + w * dt);
        
        if ((w < 0.00001) && (w > -0.00001)){
            robotstate_.mu = robotstate_.mu + Eigen::Vector3d{ v * c * dt, v * s * dt, 0.0 };
        }else{
            robotstate_.mu = robotstate_.mu + Eigen::Vector3d{ v * (-s + s_) / w, v * (c - c_) / w, w * dt };
        }
        robotstate_.mu(2) = angleLimitChecking(robotstate_.mu(2));
    }

    void omni_model(double v_x, double v_y, double w, double dt)
    {
        /* ekf prediction function for omni wheel */
        Eigen::Vector3d d_state;

        d_state << (v_x * dt), (v_y * dt), (w * dt);

        double theta_ = robotstate_.mu(2); /* <!-- ADD --> */
        double s__theta = sin(theta_);
        double c__theta = cos(theta_);
        // double s__delta = sin(d_state(2));
        // double c__delta = cos(d_state(2));
        Eigen::Matrix3d A;
        A << 1, 0, 0, 0, 1, 0, 0, 0, 1;

        Eigen::Matrix3d B;
        // if (abs(w)> 1e-3)
        //     B << (c__theta*s__delta - s__theta*(c__delta -1))/w, -(s__theta*s__delta - c__theta*(c__delta -1))/w, 0, (s__theta*s__delta - c__theta*(c__delta -1))/w, (c__theta*s__delta - s__theta*(c__delta -1))/w, 0, 0, 0, 1;
        // else
            B << c__theta, -s__theta, 0, s__theta, c__theta, 0, 0, 0, 1;

        Eigen::Matrix3d cov_past;
        cov_past = robotstate_.sigma;

        /* Update robot state mean */
        robotstate_.mu = A * robotstate_.mu + B * d_state;
    }

    void setposeCallback(const geometry_msgs::msg::PoseWithCovarianceStamped & pose_msg)
    {
        double x = pose_msg.pose.pose.position.x;
        double y = pose_msg.pose.pose.position.y;
        
        init_pose.position.x=x;
        init_pose.position.y=y;
        init_pose.orientation.x=pose_msg.pose.pose.orientation.x;
        init_pose.orientation.y=pose_msg.pose.pose.orientation.y;
        init_pose.orientation.z=pose_msg.pose.pose.orientation.z;
        init_pose.orientation.w=pose_msg.pose.pose.orientation.w;
        
        rclcpp::Clock clock;
        rclcpp::Time now=clock.now();

        tf2::Quaternion q;
        tf2::fromMsg(pose_msg.pose.pose.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        robotstate_.mu(0) = x;
        robotstate_.mu(1) = y;
        robotstate_.mu(2) = yaw;

        robotstate_.sigma(0, 0) = pose_msg.pose.covariance[0];   // x-x
        robotstate_.sigma(0, 1) = pose_msg.pose.covariance[1];   // x-y
        robotstate_.sigma(0, 2) = pose_msg.pose.covariance[5];   // x-theta
        robotstate_.sigma(1, 0) = pose_msg.pose.covariance[6];   // y-x
        robotstate_.sigma(1, 1) = pose_msg.pose.covariance[7];   // y-y
        robotstate_.sigma(1, 2) = pose_msg.pose.covariance[11];  // y-theta
        robotstate_.sigma(2, 0) = pose_msg.pose.covariance[30];  // theta-x
        robotstate_.sigma(2, 1) = pose_msg.pose.covariance[31];  // theta-y
        robotstate_.sigma(2, 2) = pose_msg.pose.covariance[35];  // theta-theta
        RCLCPP_INFO(nh_->get_logger(), "Received initial pose: x = %f, y = %f, theta = %f",  robotstate_.mu(0), robotstate_.mu(1), robotstate_.mu(2));
        
        // publish absolute coordinate
        coord_odom2map.header.stamp= now;
        coord_odom2map.header.frame_id= "map";
        coord_odom2map.pose.position.x=robotstate_.mu(0);
        coord_odom2map.pose.position.y=robotstate_.mu(1);

        tf2::Quaternion q_;
        q_.setRPY(0, 0, robotstate_.mu(2));
        coord_odom2map.pose.orientation.x=q_.getX();
        coord_odom2map.pose.orientation.y=q_.getY();
        coord_odom2map.pose.orientation.z=q_.getZ();
        coord_odom2map.pose.orientation.w=q_.getW();
        odom2map_pub_->publish(coord_odom2map);
        
    }

    void odomCallback(const geometry_msgs::msg::Twist & odom_msg) {
        // get velocity data
        // Apply low-pass filter to linear xy from odom
        linear_x_ = alpha_x * odom_msg.linear.x + (1 - alpha_x) * linear_x_;
        linear_y_ = alpha_y * odom_msg.linear.y + (1 - alpha_y) * linear_y_;
        // angular_z_=odom_msg.angular.z;
       
        double cov_multi[3];
        cov_multi[0]=cov_multi_[0]*abs(odom_msg.linear.x);
        cov_multi[1]=cov_multi_[1]*abs(odom_msg.linear.y);
        cov_multi[2]=cov_multi_[2]*abs(odom_msg.angular.z);
        linear_x_cov_=std::min(linear_cov_max_, std::max(1e-8, cov_multi[0]+cov_backup_[0]));
        linear_y_cov_=std::min(linear_cov_max_, std::max(1e-8, cov_multi[1]+cov_backup_[1]));
        cov_backup_[0]=linear_x_cov_;
        cov_backup_[1]=linear_y_cov_;
    }

    void imuCallback(const sensor_msgs::msg::Imu & imu_msg) {
        angular_z_ = imu_msg.angular_velocity.z;
        rclcpp::Time stamp=imu_msg.header.stamp;
        double dt=stamp.seconds()-prev_stamp_.seconds();
        omni_model(linear_x_, linear_y_, angular_z_, dt);
        local_filter_pub(imu_msg.header.stamp, std::min(angular_cov_max_, imu_msg.angular_velocity_covariance[8]));
        prev_stamp_ = imu_msg.header.stamp;
    }

    void finalCallback(const nav_msgs::msg::Odometry & final_pose_msg){
        rclcpp::Time stamp = final_pose_msg.header.stamp;

        geometry_msgs::msg::Pose map2base_msg;
        map2base_msg.position.x = final_pose_msg.pose.pose.position.x;
        map2base_msg.position.y = final_pose_msg.pose.pose.position.y;
        map2base_msg.position.z = final_pose_msg.pose.pose.position.z;
        map2base_msg.orientation.x = final_pose_msg.pose.pose.orientation.x;
        map2base_msg.orientation.y = final_pose_msg.pose.pose.orientation.y;
        map2base_msg.orientation.z = final_pose_msg.pose.pose.orientation.z;
        map2base_msg.orientation.w = final_pose_msg.pose.pose.orientation.w;

        updateMap2Odom(stamp, map2base_msg);
    }

    void local_filter_pub(rclcpp::Time stamp, double imu_cov)
    {
        // Publish global_filter message
        nav_msgs::msg::Odometry global_filter_msg;
        global_filter_msg.header.stamp = stamp; // imu callback stamp
        global_filter_msg.header.frame_id = "map";
        global_filter_msg.child_frame_id = "base_link";
        //velocity
            global_filter_msg.twist.twist.linear.x = linear_x_; //filtered x velocity
            global_filter_msg.twist.twist.linear.y = linear_y_;
            global_filter_msg.twist.twist.angular.z = angular_z_; //raw imu data
        //pose
            global_filter_msg.pose.pose.position.x = robotstate_.mu(0);
            global_filter_msg.pose.pose.position.y = robotstate_.mu(1);
            tf2::Quaternion quaternion_;
            quaternion_.setRPY( 0, 0, robotstate_.mu(2) );
            quaternion_ = quaternion_.normalize();
            global_filter_msg.pose.pose.orientation.z = quaternion_.getZ();
            global_filter_msg.pose.pose.orientation.w = quaternion_.getW();
        //covariance
            global_filter_msg.twist.covariance[0] = linear_x_cov_; //x-x
            global_filter_msg.twist.covariance[7] = linear_y_cov_; //y-y
            global_filter_msg.twist.covariance[35] = imu_cov; //theta-theta
            global_filter_pub_->publish(global_filter_msg);
        // publish absolute coordinate
            coord_odom2map.header.stamp= stamp;
            coord_odom2map.header.frame_id= "map";
            coord_odom2map.pose.position.x=robotstate_.mu(0);
            coord_odom2map.pose.position.y=robotstate_.mu(1);

            tf2::Quaternion q_;
            q_.setRPY(0, 0, robotstate_.mu(2));
            coord_odom2map.pose.orientation.x=q_.getX();
            coord_odom2map.pose.orientation.y=q_.getY();
            coord_odom2map.pose.orientation.z=q_.getZ();
            coord_odom2map.pose.orientation.w=q_.getW();
            odom2map_pub_->publish(coord_odom2map);
        // publish static fransform between odom and base
            updateOdom2Base(stamp);
    }

    void updateMap2Odom(rclcpp::Time stamp, geometry_msgs::msg::Pose pose_msg){
        geometry_msgs::msg::TransformStamped map2odom_msg;

        map2odom_msg.header.stamp = stamp;
        map2odom_msg.header.frame_id = "map";
        map2odom_msg.child_frame_id = "odom";

        tf2::Transform map2base, odom2base;

        map2base.setOrigin(tf2::Vector3(pose_msg.position.x, pose_msg.position.y, 0.0));
        map2base.setRotation(
            tf2::Quaternion(
                pose_msg.orientation.x, 
                pose_msg.orientation.y, 
                pose_msg.orientation.z, 
                pose_msg.orientation.w));

        odom2base.setOrigin(tf2::Vector3(robotstate_.mu(0), robotstate_.mu(1), 0.0));
        odom2base.setRotation(
            tf2::Quaternion(
                0, 
                0, 
                sin(robotstate_.mu(2)/2), 
                cos(robotstate_.mu(2)/2)));

        tf2::Transform map2odom = map2base*odom2base.inverse();

        map2odom_msg.transform = tf2::toMsg(map2odom);
        broadcaster_->sendTransform(map2odom_msg);
    }

    void updateOdom2Base(rclcpp::Time stamp){
        geometry_msgs::msg::TransformStamped odom2base_msg;

        odom2base_msg.header.frame_id = "map";
        odom2base_msg.child_frame_id = "odom";
        odom2base_msg.header.stamp = stamp;

        odom2base_msg.transform.translation.x = robotstate_.mu(0);
        odom2base_msg.transform.translation.y = robotstate_.mu(1);

        tf2::Quaternion q_;
        q_.setRPY(0, 0, robotstate_.mu(2));
        
        odom2base_msg.transform.rotation.x = q_.getX();
        odom2base_msg.transform.rotation.y = q_.getY();
        odom2base_msg.transform.rotation.z = q_.getZ();
        odom2base_msg.transform.rotation.w = q_.getW();
        broadcaster_->sendTransform(odom2base_msg);
    }

    double angleLimitChecking(double theta)
    {
        while (theta > M_PI)
        {
            theta -= M_PI * 2;
        }
        while (theta <= -M_PI)
        {
            theta += M_PI * 2;
        }
        return theta;
    }

private:
    std::shared_ptr<rclcpp::Node> nh_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr setpose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr final_pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr global_filter_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr odom2map_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    //raw
    double twist_x_;
    double twist_y_;
    double cov_backup_[3];
    double cov_multi_[3];
    geometry_msgs::msg::Pose init_pose;
    geometry_msgs::msg::PoseStamped coord_odom2map;
    //filtered
    double alpha_x; // filter coefficient
    double alpha_y; // filter coefficient
    double linear_x_; 
    double linear_x_cov_;
    double linear_y_;
    double linear_y_cov_;
    double angular_z_;
    // double imu_cov_;
    RobotState robotstate_;
    rclcpp::Time prev_stamp_;
    double linear_cov_max_;
    double angular_cov_max_;
};

int main(int argc, char** argv) {
    
    rclcpp::init(argc, argv);

    auto exec=std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared("nh");
    GlobalFilterNode global_filter_node(nh);

    exec->add_node(nh);
    exec->spin();

    rclcpp::shutdown();
    
    return 0;
}
