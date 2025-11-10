#include "imm_filter/imm_filter_node.h"

filter::filter () : Node("imm_filter_node"){

    IMM imm;

    model_ = imm;
    first_ = true;

    robot_name = "robot";
    rival_name = "rival_final";
            
    imm_sub = this->create_subscription<nav_msgs::msg::Odometry>("raw_pose", 10, std::bind(&filter::obstacles_callback, this, _1));
    imm_pub = this->create_publisher<nav_msgs::msg::Odometry>("final_pose", 10);
    imm_br = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
}

void filter::obstacles_callback(const nav_msgs::msg::Odometry::ConstPtr& msg){

    rclcpp::Clock clock;
    rclcpp::Time stamp = clock.now();
    rival_stamp = stamp;

    Eigen::VectorXd x;
    Eigen::VectorXd z;
    
    sub_px_ = msg->pose.pose.position.x;
    sub_py_ = msg->pose.pose.position.y;
    sub_vx_ = msg->twist.twist.linear.x;
    sub_vy_ = msg->twist.twist.linear.y;

    // RCLCPP_INFO(this->get_logger(),"Subscribe:");
    // RCLCPP_INFO(this->get_logger(),"center:( %f , %f )", sub_px_, sub_py_);
    // RCLCPP_INFO(this->get_logger(),"velocity:( %f , %f )", sub_vx_, sub_vy_);
    // RCLCPP_INFO(this->get_logger(),"-------------");

    if (!first_) {

        z.resize(4);

        z << sub_px_, sub_py_, sub_vx_, sub_vy_;

        model_.updateOnce(stamp.seconds(), &z);

        IMM_publisher(model_.x_[0], model_.x_[1], model_.x_[2], model_.x_[3], stamp);
    }
    else {
        ModelGenerator gen;

        x.resize(6);

        //initial state
        x << sub_px_, sub_py_, sub_vx_, sub_vy_, 0, 0;

        gen.generateIMMModel(stamp.seconds(), x, model_);

        first_ = false;
    }
}

void filter::broadcast_rival_tf() {

    static rclcpp::Time stamp_prev;

    if (stamp_prev.nanoseconds() != rival_stamp.nanoseconds()) {

        geometry_msgs::msg::TransformStamped transformStamped;

        transformStamped.header.stamp = rival_stamp;
        transformStamped.header.frame_id = "/map";
        transformStamped.child_frame_id = rival_name + "/base_footprint";

        transformStamped.transform.translation.x = sub_px_;
        transformStamped.transform.translation.y = sub_py_;
        transformStamped.transform.translation.z = 0;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0); // Roll, Pitch, Yaw
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        imm_br->sendTransform(transformStamped);
    }
    stamp_prev = rival_stamp;
}

void filter::IMM_publisher(double point_x, double point_y, double velocity_x, double velocity_y, rclcpp::Time time){

    nav_msgs::msg::Odometry odom;

    odom.pose.pose.position.x = point_x;
    odom.pose.pose.position.y = point_y;
    odom.twist.twist.linear.x = velocity_x;
    odom.twist.twist.linear.y = velocity_y;
    odom.header.stamp = time;

    broadcast_rival_tf();
    imm_pub->publish(odom);

    // RCLCPP_INFO(this->get_logger(),"Publish:");
    // RCLCPP_INFO(this->get_logger(),"center:( %f , %f )", point_x, point_y);
    // RCLCPP_INFO(this->get_logger(),"velocity:( %f , %f )", velocity_x, velocity_y);
    // RCLCPP_INFO(this->get_logger(),"time stamp: %f", time.seconds());
    // RCLCPP_INFO(this->get_logger(),"-------------");
}

int main(int argc, char * argv[]) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<filter>());
    rclcpp::shutdown();

    return 0;
}