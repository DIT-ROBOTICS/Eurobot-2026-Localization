#include "imu_drive/imu_drive.h"

int main(int argc, char * argv[]){

    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("IMU_DRIVE"), "Initializing imu drive node");
    
    try {

        rclcpp::spin(std::make_shared<IMU>());
    }
    catch (const char* s) {
  
        RCLCPP_FATAL(rclcpp::get_logger("IMU DRIVE"), "%s", s);
  	
    }
    catch (...)	{
    	
        RCLCPP_FATAL(rclcpp::get_logger("IMU DRIVE"), "Unexpected error");
    }

    rclcpp::shutdown();

    RCLCPP_INFO(rclcpp::get_logger("IMU DRIVE"), "Shutdown");

    return 0;
}