#ifndef EKF_HPP_
#define EKF_HPP_

#include <eigen3/Eigen/Dense>
#include <math.h>

typedef struct RobotState{
    Eigen::Vector3d mu;
    Eigen::Matrix3d sigma;
}RobotState;

class EKF{
    public:
    EKF();
    
    void init(Eigen::Vector3d, Eigen::Matrix3d);

    void prediction(Eigen::Vector3d, Eigen::Matrix3d);
    void correction(Eigen::Vector3d, Eigen::Matrix3d);

    Eigen::Vector3d getPose();
    Eigen::Matrix3d getCov();

    private:
    RobotState robot_state;
};

#endif