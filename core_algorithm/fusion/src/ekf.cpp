#include "fusion/ekf.hpp"

EKF::EKF(){}

void EKF::init(Eigen::Vector3d init_mu, Eigen::Matrix3d init_sigma){
    this->robot_state.mu = init_mu;
    this->robot_state.sigma = init_sigma;
}

void EKF::prediction(Eigen::Vector3d x, Eigen::Matrix3d R){
    // EKF predict step
}

void EKF::correction(Eigen::Vector3d z, Eigen::Matrix3d Q){
    // EKF update step
}

Eigen::Vector3d EKF::getPose(){
    return robot_state.mu;
}

Eigen::Matrix3d EKF::getCov(){
    return robot_state.sigma;
}