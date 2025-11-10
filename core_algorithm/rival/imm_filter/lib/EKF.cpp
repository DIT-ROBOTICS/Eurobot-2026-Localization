#include <cstdlib>
#include <iostream>
#include <algorithm>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "imm_filter/EKF.h"

void KFBase::predict(const double& stamp) {
    setCurrentTimeStamp(stamp);
    updatePrediction();
    this->P_ = this->F_ * this->P_ * this->F_.transpose() + this->Q_;
}

void KFBase::update(const Eigen::VectorXd &z) {
    updateMeasurement();
    Eigen::MatrixXd S = this->H_ * this->P_ * this->H_.transpose() + this->R_;
    Eigen::VectorXd v = z - this->z_;

    for (size_t i = 0; i < angle_mask_.size(); i++) {
        if (angle_mask_[i] == true)
            v(i) = normalizeAngle(v(i)); // 使角度在-pi~+pi之間
    }
    double det = S.determinant();
    this->S_ = S;
    S = S.inverse();
    this->likelihood_ = 1.0 / sqrt(2 * M_PI * fabs(det)) * exp(-0.5 * v.transpose() * S * v); // 假設模型殘差適用高斯分布,計算似然值
    
    S = 0.5 * (S + S.transpose());
    Eigen::MatrixXd K = this->P_ * (this->H_.transpose() * S);


    this->x_ = this->x_ + K * v;
    Eigen::MatrixXd I;
    I.setIdentity(6, 6);
    Eigen::MatrixXd C = (I - K * this->H_);
    this->P_ = C * this->P_ * C.transpose() + K * R_ * K.transpose();
    this->P_ = this->P_ + 0.0001 * I;
}

void KFBase::updateOnce(const double& stamp, const Eigen::VectorXd* z) {

    if (z == nullptr) {
        predict(stamp);
    } 
    else {
        predict(stamp);
        update(*z);
    }
}

CV::CV() {}

CV::~CV() {}

void CV::init(const double &stamp, const Eigen::VectorXd &x) {
    if (x.size() != 6) {
         RCLCPP_WARN(rclcpp::get_logger("EKF"),"Dismatch between State and CV model.");
         exit(1);
    }
    else
        RCLCPP_INFO(rclcpp::get_logger("EKF"),"CV modle built");

    this->current_time_stamp_ = stamp;
    this->P_.setIdentity(6, 6);
    this->R_.resize(4, 4);
    this->R_ << 0.25, 0, 0, 0,
                0, 0.25, 0, 0,
                0, 0, 5, 0,
                0, 0, 0, 5;
    this->x_ = x; // x, y, theta, v
    this->F_.resize(6, 6);
    this->H_.resize(4, 6);
    this->H_ << 1, 0,  0, 0, 0, 0,
                0, 1,  0, 0, 0, 0,
                0, 0,  1, 0, 0, 0,
                0, 0,  0, 1, 0, 0;
    this->angle_mask_ = {false, false, false, false};
    this->z_.resize(4);
}

void CV::updatePrediction() {
    Eigen::VectorXd xt;
    xt.resize(6);
    double vx = x_(2);
    double vy = x_(3);
    this->F_ <<     1,   0,   dt_, 0,   0,   0,
                    0,   1,   0,   dt_, 0,   0,
                    0,   0,   1,   0,   0,   0,
                    0,   0,   0,   1,   0,   0,
                    0,   0,   0,   0,   1,   0,
                    0,   0,   0,   0,   0,   1;
    this->x_ = this->F_ * this->x_;
    /*--------------------------------------------------------------------*\
    ** CALC Process Noice Q Matrix
    \*--------------------------------------------------------------------*/
    {
        double delta_1 = dt_;
        double delta_2 = dt_ * dt_;
        double delta_3 = dt_ * dt_ * dt_; 
        Eigen::Matrix<double, 6, 2> G;
        G <<
                    1 / 2.0 * delta_2,                  0,
                                    0,  1 / 2.0 * delta_2, 
                                  dt_,                  0,
                                    0,                dt_,
                                    0,                  0,
                                    0,                  0;                    
        Eigen::Matrix2d E;
        E << 400, 0, 0, 400;
        this->Q_ = G * E * G.transpose();
    }
}

void CV::updateMeasurement() {
/*  觀測量為 x, y, theta, v
    double vx = x_(2);
    double vy = x_(3);

    
    this->z_(0) = x_(0);
    this->z_(1) = x_(1);
    this->z_(2) = atan2(x_(3), x_(2));
    this->z_(3) = sqrt(x_(2) * x_(2) + x_(3) * x_(3));

    this->H_ << 1, 0,                       0,                      0, 0, 0,
                0, 1,                       0,                      0, 0, 0,
                0, 0,     -vy/(vx*vx + vy*vy),     vx/(vx*vx + vy*vy), 0, 0,
                0, 0,  vx/sqrt(vx*vx + vy*vy), vy/sqrt(vx*vx + vy*vy), 0, 0; */

    this->z_(0) = x_(0);
    this->z_(1) = x_(1);
    this->z_(2) = x_(2);
    this->z_(3) = x_(3);
}

CA::CA() {}

CA::~CA() {}

void CA::init(const double &stamp, const Eigen::VectorXd &x) {
    if (x.size() != 6) {
        RCLCPP_WARN(rclcpp::get_logger("EKF"),"Dismatch between State and CA model.");
        exit(1);
    }
    else
        RCLCPP_INFO(rclcpp::get_logger("EKF"),"CA modle built");

    this->current_time_stamp_ = stamp;
    this->P_.setIdentity(6, 6);
    this->R_.resize(4, 4);
    this->R_ << 0.25, 0, 0, 0,
                0, 0.25, 0, 0,
                0, 0, 5, 0,
                0, 0, 0, 5;
    this->x_ = x; // x, y, theta, v
    this->F_.resize(6, 6);
    this->H_.resize(4, 6);      
    this->H_ << 1, 0,  0, 0, 0, 0,
                0, 1,  0, 0, 0, 0,
                0, 0,  1, 0, 0, 0,
                0, 0,  0, 1, 0, 0;
    this->angle_mask_ = {false, false, false, false};
    this->z_.resize(4);
}

void CA::updatePrediction() {
    Eigen::VectorXd xt;
    xt.resize(6);
    double vx = x_(2);
    double vy = x_(3);
    double ax = x_(4);
    double ay = x_(5);
    this->F_ <<     1,   0,   dt_,  0,  1 / 2.0 * dt_ * dt_,                     0,
                    0,   1,   0,  dt_,                    0,   1 / 2.0 * dt_ * dt_,
                    0,   0,   1,    0,                  dt_,                     0,
                    0,   0,   0,    1,                    0,                   dt_,
                    0,   0,   0,    0,                    1,                     0,
                    0,   0,   0,    0,                    0,                     1;
    this->x_ = this->F_ * this->x_;
    /*--------------------------------------------------------------------*\
    ** CALC Process Noice Q Matrix
    \*--------------------------------------------------------------------*/
    {
        double delta_1 = dt_;
        double delta_2 = dt_ * dt_;
        double delta_3 = dt_ * dt_ * dt_; 
        Eigen::Matrix<double, 6, 2> G;
        G <<
                    1 / 6.0 * delta_3,                  0,
                                    0,  1 / 6.0 * delta_3, 
                    1 / 2.0 * delta_2,                  0,
                                    0,  1 / 2.0 * delta_2,
                                  dt_,                  0,
                                    0,                dt_;                    
        Eigen::Matrix2d E;
        E << 400, 0, 0, 400;
        this->Q_ = G * E * G.transpose();
    }
    
}

void CA::updateMeasurement() {
/*  觀測量為 x, y, theta, v
    double vx = x_(2);
    double vy = x_(3);

    
    this->z_(0) = x_(0);
    this->z_(1) = x_(1);
    this->z_(2) = atan2(x_(3), x_(2));
    this->z_(3) = sqrt(x_(2) * x_(2) + x_(3) * x_(3));

    this->H_ << 1, 0,                       0,                      0, 0, 0,
                0, 1,                       0,                      0, 0, 0,
                0, 0,     -vy/(vx*vx + vy*vy),     vx/(vx*vx + vy*vy), 0, 0,
                0, 0,  vx/sqrt(vx*vx + vy*vy), vy/sqrt(vx*vx + vy*vy), 0, 0; */
    // 觀測量為 x, y, vx, vy            
    this->z_(0) = x_(0);
    this->z_(1) = x_(1);
    this->z_(2) = x_(2);
    this->z_(3) = x_(3);
}


CT::CT(const double& yaw_rate):yaw_rate_(yaw_rate) {
    if (fabs(yaw_rate_) < 1e-3)
        RCLCPP_WARN(rclcpp::get_logger("EKF"),"Yaw rate can't be zero in Const Turn rate model.");
}

CT::~CT() {}

void CT::init(const double &stamp, const Eigen::VectorXd &x) {

    if (x.size() != 6) {
        RCLCPP_WARN(rclcpp::get_logger("EKF"),"Dismatch between State and CT model.");
        exit(1);
    }
    else
        RCLCPP_INFO(rclcpp::get_logger("EKF"),"CT modle built");

    this->current_time_stamp_ = stamp;
    this->P_.setIdentity(6, 6);
    this->R_.resize(4, 4);
    this->R_ << 0.25, 0, 0, 0,
                0, 0.25, 0, 0,
                0, 0, 5, 0,
                0, 0, 0, 5;
    this->x_ = x; // x, y, theta, v
    this->F_.resize(6, 6);
    this->H_.resize(4, 6);  
    this->H_ << 1, 0,  0, 0, 0, 0,
                0, 1,  0, 0, 0, 0,
                0, 0,  1, 0, 0, 0,
                0, 0,  0, 1, 0, 0;    
    this->angle_mask_ = {false, false, false, false};
    this->z_.resize(4);
}


void CT::updatePrediction() {
    Eigen::VectorXd xt;
    xt.resize(6);
    
    double x = x_(0);
    double y = x_(1);
    double vx = x_(2);
    double vy = x_(3);
    double ax = x_(4);
    double ay = x_(5);
    double dA = yaw_rate_ * dt_;
    double sdA = sin(dA);
    double cdA = cos(dA);
    double sdA_w = sdA / yaw_rate_;
    double cdA_w = cdA / yaw_rate_;
    

    this->F_ <<     1,   0,                   sdA_w,  (cdA - 1) / yaw_rate_,    0,  0,
                    0,   1,   (1 - cdA) / yaw_rate_,                  sdA_w,    0,  0,
                    0,   0,                     cdA,                   -sdA,    0,  0,
                    0,   0,                     sdA,                    cdA,    0,  0,
                    0,   0,                       0,                      0,    0,  0,
                    0,   0,                       0,                      0,    0,  0;
    this->x_ = this->F_ * this->x_;
    /*--------------------------------------------------------------------*\
    ** CALC Process Noice Q Matrix
    \*--------------------------------------------------------------------*/
    {
        double delta_1 = dt_;
        double delta_2 = dt_ * dt_;
        double delta_3 = dt_ * dt_ * dt_; 
        double omega_1 = pow(yaw_rate_,-1);
        double omega_2 = pow(yaw_rate_,-2);
        double omega_3 = pow(yaw_rate_,-3);

        // TODO 增加角速度噪音項
        // double dx_dw = 2*vx*cdA*omega_3 + dt_*vy*omega_2 + dt_*vy*cdA*omega_2 - 2*vy*omega_3*sdA + dt_*vx*omega_2*sdA;
        // double dy_dw = 2*vy*cdA*omega_3 - dt_*vx*omega_2 - dt_*vx*cdA*omega_2 + 2*vx*omega_3*sdA + dt_*vy*omega_2*sdA;
        // double dvx_dw = -(vy*cdA*omega_2) + dt_*vx*cdA*omega_1 - vx*omega_2*sdA - dt_*vy*omega_1*sdA;
        // double dvy_dw = vx*cdA*omega_2 + dt_*vy*cdA*omega_1 - vy*omega_2*sdA + dt_*vx*omega_1*sdA;
        // Eigen::Matrix<double, 6, 3> G;
        // G <<
        //             1 / 2.0 * delta_2,                  0,  dx_dw,
        //                             0,  1 / 2.0 * delta_2,  dy_dw, 
        //                           dt_,                  0, dvx_dw,
        //                             0,                dt_, dvy_dw,
        //                             0,                  0,      0,
        //                             0,                  0,      0;  
                          
        // Eigen::Matrix3d E;
        // E << 40, 0, 0, 
        //      0, 40, 0,
        //      0,  0, 1;

        Eigen::Matrix<double, 6, 2> G;
        G <<
            1 / 2.0 * delta_2,                  0,
                            0,  1 / 2.0 * delta_2,
                          dt_,                  0,
                            0,                dt_,
                            0,                  0,
                            0,                  0;
        Eigen::Matrix2d E;
        E << 400, 0, 
             0, 400;
        this->Q_ = G * E * G.transpose();
    }
}

void CT::updateMeasurement() {
/*  觀測量為 x, y, theta, v
    double vx = x_(2);
    double vy = x_(3);

    
    this->z_(0) = x_(0);
    this->z_(1) = x_(1);
    this->z_(2) = atan2(x_(3), x_(2));
    this->z_(3) = sqrt(x_(2) * x_(2) + x_(3) * x_(3));

    this->H_ << 1, 0,                       0,                      0, 0, 0,
                0, 1,                       0,                      0, 0, 0,
                0, 0,     -vy/(vx*vx + vy*vy),     vx/(vx*vx + vy*vy), 0, 0,
                0, 0,  vx/sqrt(vx*vx + vy*vy), vy/sqrt(vx*vx + vy*vy), 0, 0; */

    // 觀測量為 x, y, vx, vy                    
    this->z_(0) = x_(0);
    this->z_(1) = x_(1);
    this->z_(2) = x_(2);
    this->z_(3) = x_(3);
}