#ifndef EKF_H_
#define EKF_H_
#include <cstdlib>
#include <iostream>
#include <algorithm>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>
#include "rclcpp/rclcpp.hpp"

class KFBase {

    public:
        void predict(const double& stamp);
        void update(const Eigen::VectorXd &z);
        void updateOnce(const double& stamp, const Eigen::VectorXd* z = nullptr);

        virtual void init(const double &stamp, const Eigen::VectorXd &x) {}

        Eigen::VectorXd x() const {return this->x_;}
        Eigen::MatrixXd P() const {return this->P_;}
        Eigen::MatrixXd S() const {return this->S_;}

        void setStateCoveriance(const Eigen::MatrixXd& P) {
            this->P_ = P;
        }

        void setState(const Eigen::VectorXd& x) {
            this->x_ = x;
        }

        void setCurrentTimeStamp(const double& stamp) {
            this->dt_ = stamp - this->current_time_stamp_;
            /*std::cout <<"TimeStamp:"<< std::fixed << stamp << std::endl;
            std::cout <<"dt_:"<< std::fixed << dt_ << std::endl;
            std::cout <<"det(s):"<< std::fixed << this->S().determinant() << std::endl;*/
            this->current_time_stamp_ = stamp;

            if (this->dt_ < 0)
                this->dt_ = 1e-4;
        }

        double stamp() const {return this->current_time_stamp_;}
        double likelihood() const {return this->likelihood_;}
        virtual KFBase* clone() {return new KFBase(*this);}
    private:
        /* data */
    protected:
        Eigen::MatrixXd F_; // 狀態轉移矩陣
        Eigen::MatrixXd H_; // 測量矩陣
        Eigen::MatrixXd Q_; // 系統協方差矩陣
        Eigen::MatrixXd R_; // 測量協方差矩陣
        Eigen::MatrixXd J_; // 雅可比陣
        Eigen::MatrixXd P_; // 過程協方差矩陣
        Eigen::MatrixXd S_;
        Eigen::VectorXd x_; // 狀態向量
        Eigen::VectorXd z_; // 測量向量
        
        std::vector<bool> angle_mask_;
        double likelihood_;
        double dt_;
        double current_time_stamp_;
        virtual void updatePrediction() {}
        virtual void updateMeasurement() {}

        static double normalizeAngle(const double raw_angle) { //象限處理
            int n = 0;
            double angle = 0;
            n = raw_angle / (3.141592653 * 2);
            angle = raw_angle - (3.141592653 * 2) * n;

            if (angle > 3.141592653)
                angle = angle - (3.141592653 * 2);
            
            else if (angle <= -3.141592653)
                angle = angle + (3.141592653 * 2);
            

            return angle;
        }
};

class CV : public KFBase {

    private:
        void updatePrediction();
        void updateMeasurement();
    public:
        void init(const double &stamp, const Eigen::VectorXd &x);
        virtual CV* clone() {return new CV(*this);}
        CV();
        ~CV();
};

class CA : public KFBase {

    private:
        void updatePrediction();
        void updateMeasurement();
    public:
        void init(const double &stamp, const Eigen::VectorXd &x);
        virtual CA* clone() {return new CA(*this);}
        CA();
        ~CA();
};

class CT : public KFBase {
    
    private:
        void updatePrediction();
        void updateMeasurement();
        const double yaw_rate_;
    public:
        void init(const double &stamp, const Eigen::VectorXd &x);
        virtual CT* clone() {return new CT(*this);}
        CT(const double& yaw_rate);
        ~CT();
};
#endif
