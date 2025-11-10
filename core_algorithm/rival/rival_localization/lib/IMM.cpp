#include "rival_localization/IMM.h"

IMM::IMM():model_num_(0),current_time_stamp_(-1){}

IMM::~IMM(){}

void IMM::addModel(const std::shared_ptr<KFBase>& model) {
    
    this->models_.push_back(model);
    this->model_num_++;
}

void IMM::init (const Eigen::MatrixXd& transfer_prob, 
                const Eigen::VectorXd& model_prob, 
                const Eigen::VectorXd& x,
                const double& dt) {

    // RCLCPP_INFO(rclcpp::get_logger("IMM"),"imm filter initializing...");

    if (this->model_num_ == 0) {
        RCLCPP_WARN(rclcpp::get_logger("IMM"),"No valid model.");
        exit(1);
    }

    if (transfer_prob.cols() != this->model_num_ || transfer_prob.rows() != this->model_num_){

        RCLCPP_WARN(rclcpp::get_logger("IMM"),"Dimension of transfer probability matrix is not equal to number of models.");
        exit(1);
    }
    if (model_prob.size() != this->model_num_){   

        RCLCPP_WARN(rclcpp::get_logger("IMM"),"Dimension of model probability vector is not equal to number of models.");
        exit(1);
    }

    this->state_num_ = x.size();
    this->X_.resize(this->state_num_, this->model_num_);

    Eigen::VectorXd ZERO = Eigen::VectorXd::Zero(6);

    for (size_t i = 0; i < model_num_; i++)
        this->X_.col(i) = this->models_[i]->x();
    
    this->transfer_prob_ = transfer_prob;
    this->model_prob_ = model_prob;
    this->x_ = x;
    this->c_.resize(this->model_num_);

    stateInteraction();

    // RCLCPP_INFO(rclcpp::get_logger("IMM"),"imm filter initialized");
}

void IMM::stateInteraction() {
    
    this->c_ = Eigen::VectorXd::Zero(this->model_num_);

    for (size_t j = 0; j < this->model_num_; j++) {

        for (size_t i = 0; i < this->model_num_; i++)
            this->c_(j) += this->transfer_prob_(i, j) * this->model_prob_(i);
    }

    Eigen::MatrixXd U = Eigen::MatrixXd::Zero(this->model_num_, this->model_num_);

    for (size_t i = 0; i < model_num_; i++)
        this->X_.col(i) = this->models_[i]->x();
    
    Eigen::MatrixXd X = this->X_;
    this->X_.fill(0);

    for (size_t j = 0; j < this->model_num_; j++) {

        for (size_t i = 0; i < this->model_num_; i++) {

            U(i, j) = 1.0 / this->c_(j) * this->transfer_prob_(i, j) * this->model_prob_(i);
            this->X_.col(j) += X.col(i) * U(i, j);
        }
    } 

    for (size_t i = 0; i < this->model_num_; i++) {

        Eigen::MatrixXd P = Eigen::MatrixXd::Zero(this->state_num_, this->state_num_);
        
        for (size_t j = 0; j < this->model_num_; j++) {

            Eigen::VectorXd s = X.col(i) - this->X_.col(j);
            P += U(i,j) * (this->models_[i]->P() + s * s.transpose());
        }
        
        this->models_[i]->setStateCoveriance(P);
        this->models_[i]->setState(this->X_.col(i));
    }
}

void IMM::updateState(const double& stamp, const Eigen::VectorXd* z) {

    current_time_stamp_ = stamp;

    for (size_t i = 0; i < this->model_num_; i++) {

        this->models_[i]->predict(stamp);

        if (nullptr != z)
            this->models_[i]->update(*z);
    }
}

void IMM::updateModelProb() {

    double c_sum = 0;

    for (size_t i = 0; i < this->model_num_; i++)
        c_sum += this->models_[i]->likelihood() * this->c_(i);
    
    for (size_t i = 0; i < this->model_num_; i++)
        this->model_prob_(i) = 1 / c_sum * this->models_[i]->likelihood() * this->c_(i);
}

void IMM::estimateFusion() {

    this->x_ = this->X_ * this->model_prob_;

    for (size_t i = 0; i < this->model_num_; i++) {
        Eigen::MatrixXd v = this->X_.col(i) - this->x_;      
        this->P_ = this->models_[i]->P() + v * v.transpose() * this->model_prob_[i];
    }
}

void IMM::updateOnce(const double& stamp, const Eigen::VectorXd* z) {

    if (z == nullptr) {
        updateState(stamp);
        stateInteraction();
        estimateFusion();
    }
    else {
        updateState(stamp, z);
        updateModelProb();
        stateInteraction();
        estimateFusion();
    }

    // std::cout<<"ca: "<<model_prob_[0]<<" ,cv: "<<model_prob_[1]<<" ,t0: "<<model_prob_[2]<<" ,t1: "<<model_prob_[3]<<std::endl;
    // std::cout<<"cv: "<<model_prob_[0]<<" ,t0: "<<model_prob_[1]<<" ,t1: "<<model_prob_[2]<<std::endl;
}