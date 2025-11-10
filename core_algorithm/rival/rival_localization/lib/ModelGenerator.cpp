#include "rival_localization/ModelGenerator.h"

ModelGenerator::ModelGenerator(){}

ModelGenerator::~ModelGenerator(){}

void ModelGenerator::generateIMMModel(const double &stamp, const Eigen::VectorXd &x, IMM &imm) {

    auto cv  = generateCVModel(stamp, x);
    auto ca  = generateCAModel(stamp, x);
    auto ct0 = generateCTModel(stamp, x, 0.1);
    auto ct1 = generateCTModel(stamp, x, -0.1);

    imm.addModel(ca);
    imm.addModel(cv);
    imm.addModel(ct0);
    imm.addModel(ct1);

    Eigen::MatrixXd transfer_prob = Eigen::MatrixXd::Zero(4,4);

    transfer_prob << 0.8, 0.1, 0.05, 0.05,
                     0.2, 0.7, 0.05, 0.05,
                     0.1, 0.1, 0.75, 0.05,
                     0.1, 0.1, 0.05, 0.75;

    Eigen::VectorXd model_prob = Eigen::VectorXd::Zero(4);

    model_prob << 0.3, 0.2, 0.25, 0.25;

    // Eigen::MatrixXd transfer_prob = Eigen::MatrixXd::Zero(3,3);

    // transfer_prob << 0.7, 0.15, 0.15,
    //                  0.2, 0.75, 0.05,
    //                  0.2, 0.05, 0.75;

    // Eigen::VectorXd model_prob = Eigen::VectorXd::Zero(3);

    // model_prob << 0.4, 0.3, 0.3;

    imm.init(transfer_prob, model_prob, x, stamp);
}

std::shared_ptr<KFBase> ModelGenerator::generateCVModel(const double &stamp, const Eigen::VectorXd &x) {

    std::shared_ptr<KFBase> cv_ptr = std::shared_ptr<KFBase>(new CV());
    cv_ptr->init(stamp, x);
    return cv_ptr;
}

std::shared_ptr<KFBase> ModelGenerator::generateCAModel(const double &stamp, const Eigen::VectorXd &x) {

    std::shared_ptr<KFBase> ca_ptr = std::shared_ptr<KFBase>(new CA());
    ca_ptr->init(stamp, x);
    return ca_ptr;
}

std::shared_ptr<KFBase> ModelGenerator::generateCTModel(const double &stamp, const Eigen::VectorXd &x, const double& yaw_rate) {

    std::shared_ptr<KFBase> ct_ptr = std::shared_ptr<KFBase>(new CT(yaw_rate));
    ct_ptr->init(stamp, x);
    return ct_ptr;
}