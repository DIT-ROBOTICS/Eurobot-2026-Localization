#ifndef MODEL_GENERATOR_H_
#define MODEL_GENERATOR_H_
#include "imm_filter/IMM.h"
#include "imm_filter/EKF.h"

class ModelGenerator{

    private:
        /* data */
    public:
        ModelGenerator();
        ~ModelGenerator();

        void generateIMMModel(const double &stamp, const Eigen::VectorXd &x, IMM &imm);
        static std::shared_ptr<KFBase> generateCVModel(const double &stamp, const Eigen::VectorXd &x);
        static std::shared_ptr<KFBase> generateCAModel(const double &stamp, const Eigen::VectorXd &x);
        static std::shared_ptr<KFBase> generateCTModel(const double &stamp, const Eigen::VectorXd &x, const double& yaw_rate);
};
#endif