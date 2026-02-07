#include "arm_calib_kinematics/base_calib.hpp"

namespace kinematics_lib {

BaseCalibration::BaseCalibration() {
    opt_method_ = 1;
    sam_region_scale_ = 3.0;
}

void BaseCalibration::setOptParam(const EigenDRef<Eigen::VectorXd>& opt_param) {
    opt_method_ = opt_param[0];
    sam_region_scale_ = opt_param[1];
}

}