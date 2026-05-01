#include "arm_calib_kinematics/ujnt_calib.hpp"

#include "robnux_kdl_common/pose.hpp"

// register plugin
PLUGINLIB_EXPORT_CLASS(kinematics_lib::UjntCalib,
                       kinematics_lib::BaseKinematicMap)
PLUGINLIB_EXPORT_CLASS(kinematics_lib::UjntCalib,
                       kinematics_lib::BaseCalibration)

namespace kinematics_lib {

UjntCalib::UjntCalib() : SerialArmCalib(2) {}

UjntCalib::UjntCalib(const Eigen::VectorXd &kine_para)
    : SerialArmCalib(kine_para) {}

bool UjntCalib::PickRotSubJacobian(const Eigen::MatrixXd &Jp_t,
                                   const Eigen::MatrixXd &Jp_r,
                                   Eigen::MatrixXd& Js_t,
                                   Eigen::MatrixXd& Js_r) {
  std::ostringstream strs;
  size_t row_r = Jp_r.rows();
  size_t col_r = Jp_r.cols();

  if (row_r < 3 || col_r < DoF_) {
    strs.str("");
    strs << GetName() << " input Jacobian matrices have wrong dimension "
         << " in " << __FUNCTION__ << " at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return false;
  }

  Js_r.resize(3, DoF_);
  for (size_t i = 0; i < DoF_; i++) {
    Js_r.col(i) = Jp_r.col(4 * i + 2) * pitch_(i);
  }
  return true;
}

double UjntCalib::PickRotCartErr(const Eigen::Vector3d &errT,
                                 const Eigen::Vector3d &errR,
                                 Eigen::VectorXd& b) {
  std::ostringstream strs;
  double val = errR.norm();
  b.resize(3);
  b = errR;
  return val;
}


bool UjntCalib::PickSubJacobianForPara(const Eigen::MatrixXd &Jp_t,
                                       const Eigen::MatrixXd &Jp_r,
                                       Eigen::MatrixXd& Js_t1,
                                       Eigen::MatrixXd& Js_r1,
                                       const bool reduction) {
  std::ostringstream strs;
  size_t row_t = Jp_t.rows();
  size_t col_t = Jp_t.cols();
  size_t row_r = Jp_r.rows();
  size_t col_r = Jp_r.cols();

  Js_t1.resize(row_t, col_t);
  Js_t1 = Jp_t;
  for (size_t i = 0; i < DoF_; i++) {
    Js_t1.col(4 * i + 2) = Jp_t.col(4 * i + 2) * pitch_(i);
  }
  Js_r1.resize(row_r, col_r);
  Js_r1 = Jp_r;
  for (size_t i = 0; i < DoF_; i++) {
    Js_r1.col(4 * i + 2) = Jp_r.col(4 * i + 2) * pitch_(i);
  }
  return true;
}

bool UjntCalib::PickRotSubJacobianForPara(const Eigen::MatrixXd& Jp_t,
                                          const Eigen::MatrixXd& Jp_r,
                                          Eigen::MatrixXd& Js_t1,
                                          Eigen::MatrixXd& Js_r1) {
  std::ostringstream strs;
  size_t row_r = Jp_r.rows();
  size_t col_r = Jp_r.cols();
  Js_r1.resize(row_r, col_r);
  Js_r1 = Jp_r;
  for (size_t i = 0; i < DoF_; i++) {
    Js_r1.col(4 * i + 2) = Jp_r.col(4 * i + 2) * pitch_(i);
  }
  return true;
}

}  // namespace kinematics_lib