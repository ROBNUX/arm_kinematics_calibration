#include "arm_calib_kinematics/singleaxis_calib.hpp"

// register plugin
PLUGINLIB_EXPORT_CLASS(kinematics_lib::SingleAxisCalib,
                       kinematics_lib::BaseKinematicMap)
PLUGINLIB_EXPORT_CLASS(kinematics_lib::SingleAxisCalib,
                       kinematics_lib::BaseCalibration)

namespace kinematics_lib {

SingleAxisCalib::SingleAxisCalib() : SerialArmCalib(1) {}

SingleAxisCalib::SingleAxisCalib(const Eigen::VectorXd& kine_para)
    : SerialArmCalib(kine_para) {}


bool SingleAxisCalib::PickSubJacobianForPara(const Eigen::MatrixXd& Jp_t,
                                             const Eigen::MatrixXd& Jp_r,
                                             Eigen::MatrixXd& Js_t1,
                                             Eigen::MatrixXd& Js_r1,
                                             const bool reduction) {
  size_t row_t = Jp_t.rows();
  size_t col_t = Jp_t.cols();

  Js_t1.resize(row_t, col_t);
  Js_t1 = Jp_t;
  for (size_t i = 0; i < DoF_; i++) {
    Js_t1.col(4 * i + 3) = Jp_t.col(4 * i + 3) * pitch_(i);
  }
  return true;
}



}  // namespace kinematics_lib
