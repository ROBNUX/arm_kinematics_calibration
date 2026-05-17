#include "arm_calib_kinematics/sixaxis_calib.hpp"
// register plugin
PLUGINLIB_EXPORT_CLASS(kinematics_lib::SixAxisCalib,
                       kinematics_lib::BaseKinematicMap)
PLUGINLIB_EXPORT_CLASS(kinematics_lib::SixAxisCalib,
                       kinematics_lib::BaseCalibration)
namespace kinematics_lib {

SixAxisCalib::SixAxisCalib() : serialArm(6), SixAxis_1(), SerialArmCalib(6) {}

SixAxisCalib::SixAxisCalib(const Eigen::VectorXd& kine_para)
    : serialArm((kine_para.size() - 7) / 4),
      SixAxis_1(),
      SerialArmCalib(kine_para) {}


bool SixAxisCalib::PickSubJacobianForPara(const Eigen::MatrixXd &Jp_t,
                                          const Eigen::MatrixXd &Jp_r,
                                          Eigen::MatrixXd &Js_t1,
                                          Eigen::MatrixXd &Js_r1,
                                          const bool reduction) {
  size_t row_t = Jp_t.rows();
  size_t col_t = Jp_t.cols();
  size_t row_r = Jp_r.rows();
  size_t col_r = Jp_r.cols();

  Js_t1.resize(row_t, col_t);
  Js_t1 = Jp_t;
  Js_r1.resize(row_r, col_r);
  Js_r1 = Jp_r;
  for (size_t i = 0; i < DoF_; i++) {
    // for theta[i] parameters
    Js_t1.col(4 * i + 2) = Jp_t.col(4 * i + 2) * pitch_(i);
    Js_r1.col(4 * i + 2) = Jp_r.col(4 * i + 2) * pitch_(i);
  }
  return true;
}

}  // namespace kinematics_lib
