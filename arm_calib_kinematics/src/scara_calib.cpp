#include "arm_calib_kinematics/scara_calib.hpp"

#include "robnux_kdl_common/pose.hpp"

// register plugin
PLUGINLIB_EXPORT_CLASS(kinematics_lib::ScaraCalib,
                       kinematics_lib::BaseKinematicMap)
PLUGINLIB_EXPORT_CLASS(kinematics_lib::ScaraCalib,
                       kinematics_lib::BaseCalibration)

namespace kinematics_lib {

ScaraCalib::ScaraCalib() : serialArm(4), Scara(), SerialArmCalib(4) {}

ScaraCalib::ScaraCalib(const Eigen::VectorXd& kine_para)
    : serialArm((kine_para.size() - 7) / 4),
      Scara(),
      SerialArmCalib(kine_para) {}

bool ScaraCalib::PickSubJacobianForPara(const Eigen::MatrixXd &Jp_t,
                                        const Eigen::MatrixXd &Jp_r,
                                        Eigen::MatrixXd& Js_t1,
                                        Eigen::MatrixXd& Js_r1,
                                        const bool reduction) {
  size_t row_t = Jp_t.rows();
  size_t col_t = Jp_t.cols();
  size_t row_r = Jp_r.rows();
  size_t col_r = Jp_r.cols();

  Js_t1.resize(row_t, col_t);
  Js_t1 = Jp_t;

  if (reduction) {
    Js_r1.resize(1, col_r);
    Js_r1.row(0) = Jp_r.row(2);
  } else {
    Js_r1.resize(row_r, col_r);
    Js_r1 = Jp_r;
  }

  for (size_t i = 0; i < DoF_; i++) {
    if (i != 2) {
      Js_t1.col(4 * i + 2) *= pitch_(i);
      Js_r1.col(4 * i + 2) *= pitch_(i);
    } else {  // for d[2] parameter
      Js_t1.col(4 * i + 3) *= pitch_(i);
      Js_r1.col(4 * i + 3) *= pitch_(i);
    }
  }
  return true;
}

}  // namespace kinematics_lib
