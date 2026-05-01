#ifndef KINEMATICS_LIB_SCARA_CALIB_HPP_
#define KINEMATICS_LIB_SCARA_CALIB_HPP_
#include "arm_calib_kinematics/serialarm_calib.hpp"
#include "robnux_kinematics_map/scara.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "robnux_kdl_common/rotation.hpp"
#include "robnux_kdl_common/vec.hpp"
#include "simple_motion_logger/Logger.h"

namespace kinematics_lib {

class KINEMATICS_API ScaraCalib : public Scara,
                                  public SerialArmCalib {
 public:
  ScaraCalib();
  ScaraCalib(const Eigen::VectorXd &kine_para);

  bool PickSubJacobianForPara(const Eigen::MatrixXd& Jp_t,
                              const Eigen::MatrixXd& Jp_r,
                              Eigen::MatrixXd& Js_t1,
                              Eigen::MatrixXd& Js_r1,
                              const bool reduction = false) override;


  virtual std::string GetName() const { return std::string("Scara calib"); }
};

}  // namespace kinematics_lib

#endif
