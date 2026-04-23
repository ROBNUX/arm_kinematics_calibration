#ifndef KINEMATICS_LIB_SIXAXIS_CALIB_HPP_
#define KINEMATICS_LIB_SIXAXIS_CALIB_HPP_
#include "arm_calib_kinematics/serialarm_calib.hpp"
#include "robnux_kinematics_map/sixaxis_1.hpp"

namespace kinematics_lib {

class KINEMATICS_API SixAxisCalib :  public SixAxis_1,
                                     public SerialArmCalib {
 public:
  SixAxisCalib();

  bool PickSubJacobianForPara(const Eigen::MatrixXd& Jp_t,
                              const Eigen::MatrixXd& Jp_r,
                              Eigen::MatrixXd* Js_t1, Eigen::MatrixXd* Js_r1,
                              const bool reduction = false) override;

  std::string GetName() const { return std::string("SixAxis calib"); }
};

}  // namespace kinematics_lib

#endif
