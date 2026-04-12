#ifndef KINEMATICS_LIB_XYZGantry_CALIB_HPP_
#define KINEMATICS_LIB_XYZGantry_CALIB_HPP_
#include "arm_calib_kinematics/serialarm_calib.hpp"
#include "robnux_kdl_common/vec.hpp"
#include "robnux_kdl_common/rotation.hpp"
#include "simple_motion_logger/Logger.h"
#include "robnux_kinematics_map/XYZ.hpp"
namespace kinematics_lib {
   
class KINEMATICS_API XyzGantryCalib : public XYZGantry, 
                                      public SerialArmCalib {
 public:
  XyzGantryCalib();

  bool PickSubJacobianForPara(const Eigen::MatrixXd& Jp_t,
                              const Eigen::MatrixXd& Jp_r, 
                              Eigen::MatrixXd& Js_t1, 
                              Eigen::MatrixXd& Js_r1,
                              const bool reduction = false) override; 
   
  virtual std::string GetName() const {
    return std::string("XYZGantry calib");
  }
};

}

#endif
