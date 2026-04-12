#ifndef KINEMATICS_LIB_UJNT_CALIB_HPP_
#define KINEMATICS_LIB_UJNT_CALIB_HPP_
#include "arm_calib_kinematics/serialarm_calib.hpp"
#include "robnux_kdl_common/rotation.hpp"
#include "robnux_kdl_common/vec.hpp"
#include "simple_motion_logger/Logger.h"

namespace kinematics_lib {
//! UJNT (R-R) modules, which combined with XYZ constitutes the 5-axis module
class KINEMATICS_API UjntCalib : public SerialArmCalib {
 public:
  UjntCalib();

  /*
   * @brief pick a submatrix of the full rotational Jacobian that corresponds to
   * robot usual jacobian (joint only jacobian)
   */
  bool PickRotSubJacobian(const Eigen::MatrixXd &Jp_t,
                          const Eigen::MatrixXd &Jp_r, Eigen::MatrixXd *Js_t,
                          Eigen::MatrixXd *Js_r);

  /*
   * @brief given trans and euler angle error,
   * pick a sub error vector for orientation (or rotation) error,
   * and also return the absolute error norm
   * @errT, translational error vector
   * @errR, rotational error vector
   * @b output a sub error vector matching robot model (e.g. 4d error
   * vector for a scara robot)
   * @reduction whether or not we pick reduced form
   * return norm of b
   */
  double PickRotCartErr(const Eigen::Vector3d &errT,
                        const Eigen::Vector3d &errR, Eigen::VectorXd *b);

  /*
   * @brief pick a submatrix of the full Jacobian that corresponds to
   * robot DH parameters (parameter only jacobian)
   * @reduction whether or not we pick reduced form for each subgroup (
   * translational sub-jacobian Js_t, and/or rotational sub-jacobian Js_r),
   * e.g, 2-dof translational, Js_t might be 2 * n submatrix;
   * as another example 4-dof scara, Js_r might be 1 * n submatrix
   */
  bool PickSubJacobianForPara(const Eigen::MatrixXd &Jp_t,
                              const Eigen::MatrixXd &Jp_r,
                              Eigen::MatrixXd *Js_t1, Eigen::MatrixXd *Js_r1,
                              const bool reduction = false) override;

  /*
   * @brief pick a submatrix of the full Jacobian that corresponds to
   * rotational Jacobian
   */ 
  bool PickRotSubJacobianForPara(const Eigen::MatrixXd &Jp_t,
                                 const Eigen::MatrixXd &Jp_r,
                                 Eigen::MatrixXd *Js_t1,
                                 Eigen::MatrixXd *Js_r1);

  //! get name
  virtual std::string GetName() const {
    return std::string("UJNT calibration");
  }
};

}  // namespace kinematics_lib

#endif
