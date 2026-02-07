#ifndef KINEMATICS_LIB_XYZGantry_CALIB_HPP_
#define KINEMATICS_LIB_XYZGantry_CALIB_HPP_
#include "arm_calib_kinematics/serialarm_calib.hpp"
#include "robnux_kdl_common/vec.hpp"
#include "robnux_kdl_common/rotation.hpp"
#include "simple_motion_logger/Logger.h"
namespace kinematics_lib {
   
class KINEMATICS_API XyzGantryCalib : public SerialArmCalib {
 public:
  XyzGantryCalib();
  XyzGantryCalib(const Eigen::VectorXd& kine_para);

  int CartToJnt(const Pose& p, Eigen::VectorXd* q) override; 

  /*
   * @brief set turn and config flags based upon actual theta and d values
   * Note: (1) turns are only meaning for revolute joints, and will be 0
   * for prismatic joint
   *   (2) branchFlags (configuration branch of IK solution) is related to
   *  both theta and d
   */
  void  UpdateConfigTurn(const Eigen::VectorXd& theta,
                         const Eigen::VectorXd& d,
                         std::vector<int>* branchFlags,
                         std::vector<int>* jointTurns) const override;
                                  
  /*
   * @brief pick a submatrix of the full Jacobian that corresponds to
   * robot usual jacobian (joint only jacobian)
   * @reduction whether or not we pick reduced form for each subgroup (
   * translational sub-jacobian Js_t, and/or rotational sub-jacobian Js_r),
   * e.g, 2-dof translational, Js_t might be 2 * n submatrix;
   * as another example 4-dof scara, Js_r might be 1 * n submatrix
   */
  bool PickSubJacobian(const Eigen::MatrixXd& Jp_t,
                       const Eigen::MatrixXd& Jp_r,
                       Eigen::MatrixXd* Js_t,
                       Eigen::MatrixXd* Js_r,
                       const bool reduction = false) override;
   
  /*
   * @brief pick a submatrix of the full Jacobian that corresponds to
   * robot DH parameters (parameter only jacobian)
   * @reduction whether or not we pick reduced form for each subgroup (
   * translational sub-jacobian Js_t, and/or rotational sub-jacobian Js_r),
   * e.g, 2-dof translational, Js_t might be 2 * n submatrix;
   * as another example 4-dof scara, Js_r might be 1 * n submatrix
   */
  bool PickSubJacobianForPara(const Eigen::MatrixXd& Jp_t,
                              const Eigen::MatrixXd& Jp_r, 
                              Eigen::MatrixXd* Js_t1, 
                              Eigen::MatrixXd* Js_r1,
                              const bool reduction = false) override; 
   
  /*
   * @brief given trans and euler angle error,
   * pick a sub error vector matching robot model,
   * and also return the absolute error norm
   * @errT, translational error vector
   * @errR, rotational error vector
   * @b output a sub error vector matching robot model (e.g. 4d error
   * vector for a scara robot)
   * @reduction whether or not we pick reduced form
   * return norm of b
   */
  double  PickCartErr(const Eigen::Vector3d& errT,
                      const Eigen::Vector3d& errR, 
                      Eigen::VectorXd* b,
                      const bool reduction = false) override;
  /* 
   * @brief updating actual DH parameters based upon joint feedback
   * orig_dh=<alpha_1, alpha_2, .., alpha_k, a_1,...a_k, theta_1,...,theta_k,
   * d_1,...,d_k> jnt angles def. depends on specific robot type
   */ 
  void UpdateDH(const Eigen::VectorXd& orig_dh,
                const Eigen::VectorXd& jnt,
                Eigen::VectorXd* new_dh) const override;

  /*
   * @brief updating theta and d vector of actual DH parameters based upon
   *joint feedback
   * Note: theta, d has their initial values, which will be updated
   * based upon jnt vector input
   */
  void UpdateDH(const Eigen::VectorXd& jnt,
                Eigen::VectorXd* theta,
                Eigen::VectorXd* d
                ) const;

  virtual std::string GetName() const {
    return std::string("XYZGantry calib");
  }
};

}

#endif
