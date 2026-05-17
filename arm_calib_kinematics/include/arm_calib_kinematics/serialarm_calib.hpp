/*
 * File:   serialArm.hpp
 * Author: leon, ROBNUX LLC
 * Email: leon@robnux.net
 *
 * Created on February 12, 2022, 5:38 PM
 */
#ifndef KINEMATICS_LIB_SERIALARM_CALIB_HPP_
#define KINEMATICS_LIB_SERIALARM_CALIB_HPP_
#include "arm_calib_kinematics/base_calib.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "robnux_kinematics_map/serialArm.hpp"
#include "robnux_utilities/decent_alg.hpp"

namespace kinematics_lib {

class KINEMATICS_API SerialArmCalib : public virtual serialArm,
                                      public BaseCalibration {
 public:
  SerialArmCalib();
  SerialArmCalib(size_t DoF);
  SerialArmCalib(const Eigen::VectorXd& kine_para);

  // expose base-class 3-arg UpdateDH overloads so they aren't hidden by the
  // 5-arg overload below
  using serialArm::UpdateDH;

  /*
   * @brief given a whole vector of DH parameter step vector (from matrix
   * computation for solving optimization problem), retrieving each DH
   * parameters at this step
   * @delta_p: a step vector of the entire DH parameter vector
   * @alpha, @a, @theta, @d each individual DH sub-vectors
   */
  void UpdateDH(const Eigen::VectorXd& delta_p, Eigen::VectorXd& alpha,
                Eigen::VectorXd& a, Eigen::VectorXd& theta, Eigen::VectorXd& d);

  /*@brief scaling cols of jacobians (translational and rotational) using joint
   * pitch vector, this allows users to configure the pitch scale factor
   * for each joint (joint / motor angle ratio), note: only cols of jacobians
   * corresponding to theta/d will be scaled
   * @Jt_p: original translational jacobian
   * @Jp_r: original rotational jacobian
   * @Js_t1: trans. jacobian after scaling
   * @Js_r1: rotational jacobian after scaling
   * @reduction whether or not we pick reduced form for each subgroup (
   * translational sub-jacobian Js_t, and/or rotational sub-jacobian Js_r)
   */
  bool PickSubJacobianForPara(const Eigen::MatrixXd& Jt_p,
                              const Eigen::MatrixXd& Jp_r,
                              Eigen::MatrixXd& Js_t1,
                              Eigen::MatrixXd& Js_r1,
                              const bool reduction = false) override;

  /*
   * @brief get parameter set after caliberation
   */
  bool GetCalibParamSet(EigenDRef<Eigen::VectorXd>& cal_DH) override;

  /*
   * @brief load calibrated set of parameters (e.g. cal_DH is coming from
   * reading from file
   */
  bool LoadCalibParamSet(const EigenDRef<Eigen::VectorXd>& cal_DH) override;


  void ResetCalibration() override;

  double LaserDistanceCalib(
      const Eigen::VectorXd& base_offset, const Eigen::VectorXd& tool_offset,
      const EigenDRef<Eigen::Matrix3d>& laser2CartMap,
      const EigenDRef<Eigen::MatrixXd>& cart_measure,
      const EigenDRef<Eigen::MatrixXd>& qa_array,
      const EigenDRef<Eigen::MatrixXd>& laser_measure) override;

  Eigen::VectorXd VerifyLaserDistanceCalib(
      const Eigen::VectorXd& base_offset, const Eigen::VectorXd& tool_offset,
      const EigenDRef<Eigen::Matrix3d>& laser2CartMat,
      const EigenDRef<Eigen::MatrixXd>& cart_measure,
      const EigenDRef<Eigen::MatrixXd>& qa_array,
      const EigenDRef<Eigen::MatrixXd>& laser_measure) override;


  double DirectMesCalib(const Eigen::VectorXd& base_offset,
                        const Eigen::VectorXd& tool_offset,
                        const EigenDRef<Eigen::MatrixXd>& cart_measure,
                        const EigenDRef<Eigen::MatrixXd>& measureMents,
                        const EigenDRef<Eigen::MatrixXd>& qa_array) override;

  Eigen::VectorXd VerifyDirectMesCalib(
      const Eigen::VectorXd& base_offset, const Eigen::VectorXd& tool_offset,
      const EigenDRef<Eigen::MatrixXd>& cart_measure,
      const EigenDRef<Eigen::MatrixXd>& measureMents,
      const EigenDRef<Eigen::MatrixXd>& qa_array) override;


  int CalibTCPDistMethod(const Eigen::VectorXd& base_offset,
                         const EigenDRef<Eigen::MatrixXd>& qa_array,
                         const EigenDRef<Eigen::VectorXd>& measureMents,
                         const EigenDRef<Eigen::Vector3d>& mes_normal,
                         EigenDRef<Eigen::VectorXd>& tool_offset) override;


  int CalibBaseFrame(const EigenDRef<Eigen::MatrixXd>& jnt_measures,
                     const Eigen::VectorXd& mes_tool,
                     EigenDRef<Eigen::VectorXd>& orig_base,
                     EigenDRef<Eigen::VectorXd>& comp_base) override;


  int CpsCartPose(const refPose& p, const Eigen::VectorXd& canonicalBase,
                  refPose& cp) override;


  int CpsJnt(const refPose& p, Eigen::VectorXd& cq) override;


  int CpsRobPath(const Eigen::VectorXd& calibBase,
                 const Eigen::VectorXd& origBase,
                 const Eigen::VectorXd& tool,
                 const EigenDRef<Eigen::MatrixXd>& d_traj,
                 EigenDRef<Eigen::MatrixXd>& md_traj,
                 EigenDRef<Eigen::MatrixXd>& d_j_traj,
                 EigenDRef<Eigen::MatrixXd>& md_j_traj,
                 EigenDRef<Eigen::MatrixXd>& a_traj) override;

  std::string GetName() const override {
    return std::string("serial arm calibration");
  }

 protected:
  /*
   * @brief  utility function, that given initial guess of IK solution with
   * calibrated model (often using IK solution with canonical model), and the
   * desired pose ps, find the ik solution w.r.t. the calibrated model; we will
   * apply homotopoy-based algorithm first (see HomotopyAlg), and then using
   * lease square optimization
   */
  virtual int OptimizeJntAfterCalib(const Eigen::VectorXd& init_jnt,
                                    const refPose& ps,
                                    Eigen::VectorXd& opt_jnt);

  /*
   * @brief homotopy algorithm for finding the IK solution for
   * the calibrated model, given initial guess of IK solution based upon
   * canonical model
   * @init_jnt0, initial guess of IK sol. from canonical model
   * @toolOffset, translational porition of tcp
   * @init_jnt, actual IK sol corresponding to calibrated model
   */
  virtual int HomotopyAlg(const Eigen::VectorXd& init_jnt0,
                          const Eigen::VectorXd& toolOffset,
                          Eigen::VectorXd& init_jnt);

  /*
   * @brief set to using calibrated model
   */
  void SetUsingCalibratedModel(bool useCalibratedModel);

  /*
   * @brief extended DH parameters
   * (alpha_, a_, theta_, d_) are parameter vectors in Craig DH
   * convention, (alpha_c_, a_c_, theta_c_, d_c_), are parameters after
   * calibration
   *
   * Note: for scara robot d_[2], theta_[0], theta_[1], theta_[3]
   * are only offset, the actual joints are feedback joint angles from motors
   * plus the above 4 offsets: e.g.,
   *  theta[0] = actual_j[0] = theta_[0] + jnt_feedback[0]
   *  theta[1] = actual_j[1] = theta_[1] + jnt_feedback[1]
   *  d[2] = actual_j[2] = d_[2] + jnt_feedback[2]
   *  theta[3] = actual_j[3] = theta_[3] + jnt_feedback[3]
   * parameters to be calibrated include
   * alpha[0], alpha[1], alpha[2], alpha[3] (are actual values)
   * a[0], a[1], a[2], a[3] (are actual values)
   *  d_[0], d_[1], d_[2], d_[3] (d_[2] is initial value of prismatic joint)
   * theta_[0], theta_[1], theta_[2], theta_[3] (theta_[0], theta_[1],
   * theta_[3] are initial values of angular values)
   */
  Eigen::VectorXd alpha_, alpha_c_;
  // a offset vector in Craig DH convention
  Eigen::VectorXd a_, a_c_;
  // d offset vector in Craig DH convention
  Eigen::VectorXd d_, d_c_;
  // theta_ in Craig DH convention
  Eigen::VectorXd theta_, theta_c_;
  // pitch: to accommodate joint scaling or directional (CW/CCW) setting
  Eigen::VectorXd pitch_;

  //! optimization algorithm object
  DecentAlg opt_alg_;

  //! cached parameter change
  Eigen::VectorXd delta_p_old_cache_;
  bool resetCache_;
};

}  // namespace kinematics_lib
#endif /* SERIALARM_CALIB_HPP_ */
