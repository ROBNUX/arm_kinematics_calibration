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
#include "pluginlib/class_list_macros.h"
#include "robnux_kinematics_map/base_kinematics.hpp"
#include "robnux_utilities/decent_alg.hpp"

namespace kinematics_lib {

class KINEMATICS_API SerialArmCalib : public BaseKinematicMap,
                                      public BaseCalibration {
 public:
  SerialArmCalib();

  /*
   * @brief constructor with kinematics parameters input, e.g. DH parameters
   * @param kine_para kinematics parameters, e.g. DH parameters, the content
   * of which depends on specific robot type and the design of calibration 
   * algorithm, e.g. for DH parameter calibration, kine_para is a vector of 
   *  DH parameters in the order of [alpha_1,...,alpha_k, 
   *  a_1,...a_k, theta_1,...,theta_k, d_1,...,d_k]
   */
  SerialArmCalib(const Eigen::VectorXd& kine_para);

  /*
    * @brief set kinematics parameters, e.g. DH parameters
    * @param kine_para kinematics parameters, e.g. DH parameters, the content
    * of which depends on specific robot type and the design of calibration 
    * algorithm, e.g. for DH parameter calibration, kine_para is a vector of 
    *  DH parameters in the order of [alpha_1,...,alpha_k, 
    *  a_1,...a_k, theta_1,...,theta_k, d_1,...,d_k]
  */
  void SetGeometry(const Eigen::VectorXd& kine_para);

  /*
    * @brief forward kinematics with given joint angles
    * @param q joint angle vector
    * @param p output pose of TCP
  */
  int JntToCart(const Eigen::VectorXd& q, Pose* p) override;

  /*
    * @brief forward kinematics with given joint angles and joint velocities
    * @param q joint angle vector
    * @param qdot joint velocity vector
    * @param p output pose of TCP
    * @param v output twist of TCP
   */
  int JntToCart(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot, Pose* p,
                Twist* v) override;

   /*
    * @brief forward kinematics with given joint angles, joint velocities and
    * joint accelerations
    * @param q joint angle vector
    * @param qdot joint velocity vector
    * @param qddot joint acceleration vector
    * @param p output pose of TCP
    * @param v output twist of TCP
    * @param a output acceleration of TCP
   */
  int JntToCart(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot,
                const Eigen::VectorXd& qddot, Pose* p, Twist* v,
                Twist* a) override;

  /*
   * @brief inverse kinematics with given desired pose of TCP
   * @param p desired pose of TCP
   * @param q output joint angle vector
   */
  int CartToJnt(const Pose& p, Eigen::VectorXd* q) override;

  /*
   * @brief updating theta and d vector of actual DH parameters based upon
   *joint feedback
    * @param jnt joint angle vector whose def. depends on specific robot type
    * @param theta output theta vector of DH parameters
    * @param d output d vector of DH parameters
   * Note: theta, d has their initial values, which will be updated
   * based upon jnt vector input
   */
  virtual void UpdateDH(const Eigen::VectorXd& jnt, Eigen::VectorXd& theta,
                        Eigen::VectorXd& d) const;

  /*
   * @brief set turn and config flags based upon actual theta and d values
   * Note: (1) turns are only meaning for revolute joints, and will be 0
   * for prismatic joint
   *   (2) branchFlags (configuration branch of IK solution) is related to
   *  both theta and d
   * @param theta theta vector of DH parameters
   * @param d d vector of DH parameters
   * @param branchFlags output configuration branch flags
   * @param jointTurns output joint turns
   */
  virtual void UpdateConfigTurn(const Eigen::VectorXd& theta,
                                const Eigen::VectorXd& d,
                                std::vector<int>& branchFlags,
                                std::vector<int>& jointTurns) const;

  /*
   * @brief for serialArm, there is a generic way to compute translational
   * and rotational Jacobian
   * @kine_para: kinematics DH parameters
   * @p output pose of TCP
   * @Jp_t 3 * nDHs translational Jacobian matrix
   * @Jp_r 3 * nDHs rotational Jacobian matrix
   * @reduction whether we shall default robot base(which is identify matrix)
   */
  virtual int CalcJacobian(const Eigen::VectorXd& kine_para, Pose& p,
                           Eigen::MatrixXd* Jp_t, Eigen::MatrixXd& Jp_r,
                           const bool default_rob_base = false);

  /*
   * @brief pick a submatrix of the full Jacobian that corresponds to
   * robot usual jacobian (joint only jacobian)
   * @reduction whether or not we pick reduced form for each subgroup (
   * translational sub-jacobian Js_t, and/or rotational sub-jacobian Js_r),
   * e.g, 2-dof translational, Js_t might be 2 * n submatrix;
   * as another example 4-dof scara, Js_r might be 1 * n submatrix
   */
  virtual bool PickSubJacobian(const Eigen::MatrixXd& Jp_t,
                               const Eigen::MatrixXd& Jp_r,
                               Eigen::MatrixXd& Js_t, Eigen::MatrixXd& Js_r,
                               const bool reduction = false);

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
  virtual double PickCartErr(const Eigen::Vector3d& errT,
                             const Eigen::Vector3d& errR, Eigen::VectorXd* b,
                             const bool reduction = false);

  int CartToJnt(const Pose& p, const Twist& v, Eigen::VectorXd* q,
                Eigen::VectorXd* qdot) override;

  int CartToJnt(const Pose& p, const Twist& v, const Twist& a,
                Eigen::VectorXd* q, Eigen::VectorXd* qdot,
                Eigen::VectorXd* qddot) override;

  int CalcPassive(const Eigen::VectorXd& q, const Pose& p,
                  Eigen::VectorXd* qpassive) override {
    return 0;
  }

  /*
   * @brief reset calibration (make alpha_c_ = alpha_, a_c_ = a_, ..., etc)
   * so allow recalibration
   */
  bool resetCalibration();

  /*@brief updating actual DH parameters based upon joint feedback
   *@orig_dh=<alpha_1, alpha_2, .., alpha_k, a_1,...a_k,
   * theta_1,...,theta_k, d_1,...,d_k>
   *@jnt jnt angle vector whose def. depends on specific robot type
   *@new_dh: output updated dh parameters
   */
  virtual void UpdateDH(const Eigen::VectorXd& orig_dh,
                        const Eigen::VectorXd& jnt,
                        Eigen::VectorXd* new_dh) const;

  /*
   * @brief given a whole vector of DH parameter step vector (from matrix
   * computation for solving optimization problem), retrieving each DH
   * parameters at this step
   * @delta_p: a step vector of the entire DH parameter vector
   * @alpha, @a, @theta, @d, @beta: each individual DH sub-vectors
   */
  void UpdateDH(const Eigen::VectorXd& delta_p, Eigen::VectorXd* alpha,
                Eigen::VectorXd* a, Eigen::VectorXd* theta, Eigen::VectorXd* d,
                Eigen::VectorXd* beta);

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
                              Eigen::MatrixXd* Js_t1, Eigen::MatrixXd* Js_r1,
                              const bool reduction = false) override;

  /*
   * @brief get parameter set after caliberation
   */
  bool GetCalibParamSet(EigenDRef<Eigen::VectorXd>* cal_DH) override;

  /*
   * @brief load calibrated set of parameters (e.g. cal_DH is coming from
   * reading from file
   */
  bool LoadCalibParamSet(const EigenDRef<Eigen::VectorXd>& cal_DH) override;

  /*
   * @brief robot kinematics calibration using an array of 3 lasers that
   * measures distance change toward the blocking plane
   * @base_offset robot base offset
   * @tool_offset robot tool offset
   * @laser2CartMap: 3 by 3 transformation matrix from laser displacement vec
   * to Cartesian displacement vec, i.e. [dx, dy, dz]^T =
   * laser2CartMap[dlx, dly, dlz]^T
   * @cart_measure, an array of Cartesian vectors, that represents the
   * measured cart position
   * @qd_array: an array of joint angle vectors
   * @laser_measure: an array of 3d vecs, that represents the measured
   * readings of a number of lasers
   * Note: cart_measure, qa_array, laser_measure should all have same number of
   * columns
   * @output: matching error (on testing samples) after compensation
   */
  double LaserDistanceCalib(
      const Eigen::VectorXd& base_offset, const Eigen::VectorXd& tool_offset,
      const EigenDRef<Eigen::Matrix3d>& laser2CartMap,
      const EigenDRef<Eigen::MatrixXd>& cart_measure,
      const EigenDRef<Eigen::MatrixXd>& qa_array,
      const EigenDRef<Eigen::MatrixXd>& laser_measure) override;

  /*@brief for verification of laser distance based DH calibration algorithm
   *@base_offset robot base offset
   *@tool_offset robot tool offset
   *@laser2CartMat mapping matrix that maps laser reading differences into cart
   * displacement
   *@cart_measure, an array of Cartesian vectors
   * each column (x,y,z,R, P, Y, S (branch), T(turn)) is one measurement,
   * Note: depends on the robot vendor and their available data format,
   * (x,y,z) shall be enough to carry on the calibration algorithm
   *@qa_array: an array of joint angle vectors, each col is one joint vecotr
   * measurement corresponding to each cart measurement
   *@laser_measure: an array of laser vector measurement
   * Note: cart_measure, qa_array, laser_measure should all have same number of
   * columns
   * @output[0] the average pt error using uncalibrated model
   * @|output[1]| the average pt error using calibrated model; output[1]>0
   * means average pt error after calibration < average pt error
   * before calibration
   */
  Eigen::VectorXd VerifyLaserDistanceCalib(
      const Eigen::VectorXd& base_offset, const Eigen::VectorXd& tool_offset,
      const EigenDRef<Eigen::Matrix3d>& laser2CartMat,
      const EigenDRef<Eigen::MatrixXd>& cart_measure,
      const EigenDRef<Eigen::MatrixXd>& qa_array,
      const EigenDRef<Eigen::MatrixXd>& laser_measure) override;

  /*@brief calibrate robot parameters with direct measuring method
   * @base_offset robot base offset
   * @tool_offset robot tool offset
   * @cart_measure, an array of Cartesian vectors, that represents the
   * measured cart position
   * Note: cart_measure, qa_array, laser_measure should all have same number of
   * columns
   * @qa_array: an array of joint angle vectors
   * @output: matching error (on testing samples) after compensation
   */
  double DirectMesCalib(const Eigen::VectorXd& base_offset,
                        const Eigen::VectorXd& tool_offset,
                        const EigenDRef<Eigen::MatrixXd>& cart_measure,
                        const EigenDRef<Eigen::MatrixXd>& measureMents,
                        const EigenDRef<Eigen::MatrixXd>& qa_array) override;

  /*@brief calibrate robot parameters with direct measuring method
   * @base_offset robot base offset
   * @tool_offset robot tool offset
   * @cart_measure, an array of Cartesian vectors, that represents the
   * measured cart position
   * @measureMents: an array of 3d vecs, that represents the measured robot tool
   * 3d coordinates
   * Note: cart_measure, qa_array, laser_measure should all have same number of
   * columns
   * @qa_array: an array of joint angle vectors
   * @output[0] the average pt error using uncalibrated model
   * @|output[1]| the average pt error using calibrated model; output[1]>0
   * means average pt error after calibration < average pt error
   * before calibration
   */
  Eigen::VectorXd VerifyDirectMesCalib(
      const Eigen::VectorXd& base_offset, const Eigen::VectorXd& tool_offset,
      const EigenDRef<Eigen::MatrixXd>& cart_measure,
      const EigenDRef<Eigen::MatrixXd>& measureMents,
      const EigenDRef<Eigen::MatrixXd>& qa_array) override;

  /*
   * @brief robot tcp calibration algorithm using single mechanical
   * distance sensor
   * @base_offset robot base offset
   * @qa_array each column is a measure of joint vector
   * @measureMents a measurement vector, each element is a distance reading
   * for one measurement
   * @mes_normal the normal of measurement plane that distance sensor touches
   * and deformed
   * @tool_offset output of TCP value
   * @return 0 when calib. succeeds, < 0 if any error
   */
  int CalibTCPDistMethod(const Eigen::VectorXd& base_offset,
                         const EigenDRef<Eigen::MatrixXd>& qa_array,
                         const EigenDRef<Eigen::VectorXd>& measureMents,
                         const EigenDRef<Eigen::Vector3d>& mes_normal,
                         EigenDRef<Eigen::VectorXd>* tool_offset) override;

  /*
   * @brief robot base frame calibration algorithm using laser displacement
   * sensors or probe
   * @jnt_measures a measurement matrix, each col is a joint angle vector
   * when laser or probe hits the edge of workpiece (8pt methods)
   * @mes_tool tool frame of laser or probe
   * @orig_base  output base frame using uncalibrated model
   * @comp_base output base frame under calibrated model
   * @return 0 when calib. succeeds, < 0 if any error
   */
  int CalibBaseFrame(const EigenDRef<Eigen::MatrixXd>& jnt_measures,
                     const Eigen::VectorXd& mes_tool,
                     EigenDRef<Eigen::VectorXd>* orig_base,
                     EigenDRef<Eigen::VectorXd>* comp_base) override;

  /*
   * @brief single-point Cartesian error compensation (return compensated
   * Cartesian pose)
   * @p desired pose including base/tool and Cartesian data
   * @canonicalBase base frame (calibrated based upon uncalibrated model)
   * that filtered path references to
   * @cp modified point (or compensated pose)
   */
  int CpsCartPose(const refPose& p, const Eigen::VectorXd& canonicalBase,
                  refPose* cp) override;

  /*
   * @brief single-point error compensation (return compensated joint angles)
   * @p desired pose including base/tool and Cartesian data
   * @cq compensated joint angles using calibrated model
   */
  int CpsJnt(const refPose& p, Eigen::VectorXd* cq) override;

  /*
   * @brief Trajectory correction through calibrated DH models (for testing
   * purpose)
   * @calibBase actual user base (or calibrated base) where the desired
   * trajectory references to
   * @origBase uncalibrated base that filtered path references to
   * @d_traj: input desired trajectory that robot tries to achieve, NOTE:
   * d_traj is w.r.t. calibBase, and tool
   * @d_j_traj: supposed desired joint_traj based upon the canonical model
   * @md_traj:modified desired trajectory that compensates robot model errors
   * using calibrated model
   * @md_j_traj: modified joint traj corresponding to md_traj (also based upon
   * calibrated model)
   * @a_traj: actual Cartesian trajectory with md_j_traj as joint input,
   * and calibrated parameters as model (this is for verification whether
   * compensation works
   */
  virtual int CpsRobPath(const Eigen::VectorXd& calibBase,
                         const Eigen::VectorXd& origBase,
                         const Eigen::VectorXd& tool,
                         const EigenDRef<Eigen::MatrixXd>& d_traj,
                         EigenDRef<Eigen::MatrixXd>* md_traj,
                         EigenDRef<Eigen::MatrixXd>* d_j_traj,
                         EigenDRef<Eigen::MatrixXd>* md_j_traj,
                         EigenDRef<Eigen::MatrixXd>* a_traj) override;

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
                                    Eigen::VectorXd* opt_jnt);

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
                          Eigen::VectorXd* init_jnt);

  /*
   * @brief set to using calibrated model
   */
  void SetUsingCalibratedModel(bool useCalibratedModel);

  /*
   * @brief extended DH parameters
   * (alpha_, a_, theta_, d_, beta_) are parameter vectors in Craig DH
   * convention, (alpha_c_, a_c_, theta_c_, d_c_, beta_c_), are parameters after
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
  // new added:  beta_ are extra angles that models the rotation of z_i r.t.
  // z_{i-1} about y_i
  Eigen::VectorXd beta_, beta_c_;
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
