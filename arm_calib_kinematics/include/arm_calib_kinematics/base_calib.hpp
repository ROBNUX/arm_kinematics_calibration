#ifndef KINEMATICS_LIB_BASE_CALIB_HPP_
#define KINEMATICS_LIB_BASE_CALIB_HPP_
#include <algorithm>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/SVD>
#include <vector>

#include "robnux_kdl_common/pose.hpp"
#include "robnux_kdl_common/vec.hpp"
#include "robnux_kinematics_map/base_kinematics.hpp"
#include "robnux_kinematics_map/kinematics_exportdecl.h"
#include "simple_motion_logger/Logger.h"

namespace kinematics_lib {

class KINEMATICS_API BaseCalibration {
 public:
  /*
   * @brief robot kinematics calibration using an array of 3 lasers that
   * measures distance changes toward 3 mutually perpendicular blocking planes
   * @param base_offset robot base offset
   * @param tool_offset robot tool offset
   * @param laser2CartMap: 3 by 3 transformation matrix from laser displacement vec
   * to Cartesian displacement vec, i.e. [dx, dy, dz]^T =
   * laser2CartMap[dlaserx, dlasery, dlaserz]^T
   * @param cart_measure, an array of Cartesian vectors
   * each column (x,y,z,R, P, Y, S (branch), T(turn)) is one measurement,
   * Note: depends on the robot vendor and their available data format,
   * (x,y,z) shall be enough to carry on the calibration algorithm
   * @param qa_array: an array of joint angle vectors
   * @param laser_measure: an array of 3d vecs, that represents the measured
   * readings of 3 lasers corresponding to each cart measurement, 
   * each column is one measurement
   * Note: cart_measure, qa_array, laser_measure should all have same number of
   * columns
   * @output: matching error (on testing samples) after compensation
   */
  virtual double LaserDistanceCalib(
      const Eigen::VectorXd& base_offset, const Eigen::VectorXd& tool_offset,
      const EigenDRef<Eigen::Matrix3d>& laser2CartMap,
      const EigenDRef<Eigen::MatrixXd>& cart_measure,
      const EigenDRef<Eigen::MatrixXd>& qa_array,
      const EigenDRef<Eigen::MatrixXd>& laser_measure) = 0;

  /*@brief for verification of laser distance based DH calibration algorithm
   *@param base_offset robot base offset
   *@param tool_offset robot tool offset
   *@param laser2CartMat mapping matrix that maps laser reading differences into cart
   * displacement
   *@param cart_measure, an array of Cartesian vectors
   * each column (x,y,z,R, P, Y, S (branch), T(turn)) is one measurement,
   * Note: depends on the robot vendor and their available data format,
   * (x,y,z) shall be enough to carry on the calibration algorithm
   *@param qa_array: an array of joint angle vectors, each col is one joint vecotr
   * measurement corresponding to each cart measurement
   *@param laser_measure: an array of laser vector measurement
   * Note: cart_measure, qa_array, laser_measure should all have same number of
   * columns
   * @output[0] the average pt error using uncalibrated model
   * @|output[1]| the average pt error using calibrated model; output[1]>0
   * means average pt error after calibration < average pt error
   * before calibration
   */
  virtual Eigen::VectorXd VerifyLaserDistanceCalib(
      const Eigen::VectorXd& base_offset, const Eigen::VectorXd& tool_offset,
      const EigenDRef<Eigen::Matrix3d>& laser2CartMat,
      const EigenDRef<Eigen::MatrixXd>& cart_measure,
      const EigenDRef<Eigen::MatrixXd>& qa_array,
      const EigenDRef<Eigen::MatrixXd>& laser_measure) = 0;

  /*@brief calibrate robot parameters with direct measuring method
   * @param base_offset robot base offset
   * @param tool_offset robot tool offset
   * @param cart_measure, an array of Cartesian vectors, that represents the
   * measured cart position
   * Note: cart_measure, qa_array, laser_measure should all have same number of
   * columns
   * @param qa_array: an array of joint angle vectors
   * @output: matching error (on testing samples) after compensation
   */
  virtual double DirectMesCalib(const Eigen::VectorXd& base_offset,
                                const Eigen::VectorXd& tool_offset,
                                const EigenDRef<Eigen::MatrixXd>& cart_measure,
                                const EigenDRef<Eigen::MatrixXd>& measureMents,
                                const EigenDRef<Eigen::MatrixXd>& qa_array) = 0;

  /*@brief calibrate robot parameters with direct measuring method
   * @param base_offset robot base offset
   * @param tool_offset robot tool offset
   * @param cart_measure, an array of Cartesian vectors, that represents the
   * measured cart position
   * @param measureMents: an array of 3d vecs, that represents the measured robot tool
   * 3d coordinates
   * Note: cart_measure, qa_array, laser_measure should all have same number of
   * columns
   * @param qa_array: an array of joint angle vectors
   * @output[0] the average pt error using uncalibrated model
   * @output[1] output[1]>0 means average pt error after 
   * calibration < average pt error before calibration
   */
  virtual Eigen::VectorXd VerifyDirectMesCalib(
      const Eigen::VectorXd& base_offset, const Eigen::VectorXd& tool_offset,
      const EigenDRef<Eigen::MatrixXd>& cart_measure,
      const EigenDRef<Eigen::MatrixXd>& measureMents,
      const EigenDRef<Eigen::MatrixXd>& qa_array) = 0;

  /*
   * @brief robot tcp calibration algorithm using single mechanical
   * distance sensor
   * @param base_offset robot base offset
   * @param qa_array each column is a measure of joint vector
   * @param measureMents a measurement vector, each element is a distance reading
   * for one measurement
   * @param mes_normal the normal of measurement plane that distance sensor touches
   * and deformed
   * @param tool_offset output of TCP value
   * @return 0 when calib. succeeds, < 0 if any error
   */
  virtual int CalibTCPDistMethod(const Eigen::VectorXd& base_offset,
                                 const EigenDRef<Eigen::MatrixXd>& qa_array,
                                 const EigenDRef<Eigen::VectorXd>& measureMents,
                                 const EigenDRef<Eigen::Vector3d>& mes_normal,
                                 EigenDRef<Eigen::VectorXd>& tool_offset) = 0;

  /*
   * @brief robot base frame calibration algorithm using laser displacement
   * sensors or probe
   * @param jnt_measures a measurement matrix, each col is a joint angle vector
   * when laser or probe hits the edge of workpiece (8pt methods)
   * @param mes_tool tool frame of laser or probe
   * @param orig_base  output base frame using uncalibrated model
   * @param comp_base output base frame under calibrated model
   * @return 0 when calib. succeeds, < 0 if any error
   */
  virtual int CalibBaseFrame(const EigenDRef<Eigen::MatrixXd>& jnt_measures,
                             const Eigen::VectorXd& mes_tool,
                             EigenDRef<Eigen::VectorXd>& orig_base,
                             EigenDRef<Eigen::VectorXd>& comp_base) = 0;

  /*
   * @brief single-pose Cartesian error compensation (return compensated
   * Cartesian pose)
   * @param p desired pose including base/tool and Cartesian data
   * @param canonicalBase base frame (assume it is obtained based upon uncalibrated canonical model)
   * that filtered path references to
   * @param cp modified point (or compensated pose)
   */
  virtual int CpsCartPose(const refPose& p,
                          const Eigen::VectorXd& canonicalBase,
                          refPose& cp) = 0;

  /*
   * @brief single-point error compensation (return compensated joint angles)
   * @param p desired pose including base/tool and Cartesian data
   * @param cq compensated joint angles using calibrated model
   */
  virtual int CpsJnt(const refPose& p, Eigen::VectorXd& cq) = 0;

  /*
   * @brief Trajectory correction through calibrated DH models (for testing
   * purpose)
   * @param calibBase actual user base (or calibrated base) where the desired
   * trajectory references to
   * @param origBase uncalibrated base that filtered path references to
   * @param d_traj: input desired trajectory that robot tries to achieve, NOTE:
   * d_traj is w.r.t. calibBase, and tool
   * @param d_j_traj: supposed desired joint_traj based upon the canonical model
   * @param md_traj:modified desired trajectory that compensates robot model errors
   * using calibrated model
   * @param md_j_traj: modified joint traj corresponding to md_traj (also based upon
   * calibrated model)
   * @param a_traj: actual Cartesian trajectory with md_j_traj as joint input,
   * and calibrated parameters as model (this is for verification whether
   * compensation works
   */
  virtual int CpsRobPath(const Eigen::VectorXd& calibBase,
                         const Eigen::VectorXd& origBase,
                         const Eigen::VectorXd& tool,
                         const EigenDRef<Eigen::MatrixXd>& d_traj,
                         EigenDRef<Eigen::MatrixXd>& md_traj,
                         EigenDRef<Eigen::MatrixXd>& d_j_traj,
                         EigenDRef<Eigen::MatrixXd>& md_j_traj,
                         EigenDRef<Eigen::MatrixXd>& a_traj) = 0;
  /*
   * @brief get parameter set after calibration
   */
  virtual bool GetCalibParamSet(EigenDRef<Eigen::VectorXd>& cal_DH) = 0;

  /*
   * @brief load calibrated set of parameters (e.g. cal_DH is coming from
   * reading from file
   * @param cal_DH calibrated parameters
   */
  virtual bool LoadCalibParamSet(const EigenDRef<Eigen::VectorXd>& cal_DH) = 0;

  /*
   * @brief pick translation and rotational sub-jacobian corresponding to (calib) parameter part
   * @param Jp_t: translation part of the entire Jacobian matrix w.r.t. parameters + joint angles, 
   * i.e. [Jp_t_para, Jp_t_jnt] (but may be mixed columns)
   * @param Jp_r: rotation part of Jacobian matrix w.r.t. parameters + joint angles
     * i.e. [Jp_r_para, Jp_r_jnt] (but may be mixed columns)
   * @param Js_t1: picked translation part of Jacobian matrix w.r.t. parameters only
   * @param Js_r1: picked rotation part of Jacobian matrix w.r.t. parameters only
   * @param reduction: whether to reduce the Jacobian matrix by removing
   * columns corresponding to parameters that are not calibrated
   * @return true if picking succeeds, false if any error
   */
  virtual bool PickSubJacobianForPara(const Eigen::MatrixXd& Jt_p,
                                      const Eigen::MatrixXd& Jp_r,
                                      Eigen::MatrixXd& Js_t1,
                                      Eigen::MatrixXd& Js_r1,
                                      const bool reduction = false) = 0;

  /*
   * @brief set parameter for gradient-type optimization algorithms
    * @param opt_param optimization parameters, e.g. opt_method, sam_region_scale, etc.
     * Note: the content of opt_param depends on the implementation of the
     * calibration algorithm, e.g. for sam-type optimization, opt_param[0] is
     * opt_method = 0, opt_param[1] is sam_region_scale; for sam-adam-type
     * optimization, opt_param[0] is opt_method = 1, opt_param[1] is sam_region_scale
   */
  virtual void setOptParam(const EigenDRef<Eigen::VectorXd>& opt_param);

  virtual ~BaseCalibration() {}

 protected:
  BaseCalibration();
  // optimization method: 0: Sam-type 1. Sam-adam-type
  int opt_method_;
  // sam region scale factor
  double sam_region_scale_;
};

}  // namespace kinematics_lib

#endif /* KINEMATICS_LIB_BASE_CALIB_HPP_ */
