#ifndef KINEMATICS_LIB_XYZ_UR_CALIB_HPP_
#define KINEMATICS_LIB_XYZ_UR_CALIB_HPP_
#include "arm_calib_kinematics/base_calib.hpp"
#include "arm_calib_kinematics/serialarm_calib.hpp"
#include "arm_calib_kinematics/ujnt_calib.hpp"
#include "arm_calib_kinematics/xyz_calib.hpp"
#include "robnux_kinematics_map/xyz_ur.hpp"
namespace kinematics_lib {
// separate-mode XYZ + UR
class KINEMATICS_API XyzUrCalib : public XYZ_UR,
                                  public BaseCalibratoin {
 public:
  XyzUrCalib();


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
                         EigenDRef<Eigen::VectorXd>* tool_offset) override;

  int CalibBaseFrame(const EigenDRef<Eigen::MatrixXd>& jnt_measures,
                     const Eigen::VectorXd& mes_tool,
                     EigenDRef<Eigen::VectorXd>* orig_base,
                     EigenDRef<Eigen::VectorXd>* comp_base) override;

  int CpsCartPose(const refPose& p, const Eigen::VectorXd& canonicalBase,
                  refPose* cp) = 0;

  int CpsJnt(const refPose& p, Eigen::VectorXd* cq) override;

  int CpsRobPath(const Eigen::VectorXd& calibBase,
                 const Eigen::VectorXd& origBase, const Eigen::VectorXd& tool,
                 const EigenDRef<Eigen::MatrixXd>& d_traj,
                 EigenDRef<Eigen::MatrixXd>* md_traj,
                 EigenDRef<Eigen::MatrixXd>* d_j_traj,
                 EigenDRef<Eigen::MatrixXd>* md_j_traj,
                 EigenDRef<Eigen::MatrixXd>* a_traj) override;

  bool GetCalibParamSet(EigenDRef<Eigen::VectorXd>* cal_DH) override;

  bool LoadCalibParamSet(const EigenDRef<Eigen::VectorXd>& cal_DH) override;

  bool PickSubJacobianForPara(const Eigen::MatrixXd& Jt_p,
                              const Eigen::MatrixXd& Jp_r,
                              Eigen::MatrixXd* Js_t1, Eigen::MatrixXd* Js_r1,
                              const bool reduction = false) override;

  std::string GetName() const { return std::string("XYZUR calib"); }

 private:
  //! define two robots
  std::shared_ptr<SerialArmCalib> ur_calib, xyz_calib;
};

}  // namespace kinematics_lib

#endif
