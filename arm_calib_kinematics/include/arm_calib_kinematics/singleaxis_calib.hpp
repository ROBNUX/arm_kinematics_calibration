#ifndef KINEMATICS_LIB_SINGLEAXIS_CALIB_HPP_
#define KINEMATICS_LIB_SINGLEAXIS_CALIB_HPP_
#include "arm_calib_kinematics/serialarm_calib.hpp"
namespace kinematics_lib {
   
class KINEMATICS_API SingleAxisCalib : public SerialArmCalib {
 public:
  SingleAxisCalib();
  SingleAxisCalib(const Eigen::VectorXd& kine_para);

  int CartToJnt(const Pose& p, Eigen::VectorXd* q) override; 

  void  UpdateConfigTurn(const Eigen::VectorXd & theta,
                         const Eigen::VectorXd &d,
                         std::vector<int>  *branchFlags,
                         std::vector<int>  *jointTurns) const override;

  bool PickSubJacobian(const Eigen::MatrixXd& Jp_t,
                       const Eigen::MatrixXd& Jp_r,
                       Eigen::MatrixXd* Js_t,
                       Eigen::MatrixXd* Js_r,
                       const bool reduction = false) override;

  bool PickSubJacobianForPara(const Eigen::MatrixXd& Jp_t,
                              const Eigen::MatrixXd& Jp_r, 
                              Eigen::MatrixXd* Js_t1, 
                              Eigen::MatrixXd* Js_r1,
                              const bool reduction = false) override; 

  double  PickCartErr(const Eigen::Vector3d& errT,
                      const Eigen::Vector3d& errR, 
                      Eigen::VectorXd* b,
                      const bool reduction = false) override;

  void UpdateDH(const Eigen::VectorXd& orig_dh,
                const Eigen::VectorXd& jnt,
                Eigen::VectorXd* new_dh) const override;

  void UpdateDH(const Eigen::VectorXd& jnt,
                Eigen::VectorXd* theta,
                Eigen::VectorXd* d
                ) const override;


   virtual std::string GetName() const {
       return std::string("SingleAxisCalib");
   }
};

}

#endif
