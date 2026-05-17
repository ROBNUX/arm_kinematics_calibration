#include "arm_calib_kinematics/xyzur_calib.hpp"

// register plugin
PLUGINLIB_EXPORT_CLASS(kinematics_lib::XyzUrCalib,
                       kinematics_lib::BaseKinematicMap)
PLUGINLIB_EXPORT_CLASS(kinematics_lib::XyzUrCalib,
                       kinematics_lib::BaseCalibration)

namespace kinematics_lib {

XyzUrCalib::XyzUrCalib() : XYZ_UR() {
  ur_calib = std::make_shared<UjntCalib>();
  xyz_calib = std::make_shared<XyzGantryCalib>();
}

XyzUrCalib::XyzUrCalib(const Eigen::VectorXd& dh_UR,
                       const Eigen::VectorXd& dh_XYZ)
    : XYZ_UR() {
  ur_calib = std::make_shared<UjntCalib>(dh_UR);
  xyz_calib = std::make_shared<XyzGantryCalib>(dh_XYZ);
  initialized_ = true;
}

double XyzUrCalib::LaserDistanceCalib(
    const Eigen::VectorXd& base_offset, const Eigen::VectorXd& tool_offset,
    const EigenDRef<Eigen::Matrix3d>& laser2CartMap,
    const EigenDRef<Eigen::MatrixXd>& cart_measure,
    const EigenDRef<Eigen::MatrixXd>& qa_array,
    const EigenDRef<Eigen::MatrixXd>& laser_measure) {
  std::ostringstream strs;
  if (!initialized_) {
    strs.str("");
    strs << GetName() << " geometric parameters are not initialized"
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_PARAM_NOT_INITIALIZED;
  }

  Eigen::MatrixXd cart_m = cart_measure;
  size_t numRow = cart_m.rows();
  size_t numCol = cart_m.cols();
  if (numRow > 3) {
    // for XYZ robot, set Euler angles and turns/branch flags as 0
    cart_m.block(3, 0, numRow - 3, numCol) =
        Eigen::MatrixXd::Zero(numRow - 3, numCol);
  }
  double ret = xyz_calib->LaserDistanceCalib(
      base_offset, tool_offset, laser2CartMap, cart_m, qa_array, laser_measure);
  if (ret >= 0 && ur_calib->isDHCalibrated()) {
    isDHCalibrated_ = true;
  }
  return ret;
}

Eigen::VectorXd XyzUrCalib::VerifyLaserDistanceCalib(
    const Eigen::VectorXd& base_offset, const Eigen::VectorXd& tool_offset,
    const EigenDRef<Eigen::Matrix3d>& laser2CartMat,
    const EigenDRef<Eigen::MatrixXd>& cart_measure,
    const EigenDRef<Eigen::MatrixXd>& qa_array,
    const EigenDRef<Eigen::MatrixXd>& laser_measure) {
  Eigen::MatrixXd cart_m = cart_measure;
  size_t numRow = cart_m.rows();
  size_t numCol = cart_m.cols();
  if (numRow > 3) {
    cart_m.block(3, 0, numRow - 3, numCol) =
        Eigen::MatrixXd::Zero(numRow - 3, numCol);
  }
  return xyz_calib->VerifyLaserDistanceCalib(
      base_offset, tool_offset, laser2CartMat, cart_m, qa_array, laser_measure);
}

double XyzUrCalib::DirectMesCalib(
    const Eigen::VectorXd& base_offset, const Eigen::VectorXd& tool_offset,
    const EigenDRef<Eigen::MatrixXd>& cart_measure,
    const EigenDRef<Eigen::MatrixXd>& measureMents,
    const EigenDRef<Eigen::MatrixXd>& qa_array) {
  std::ostringstream strs;
  if (!initialized_) {
    strs.str("");
    strs << GetName() << " geometric parameters are not initialized"
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_PARAM_NOT_INITIALIZED;
  }
  double ret = ur_calib->DirectMesCalib(base_offset, tool_offset, cart_measure,
                                        measureMents, qa_array);
  if (ret >= 0 && xyz_calib->isDHCalibrated()) {
    isDHCalibrated_ = true;
  }
  return ret;
}

Eigen::VectorXd XyzUrCalib::VerifyDirectMesCalib(
    const Eigen::VectorXd& base_offset, const Eigen::VectorXd& tool_offset,
    const EigenDRef<Eigen::MatrixXd>& cart_measure,
    const EigenDRef<Eigen::MatrixXd>& measureMents,
    const EigenDRef<Eigen::MatrixXd>& qa_array) {
  return ur_calib->VerifyDirectMesCalib(base_offset, tool_offset, cart_measure,
                                        measureMents, qa_array);
}

int XyzUrCalib::CalibTCPDistMethod(
    const Eigen::VectorXd& base_offset,
    const EigenDRef<Eigen::MatrixXd>& qa_array,
    const EigenDRef<Eigen::VectorXd>& measureMents,
    const EigenDRef<Eigen::Vector3d>& mes_normal,
    EigenDRef<Eigen::VectorXd>& tool_offset) {
  return xyz_calib->CalibTCPDistMethod(base_offset, qa_array, measureMents,
                                       mes_normal, tool_offset);
}

int XyzUrCalib::CalibBaseFrame(
    const EigenDRef<Eigen::MatrixXd>& jnt_measures,
    const Eigen::VectorXd& mes_tool,
    EigenDRef<Eigen::VectorXd>& orig_base,
    EigenDRef<Eigen::VectorXd>& comp_base) {
  std::ostringstream strs;
  if (!initialized_) {
    strs.str("");
    strs << GetName() << " geometric parameters are not initialized"
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_PARAM_NOT_INITIALIZED;
  }
  size_t numJnts = jnt_measures.cols();
  if (jnt_measures.rows() < DoF_ || numJnts < 1) {
    strs.str("");
    strs << GetName() << ":"
         << "The input matrix has wrong dimension in function " << __FUNCTION__
         << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_INPUT_PARA_WRONG_DIM;
  }
  // delegate base-frame calibration to the XYZ sub-robot using the joint
  // measurements that drive the XYZ axes (first DoF rows)
  Eigen::MatrixXd jnts_XYZ =
      jnt_measures.block(0, 0, xyz_calib->GetDoF(), numJnts);
  Eigen::Ref<Eigen::MatrixXd> jnts_XYZ_ref(jnts_XYZ);
  EigenDRef<Eigen::MatrixXd> jnts_XYZ_dref(jnts_XYZ_ref);
  return xyz_calib->CalibBaseFrame(jnts_XYZ_dref, mes_tool, orig_base,
                                   comp_base);
}

int XyzUrCalib::CpsCartPose(const refPose& /*p*/,
                            const Eigen::VectorXd& /*canonicalBase*/,
                            refPose& /*cp*/) {
  return -1;
}

int XyzUrCalib::CpsJnt(const refPose& /*p*/, Eigen::VectorXd& /*cq*/) {
  return -1;
}

int XyzUrCalib::CpsRobPath(
    const Eigen::VectorXd& /*calibBase*/, const Eigen::VectorXd& /*origBase*/,
    const Eigen::VectorXd& /*tool*/,
    const EigenDRef<Eigen::MatrixXd>& /*d_traj*/,
    EigenDRef<Eigen::MatrixXd>& /*md_traj*/,
    EigenDRef<Eigen::MatrixXd>& /*d_j_traj*/,
    EigenDRef<Eigen::MatrixXd>& /*md_j_traj*/,
    EigenDRef<Eigen::MatrixXd>& /*a_traj*/) {
  std::ostringstream strs;
  strs.str("");
  strs << GetName() << ": CpsRobPath not yet implemented for XYZ-UR, in "
       << __FUNCTION__ << ", line " << __LINE__ << std::endl;
  LOG_ALARM(strs);
  return -1;
}

bool XyzUrCalib::GetCalibParamSet(EigenDRef<Eigen::VectorXd>& cal_DH) {
  std::ostringstream strs;
  if (!initialized_) {
    strs.str("");
    strs << GetName() << " geometric parameters are not initialized"
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  size_t dof_XYZ = xyz_calib->GetDoF();
  size_t dof_UR = ur_calib->GetDoF();
  Eigen::VectorXd cal_DH_XYZ(4 * dof_XYZ + 7);
  Eigen::VectorXd cal_DH_UR(4 * dof_UR + 7);
  Eigen::Ref<Eigen::VectorXd> ref_XYZ(cal_DH_XYZ);
  Eigen::Ref<Eigen::VectorXd> ref_UR(cal_DH_UR);
  EigenDRef<Eigen::VectorXd> dref_XYZ(ref_XYZ);
  EigenDRef<Eigen::VectorXd> dref_UR(ref_UR);
  if (!xyz_calib->GetCalibParamSet(dref_XYZ)) {
    return false;
  }
  if (!ur_calib->GetCalibParamSet(dref_UR)) {
    return false;
  }
  if (cal_DH.size() < static_cast<Eigen::Index>(4 * DoF_ + 14)) {
    strs.str("");
    strs << GetName() << ":"
         << "The output vector has wrong dimension in function " << __FUNCTION__
         << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  // Interleave the XYZ and UR DH parameters into the combined vector:
  // [alpha (DoF_), a (DoF_), theta (DoF_), d (DoF_), baseOff_XYZ (7), baseOff_UR (7)]
  for (size_t i = 0; i < dof_XYZ; ++i) {
    cal_DH(i) = cal_DH_XYZ(i);
    cal_DH(DoF_ + i) = cal_DH_XYZ(dof_XYZ + i);
    cal_DH(2 * DoF_ + i) = cal_DH_XYZ(2 * dof_XYZ + i);
    cal_DH(3 * DoF_ + i) = cal_DH_XYZ(3 * dof_XYZ + i);
  }
  for (size_t i = 0; i < dof_UR; ++i) {
    cal_DH(dof_XYZ + i) = cal_DH_UR(i);
    cal_DH(DoF_ + dof_XYZ + i) = cal_DH_UR(dof_UR + i);
    cal_DH(2 * DoF_ + dof_XYZ + i) = cal_DH_UR(2 * dof_UR + i);
    cal_DH(3 * DoF_ + dof_XYZ + i) = cal_DH_UR(3 * dof_UR + i);
  }
  cal_DH.segment(4 * DoF_, 7) = cal_DH_XYZ.segment(4 * dof_XYZ, 7);
  cal_DH.segment(4 * DoF_ + 7, 7) = cal_DH_UR.segment(4 * dof_UR, 7);
  return true;
}

bool XyzUrCalib::LoadCalibParamSet(const EigenDRef<Eigen::VectorXd>& cal_DH) {
  std::ostringstream strs;
  if (!initialized_) {
    strs.str("");
    strs << GetName() << " geometric parameters are not initialized"
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  if (cal_DH.size() < static_cast<Eigen::Index>(4 * DoF_ + 14)) {
    strs.str("");
    strs << GetName() << ":"
         << "The input vector has wrong dimension in function " << __FUNCTION__
         << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  size_t dof_XYZ = xyz_calib->GetDoF();
  size_t dof_UR = ur_calib->GetDoF();
  Eigen::VectorXd dh_XYZ(4 * dof_XYZ + 7);
  Eigen::VectorXd dh_UR(4 * dof_UR + 7);
  for (size_t i = 0; i < dof_XYZ; ++i) {
    dh_XYZ(i) = cal_DH(i);
    dh_XYZ(dof_XYZ + i) = cal_DH(DoF_ + i);
    dh_XYZ(2 * dof_XYZ + i) = cal_DH(2 * DoF_ + i);
    dh_XYZ(3 * dof_XYZ + i) = cal_DH(3 * DoF_ + i);
  }
  for (size_t i = 0; i < dof_UR; ++i) {
    dh_UR(i) = cal_DH(dof_XYZ + i);
    dh_UR(dof_UR + i) = cal_DH(DoF_ + dof_XYZ + i);
    dh_UR(2 * dof_UR + i) = cal_DH(2 * DoF_ + dof_XYZ + i);
    dh_UR(3 * dof_UR + i) = cal_DH(3 * DoF_ + dof_XYZ + i);
  }
  dh_XYZ.segment(4 * dof_XYZ, 7) = cal_DH.segment(4 * DoF_, 7);
  dh_UR.segment(4 * dof_UR, 7) = cal_DH.segment(4 * DoF_ + 7, 7);

  Eigen::Ref<Eigen::VectorXd> ref_XYZ(dh_XYZ);
  Eigen::Ref<Eigen::VectorXd> ref_UR(dh_UR);
  EigenDRef<Eigen::VectorXd> dref_XYZ(ref_XYZ);
  EigenDRef<Eigen::VectorXd> dref_UR(ref_UR);
  if (xyz_calib->LoadCalibParamSet(dref_XYZ) &&
      ur_calib->LoadCalibParamSet(dref_UR)) {
    isDHCalibrated_ = true;
    return true;
  }
  return false;
}

bool XyzUrCalib::PickSubJacobianForPara(const Eigen::MatrixXd& /*Jt_p*/,
                                        const Eigen::MatrixXd& /*Jp_r*/,
                                        Eigen::MatrixXd& /*Js_t1*/,
                                        Eigen::MatrixXd& /*Js_r1*/,
                                        const bool /*reduction*/) {
  std::ostringstream strs;
  strs.str("");
  strs << GetName() << ": PickSubJacobianForPara not implemented for "
                       "composite XYZ-UR; calibration runs through the two "
                       "sub-robots independently. In "
       << __FUNCTION__ << ", line " << __LINE__ << std::endl;
  LOG_ALARM(strs);
  return false;
}

}  // namespace kinematics_lib
