#include "arm_calib_kinematics/xyzur_calib.hpp"

// register plugin
PLUGINLIB_EXPORT_CLASS(kinematics_lib::XyzUrCalib,
                       kinematics_lib::BaseKinematicMap)
PLUGINLIB_EXPORT_CLASS(kinematics_lib::XyzUrCalib,
                       kinematics_lib::BaseCalibration)

namespace kinematics_lib {

XyzUrCalib::XyzUrCalib() : BaseKinematicMap(5, 5) {
  ur_calib = std::make_shared<UjntCalib>();
  xyz_calib = std::make_shared<XyzGantryCalib>();
}

XyzUrCalib::XyzUrCalib(const Eigen::VectorXd &dh_UR,
                       const Eigen::VectorXd &dh_XYZ)
    : BaseKinematicMap(5, 5) {
  ur_calib = std::make_shared<UjntCalib>(dh_UR);
  xyz_calib = std::make_shared<XyzGantryCalib>(dh_XYZ);
  initialized_ = true;
}

void XyzUrCalib::SetGeometry(const Eigen::VectorXd &kine_para) {
  std::ostringstream strs;
  if (kine_para.size() < 6 * DoF_) {
    strs.str("");
    strs << GetName() << ":"
         << "XYZ_UR set geometry got input parameters with wrong dimension"
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return;
  }
  if (!ur_calib || !xyz_calib) {
    strs.str("");
    strs << GetName() << ":"
         << "XYZ_UR set geometry fails because UR and XYZ are null"
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return;
  }
  Eigen::VectorXd alpha(DoF_), a(DoF_), theta(DoF_), d(DoF_), beta(DoF_),
      pitch(DoF_);

  alpha = kine_para.segment(0, DoF_);
  a = kine_para.segment(DoF_, DoF_);
  theta = kine_para.segment(2 * DoF_, DoF_);
  d = kine_para.segment(3 * DoF_, DoF_);
  beta = kine_para.segment(4 * DoF_, DoF_);
  pitch = kine_para.segment(5 * DoF_, DoF_);

  Eigen::VectorXd dh_XYZ(18), dh_UR(12);
  dh_XYZ.segment(0, 3) = alpha.segment(0, 3);
  dh_XYZ.segment(3, 3) = a.segment(0, 3);
  dh_XYZ.segment(6, 3) = theta.segment(0, 3);
  dh_XYZ.segment(9, 3) = d.segment(0, 3);
  dh_XYZ.segment(12, 3) = beta.segment(0, 3);
  dh_XYZ.segment(15, 3) = pitch.segment(0, 3);
  xyz_calib->SetGeometry(dh_XYZ);
  dh_UR.segment(0, 2) = alpha.segment(3, 2);
  dh_UR.segment(2, 2) = a.segment(3, 2);
  dh_UR.segment(4, 2) = theta.segment(3, 2);
  dh_UR.segment(6, 2) = d.segment(3, 2);
  dh_UR.segment(8, 2) = beta.segment(3, 2);
  dh_UR.segment(10, 2) = pitch.segment(3, 2);
  ur_calib->SetGeometry(dh_UR);
  initialized_ = true;
}

int XyzUrCalib::CartToJnt(const Pose &p, Eigen::VectorXd *q) {
  std::ostringstream strs;
  if (!q) {
    strs.str("");
    strs << GetName() << " input joint angle pointer is null in "
         << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_INPUT_POINTER_NULL;
  }
  if (!initialized_) {
    strs.str("");
    strs << GetName() << ":"
         << "XYZ_UR geometric parameters are not initialized"
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_PARAM_NOT_INITIALIZED;
  }
  std::vector<int> ikJointTurns;
  p.getJointTurns(&ikJointTurns);

  std::vector<int> ikJointTurn_UR;
  ikJointTurn_UR.insert(ikJointTurn_UR.end(),
                        ikJointTurns.begin() + xyz_calib->GetDoF(),
                        ikJointTurns.end());
  Pose p1 = p;
  p1.setJointTurns(ikJointTurn_UR);
  Eigen::VectorXd UR_jnt;
  int ret = ur_calib->CartToJnt(p1, &UR_jnt);
  if (ret < 0) {
    strs.str("");
    strs << GetName() << ":"
         << "ur_calib->CartToJnt fails in " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return ret;
  }

  Pose p2;  // default orientation
  p2.setTranslation(p.getTranslation());

  Eigen::VectorXd XYZ_jnt;
  ret = xyz_calib->CartToJnt(p2, &XYZ_jnt);
  if (ret < 0) {
    strs.str("");
    strs << GetName() << ":"
         << "xyz_calib->CartToJnt fails in " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return ret;
  }
  if (q->size() < DoF_) {
    q->resize(DoF_);
  }
  q->segment(0, xyz_calib->GetDoF()) = XYZ_jnt;
  q->segment(xyz_calib->GetDoF(), ur_calib->GetDoF()) = UR_jnt;
  return 0;
}

int XyzUrCalib::JntToCart(const Eigen::VectorXd &q, Pose *p) {
  std::ostringstream strs;
  if (!initialized_) {
    strs.str("");
    strs << GetName() << ":"
         << "XYZ_UR geometric parameters are not initialized"
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_PARAM_NOT_INITIALIZED;
  }
  if (!p) {
    strs.str("");
    strs << GetName() << " input pose pointer is null in " << __FUNCTION__
         << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_INPUT_POINTER_NULL;
  }
  if (q.size() != DoF_) {
    strs.str("");
    strs << GetName() << ":"
         << "Input q dimension does not match with the robot"
         << ", q size=" << q.size() << ", DoF_ =" << DoF_ << " in function "
         << __FUNCTION__ << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_INPUT_PARA_WRONG_DIM;
  }
  Pose pXYZ;
  Eigen::VectorXd q1 = q.segment(0, xyz_calib->GetDoF());
  int ret = xyz_calib->JntToCart(q1, &pXYZ);
  if (ret < 0) {
    return ret;
  }

  Pose pUR;
  Eigen::VectorXd q2 = q.segment(xyz_calib->GetDoF(), ur_calib->GetDoF());
  ret = ur_calib->JntToCart(q2, &pUR);
  if (ret < 0) {
    return ret;
  }
  // turns and flags
  std::vector<int> jntTurns1, jntTurns2, jntTurns;
  std::vector<int> ikBranchFlags;
  // jnt turns vector will be combo of XYZ jnt turns + UR jnt turns
  pXYZ.getJointTurns(&jntTurns1);
  pUR.getJointTurns(&jntTurns2);
  jntTurns.insert(jntTurns.end(), jntTurns1.begin(), jntTurns1.end());
  jntTurns.insert(jntTurns.end(), jntTurns2.begin(), jntTurns2.end());
  // BranchFlags will be only for UR robot
  pUR.getBranchFlags(&ikBranchFlags);

  // combine
  Frame tmp(pUR.getRotation(), pXYZ.getTranslation());
  p->setFrame(tmp);
  p->setBranchFlags(ikBranchFlags);
  p->setJointTurns(jntTurns);
  return 0;
}

int XyzUrCalib::JntToCart(const Eigen::VectorXd &q, const Eigen::VectorXd &qdot,
                          Pose *p, Twist *v) {
  std::ostringstream strs;
  if (!p || !v) {
    strs.str("");
    strs << GetName() << ":"
         << "input pose and twist parameter is null in function "
         << __FUNCTION__ << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_INPUT_POINTER_NULL;
  }
  if (!initialized_) {
    strs.str("");
    strs << GetName() << ":"
         << "XYZ_UR geometric parameters are not initialized"
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_PARAM_NOT_INITIALIZED;
  }
  if (q.size() != DoF_ || qdot.size() != DoF_) {
    strs.str("");
    strs << GetName() << ":"
         << "Input q and qdot dimension does not match with the robot"
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return -ERR_INPUT_PARA_WRONG_DIM;
  }
  Eigen::VectorXd q1 = q.segment(0, xyz_calib->GetDoF());
  Eigen::VectorXd q1dot = qdot.segment(0, xyz_calib->GetDoF());
  Eigen::VectorXd q2 = q.segment(xyz_calib->GetDoF(), ur_calib->GetDoF());
  Eigen::VectorXd q2dot = qdot.segment(xyz_calib->GetDoF(), ur_calib->GetDoF());

  Pose pXYZ, pUR;
  Twist vXYZ, vUR;
  int ret = xyz_calib->JntToCart(q1, q1dot, &pXYZ, &vXYZ);
  if (ret < 0) {
    return ret;
  }
  ret = ur_calib->JntToCart(q2, q2dot, &pUR, &vUR);
  if (ret < 0) {
    return ret;
  }
  Frame fr(pUR.getRotation(), pXYZ.getTranslation());
  p->setFrame(fr);

  // turns and flags
  std::vector<int> jntTurns1, jntTurns2, jntTurns;
  std::vector<int> ikBranchFlags;
  // jnt turns vector will be combo of XYZ jnt turns + UR jnt turns
  pXYZ.getJointTurns(&jntTurns1);
  pUR.getJointTurns(&jntTurns2);
  jntTurns.insert(jntTurns.end(), jntTurns1.begin(), jntTurns1.end());
  jntTurns.insert(jntTurns.end(), jntTurns2.begin(), jntTurns2.end());
  // BranchFlags will be only for UR robot
  pUR.getBranchFlags(&ikBranchFlags);

  p->setBranchFlags(ikBranchFlags);
  p->setJointTurns(jntTurns);

  Vec linear_vel = vXYZ.getLinearVel();
  Vec angular_vel = vUR.getAngularVel();
  v->setLinearVel(linear_vel);
  v->setAngularVel(angular_vel);
  return 0;
}

int XyzUrCalib::CartToJnt(const Pose &p, const Twist &v, Eigen::VectorXd *q,
                             Eigen::VectorXd *qdot) {
  std::ostringstream strs;
  if (!q || !qdot) {
    strs.str("");
    strs << GetName() << ":"
         << "Input q and qdot vectors are null"
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return -ERR_INPUT_POINTER_NULL;
  }
  if (!initialized_) {
    strs.str("");
    strs << GetName() << " geometric parameters are not initialized"
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_PARAM_NOT_INITIALIZED;
  }
  Eigen::VectorXd qXYZ, qUR, qXYZdot, qURdot;
  int ret = xyz_calib->CartToJnt(p, v, &qXYZ, &qXYZdot);
  if (ret < 0) {
    return ret;
  }
  ret = ur_calib->CartToJnt(p, v, &qUR, &qURdot);
  if (ret < 0) {
    return ret;
  }
  if (q->size() < DoF_) {
    q->resize(DoF_);
  }
  q->segment(0, xyz_calib->GetDoF()) = qXYZ;
  q->segment(xyz_calib->GetDoF(), ur_calib->GetDoF()) = qUR;
  if (qdot->size() < DoF_) {
    qdot->resize(DoF_);
  }
  qdot->segment(0, xyz_calib->GetDoF()) = qXYZdot;
  qdot->segment(xyz_calib->GetDoF(), ur_calib->GetDoF()) = qURdot;
  return 0;
}

int XyzUrCalib::CalcJacobian(const Eigen::VectorXd &kine_para, Pose *p,
                             Eigen::MatrixXd *Jp_t, Eigen::MatrixXd *Jp_r,
                             const bool reduction) {
  std::ostringstream strs;
  // Jp is 6 * kine_para_.size() matrix
  if (!p || !Jp_t || !Jp_r) {
    strs.str("");
    strs << GetName() << ":"
         << "input pose, Jp, Jj pointers are null in " << __FUNCTION__
         << " at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_INPUT_POINTER_NULL;
  }
  if (!initialized_) {
    strs.str("");
    strs << GetName() << " geometric parameters are not initialized"
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_PARAM_NOT_INITIALIZED;
  }
  if (kine_para.size() != 5 * DoF_) {
    strs.str("");
    strs << GetName() << ":"
         << "input kine_parameters has dimension not equal to 5 * DoF in "
         << __FUNCTION__ << " line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_INPUT_PARA_WRONG_DIM;
  }

  Eigen::VectorXd alpha(DoF_), a(DoF_), theta(DoF_), d(DoF_), beta(DoF_);
  alpha = kine_para.segment(0, DoF_);
  a = kine_para.segment(DoF_, DoF_);
  theta = kine_para.segment(2 * DoF_, DoF_);
  d = kine_para.segment(3 * DoF_, DoF_);
  beta = kine_para.segment(4 * DoF_, DoF_);

  Eigen::VectorXd dh_XYZ(xyz_calib->GetDoF() * 5),
      dh_UR(ur_calib->GetDoF() * 5);
  dh_XYZ.segment(0, 3) = alpha.segment(0, 3);
  dh_XYZ.segment(3, 3) = a.segment(0, 3);
  dh_XYZ.segment(6, 3) = theta.segment(0, 3);
  dh_XYZ.segment(9, 3) = d.segment(0, 3);
  dh_XYZ.segment(12, 3) = beta.segment(0, 3);

  dh_UR.segment(0, 2) = alpha.segment(3, 2);
  dh_UR.segment(2, 2) = a.segment(3, 2);
  dh_UR.segment(4, 2) = theta.segment(3, 2);
  dh_UR.segment(6, 2) = d.segment(3, 2);
  dh_UR.segment(8, 2) = beta.segment(3, 2);

  Pose pXYZ, pUR;
  Eigen::MatrixXd Jp_t_XYZ, Jp_r_XYZ;
  Eigen::MatrixXd Jp_t_UR, Jp_r_UR;
  int ret =
      xyz_calib->CalcJacobian(dh_XYZ, &pXYZ, &Jp_t_XYZ, &Jp_r_XYZ, reduction);
  if (ret < 0) {
    return ret;
  }

  ret = ur_calib->CalcJacobian(dh_UR, &pUR, &Jp_t_UR, &Jp_r_UR, reduction);
  if (ret < 0) {
    return ret;
  }
  Frame fr(pUR.getRotation(), pXYZ.getTranslation());
  p->setFrame(fr);

  // turns and flags
  std::vector<int> jntTurns1, jntTurns2, jntTurns;
  std::vector<int> ikBranchFlags;
  // jnt turns vector will be combo of XYZ jnt turns + UR jnt turns
  pXYZ.getJointTurns(&jntTurns1);
  pUR.getJointTurns(&jntTurns2);
  jntTurns.insert(jntTurns.end(), jntTurns1.begin(), jntTurns1.end());
  jntTurns.insert(jntTurns.end(), jntTurns2.begin(), jntTurns2.end());
  // BranchFlags will be only for UR robot
  pUR.getBranchFlags(&ikBranchFlags);
  p->setBranchFlags(ikBranchFlags);
  p->setJointTurns(jntTurns);
  *Jp_t = Jp_t_XYZ;
  *Jp_r = Jp_r_UR;
  return 0;
}

double XyzUrCalib::CalibrateLaserCoplanar(
    const EigenDRef<Eigen::MatrixXd>
        &cart_measure_x,  // cartesian coordinates reported from robot
    const EigenDRef<Eigen::MatrixXd> &cart_measure_y,
    const EigenDRef<Eigen::MatrixXd> &cart_measure_z,
    const EigenDRef<Eigen::MatrixXd> &laserMat_x,
    const EigenDRef<Eigen::MatrixXd> &laserMat_y,
    const EigenDRef<Eigen::MatrixXd> &laserMat_z,
    const EigenDRef<Eigen::Vector3d> &laser_scale) {
  std::ostringstream strs;
  if (!initialized_) {
    strs.str("");
    strs << GetName() << " geometric parameters are not initialized"
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_PARAM_NOT_INITIALIZED;
  }

  Eigen::MatrixXd cart_m_x = cart_measure_x;
  Eigen::MatrixXd cart_m_y = cart_measure_y;
  Eigen::MatrixXd cart_m_z = cart_measure_z;
  size_t numRow = cart_m_x.rows();
  size_t numCol = cart_m_x.cols();
  if (numRow > 3) {
    cart_m_x.block(3, 0, numRow - 3, numCol) = Eigen::MatrixXd::Zero(
        numRow - 3, numCol);  // for XYZ robot, needs to set Euler angles and
                              // turns, branch as 0
  }
  numRow = cart_m_y.rows();
  numCol = cart_m_y.cols();
  if (numRow > 3) {
    cart_m_y.block(3, 0, numRow - 3, numCol) = Eigen::MatrixXd::Zero(
        numRow - 3, numCol);  // for XYZ robot, needs to set Euler angles and
                              // turns, branch as 0
  }
  numRow = cart_m_z.rows();
  numCol = cart_m_z.cols();
  if (numRow > 3) {
    cart_m_z.block(3, 0, numRow - 3, numCol) = Eigen::MatrixXd::Zero(
        numRow - 3, numCol);  // for XYZ robot, needs to set Euler angles and
                              // turns, branch as 0
  }

  double ret =
      xyz_calib->CalibrateLaserCoplanar(cart_m_x, cart_m_y, cart_m_z, laserMat_x,
                                  laserMat_y, laserMat_z, laser_scale);
  if (ret >= 0) {  // if XYZ calibration sucess
    if (ur_calib->isCalibrated()) {
      isDHCalibrated_ = true;  // set XYZ-UR calibration success
    }
  }
  return ret;
}

double XyzUrCalib::LaserDistanceCalib(
    const Eigen::VectorXd &base_offset, const Eigen::VectorXd &tool_offset,
    const EigenDRef<Eigen::Matrix3d> &laser2CartMap,
    const EigenDRef<Eigen::MatrixXd> &cart_measure,
    const EigenDRef<Eigen::MatrixXd> &qa_array,
    const EigenDRef<Eigen::MatrixXd> &laser_measure) {
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
    cart_m.block(3, 0, numRow - 3, numCol) = Eigen::MatrixXd::Zero(
        numRow - 3, numCol);  // for XYZ robot, needs to set Euler angles and
                              // turns, branch as 0
  }
  double ret = xyz_calib->LaserDistanceCalib(
      base_offset, tool_offset, laser2CartMap, cart_m, qa_array, laser_measure);
  if (ret >= 0) {  // if UR calibration success
    if (ur_calib->isCalibrated()) {
      isDHCalibrated_ = true;  // set XYZ-UR calibration success
    }
  }
  return ret;
}

Eigen::VectorXd XyzUrCalib::VerifyLaserDistanceCalib(
    const Eigen::VectorXd &base_offset, const Eigen::VectorXd &tool_offset,
    const EigenDRef<Eigen::Matrix3d> &laser2CartMat,
    const EigenDRef<Eigen::MatrixXd> &cart_measure,
    const EigenDRef<Eigen::MatrixXd> &qa_array,
    const EigenDRef<Eigen::MatrixXd> &laser_measure) {
  Eigen::MatrixXd cart_m = cart_measure;
  size_t numRow = cart_m.rows();
  size_t numCol = cart_m.cols();
  if (numRow > 3) {
    cart_m.block(3, 0, numRow - 3, numCol) = Eigen::MatrixXd::Zero(
        numRow - 3, numCol);  // for XYZ robot, needs to set Euler angles and
                              // turns, branch as 0
  }
  return xyz_calib->VerifyLaserDistanceCalib(
      base_offset, tool_offset, laser2CartMap, cart_m, qa_array, laser_measure);
}

double XyzUrCalib::DirectMesCalib(
    const Eigen::VectorXd &base_offset, const Eigen::VectorXd &tool_offset,
    const EigenDRef<Eigen::MatrixXd> &cart_measure,
    const EigenDRef<Eigen::MatrixXd> &measureMents,
    const EigenDRef<Eigen::MatrixXd> &qa_array) {
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
  if (ret >= 0) {  // if XYZ calibration success
    if (xyz_calib->isCalibrated()) {
      isDHCalibrated_ = true;  // set XYZ-UR calibration success
    }
  }
  return ret;
}

Eigen::VectorXd XyzUrCalib::VerifyDirectMesCalib(
    const Eigen::VectorXd &base_offset, const Eigen::VectorXd &tool_offset,
    const EigenDRef<Eigen::MatrixXd> &cart_measure,
    const EigenDRef<Eigen::MatrixXd> &measureMents,
    const EigenDRef<Eigen::MatrixXd> &qa_array) {
  return ur_calib->VerifyDirectMesCalib(base_offset, tool_offset, cart_measure,
                                        measureMents, qa_array);
}

int XyzUrCalib::CalibTCPDistMethod(
    const Eigen::VectorXd &base_offset,
    const EigenDRef<Eigen::MatrixXd> &qa_array,
    const EigenDRef<Eigen::VectorXd> &measureMents,
    const EigenDRef<Eigen::Vector3d> &mes_normal,
    EigenDRef<Eigen::VectorXd> *tool_offset) {
  return -1;
}

int XyzUrCalib::CalibBaseFrame(const EigenDRef<Eigen::MatrixXd> &jnt_measures,
                               const Eigen::VectorXd &mes_tool,
                               EigenDRef<Eigen::VectorXd> *orig_base,
                               EigenDRef<Eigen::VectorXd> *comp_base) {
  std::ostringstream strs;
  if (!initialized_) {
    strs.str("");
    strs << GetName() << " geometric parameters are not initialized"
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_PARAM_NOT_INITIALIZED;
  }
  size_t numJnts = jnt_base_measures.cols();
  // here orig_base is the workpiece frame, should be reachable by IK
  if (jnt_base_measures.rows() < DoF_ || numJnts < 1) {
    strs.str("");
    strs << GetName() << ":"
         << "The input vector has wrong dimension in function " << __FUNCTION__
         << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_INPUT_POINTER_NULL;
  }
  if (!tcp_UR_uncal || !tcp_UR_cal) {
    strs.str("");
    strs << GetName() << ":"
         << "input comp_base is null in function " << __FUNCTION__
         << " at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_INPUT_POINTER_NULL;
  }

  Eigen::MatrixXd jnts_UR = jnt_base_measures.block(
      xyz_calib->GetDoF(), 0, ur_calib->GetDoF(), numJnts);
  Eigen::MatrixXd jnts_XYZ =
      jnt_base_measures.block(0, 0, xyz_calib->GetDoF(), numJnts);

  // using XYZ 8pt method to compute the base
  // Eigen::MatrixXd  origCart(6, 8), compCart(6, 8);
  Eigen::VectorXd comp_base(7), comp_base_uncal(7);
  int ret = xyz_calib->CalibBaseFrame(jnts_XYZ, orig_tool, &comp_base_uncal,
                                      &comp_base);
  if (ret < 0) {
    return ret;
  }

  // note tcp_UR and tcp_XYZ will be overrided by UR workobj calib
  (*tcp_UR_uncal).setZero();
  (*tcp_UR_cal).setZero();

  // first check if all jnts are same
  for (size_t j = 1; j < numJnts; j++) {
    Eigen::VectorXd diff_jnt_UR = jnts_UR.col(j) - jnts_UR.col(0);
    if (diff_jnt_UR.norm() > K_EPSILON) {
      strs.str("");
      strs << GetName() << ":"
           << "UR joints in each plane is not same"
           << " , diff_jnt_UR=" << diff_jnt_UR << ", in function "
           << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -2001;
    }
  }
  // second, compute the FK of UR
  SetUsingCalibratedModel(false);
  Eigen::VectorXd jnt = jnts_UR.col(0);
  Pose ps;
  ret = ur_calib->JntToCart(jnt, &ps);
  if (ret < 0) {
    strs.str("");
    strs << GetName() << ":"
         << " FK error, code  " << ret << "can not do error compensation in"
         << __FUNCTION__ << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return ret;
  }
  Frame flange;
  ps.getFrame(&flange);
  Eigen::VectorXd v_uncal = comp_base_uncal.segment(0, 3);
  Eigen::VectorXd q_uncal = comp_base_uncal.segment(3, 4);
  Vec v_u(v_uncal);
  Quaternion q_u = Quaternion::FromEigenVec(q_uncal);
  Frame block_u(q_u, v_u);
  Frame rel_block_u = flange.Inverse() * block_u;

  strs.str("");
  strs << GetName() << ": comp_base_uncal=" << block_u.ToString(true)
       << std::endl;
  strs << " jnts_UR=" << jnts_UR << ", uncal_flage=" << flange.ToString(false)
       << ", in euler=" << flange.ToString(true) << std::endl;
  strs << "rel_block_u=" << rel_block_u.ToString(true) << std::endl;
  LOG_INFO(strs);
  tcp_UR_uncal->segment(0, 3) = rel_block_u.getTranslation().ToEigenVec();
  tcp_UR_uncal->segment(3, 4) = rel_block_u.getQuaternion().ToEigenVec();

  SetUsingCalibratedModel(true);
  ret = ur_calib->JntToCart(jnt, &ps);
  if (ret < 0) {
    strs.str("");
    strs << GetName() << ":"
         << " (calibrated) FK error, code  " << ret
         << "can not do error compensation in" << __FUNCTION__ << ", line "
         << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return ret;
  }
  ps.getFrame(&flange);
  Eigen::VectorXd v_cal = comp_base.segment(0, 3);
  Eigen::VectorXd q_cal = comp_base.segment(3, 4);
  Vec v_c(v_cal);
  Quaternion q_c = Quaternion::FromEigenVec(q_cal);
  Frame block_c(q_c, v_c);
  Frame rel_block_c = flange.Inverse() * block_c;

  strs.str("");
  strs << GetName() << ":comp_base=" << block_c.ToString(true) << std::endl;
  strs << ", jnts_UR=" << jnts_UR << ", cal_flage=" << flange.ToString(false)
       << ", in euler=" << flange.ToString(true) << std::endl;
  strs << "rel_block_c=" << rel_block_c.ToString(true) << std::endl;
  LOG_INFO(strs);

  tcp_UR_cal->segment(0, 3) = rel_block_c.getTranslation().ToEigenVec();
  tcp_UR_cal->segment(3, 4) = rel_block_c.getQuaternion().ToEigenVec();
  return 0;
}

int XyzUrCalib::CpsCartPose(const refPose &p,
                            const Eigen::VectorXd &canonicalBase, refPose *cp) {
  return -1;
}

int XyzUrCalib::CpsJnt(const refPose &p, Eigen::VectorXd *cq) { return -1; }

int XyzUrCalib::CpsRobPath(
    const Eigen::VectorXd &calibBase, const Eigen::VectorXd &origBase,
    const Eigen::VectorXd &tool, const EigenDRef<Eigen::MatrixXd> &d_traj,
    EigenDRef<Eigen::MatrixXd> *md_traj, EigenDRef<Eigen::MatrixXd> *d_j_traj,
    EigenDRef<Eigen::MatrixXd> *md_j_traj, EigenDRef<Eigen::MatrixXd> *a_traj) {
  std::ostringstream strs;
  if (!md_traj || !d_j_traj || !md_j_traj || !a_traj) {
    strs.str("");
    strs << GetName() << ":"
         << "Input pointer is null"
         << " so can not do compensation, in function " << __FUNCTION__
         << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_INPUT_POINTER_NULL;
  }
  if (!isDHCalibrated_) {
    strs.str("");
    strs << GetName() << ":"
         << " robot is not calibrated, "
         << "can not do error compensation in" << __FUNCTION__ << ", line "
         << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_COMP_WITHOUT_CALIB;
  }
  // generate base and tool frame from the input data
  if (calibBase.rows() != 7 || origBase.rows() != 7 || tool.rows() != 7) {
    strs.str("");
    strs << GetName() << ":"
         << "Input Tcp/tool data is not translation + quaternion"
         << " so can not do calibration, in function " << __FUNCTION__
         << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_INPUT_PARA_WRONG_DIM;
  }
  strs.str("");
  strs << GetName() << ":"
       << "In traj. compensation: calib UR Tcp= " << calibTCP
       << ", uncalibrated UR tcp= " << origTCP << ", desired tool=" << bestTool
       << std::endl;
  LOG_INFO(strs);

  size_t numPts = d_traj.cols();
  size_t numRows = d_traj.rows();
  // resize output variables
  md_traj->resize(9, numPts);
  d_j_traj->resize(DoF_, numPts);
  md_j_traj->resize(DoF_, numPts);
  a_traj->resize(9, numPts);

  // XYZ tool
  Frame newXYZ_Tool;
  Vec t(tool.segment(0, 3));
  newXYZ_Tool.setTranslation(t);
  Quaternion q = Quaternion::FromEigenVec(tool.segment(3, 4));
  newXYZ_Tool.setQuaternion(q);

  // calibrated TCP of block
  Vec bv(calibBase.segment(0, 3));
  Quaternion bq = Quaternion::FromEigenVec(calibBase.segment(3, 4));
  Frame newBaseTCP(bq, bv);

  // original TCP of block
  Vec bv_old(origBaes.segment(0, 3));
  Quaternion bq_old = Quaternion::FromEigenVec((origBase.segment(3, 4));
  Frame oldBaseTCP(bq_old, bv_old);

  // At every pt in desired traj, we need to  compute ref UR pose in calibrated
  // model, and in orig model
  refPose rf_pose_UR_calib, rf_pose_UR_orig;

  // default pose in orig and calib
  Pose df_pose_UR_orig, df_pose_UR_calib;

  // At every pt in desired traj, what is calibrated (accurate) UR joint and
  // orig (or errored/not accurate) UR joint
  Eigen::VectorXd qUR_cal(ur_calib->GetDoF(), 0),
                  qUR_orig(ur_calib->GetDoF(), 0);
  // next we need to compute, at very pt, the calib / orig base of block w.r.t.
  // XYZ base using pose_UR_calib and pose_UR_orig and calibTCP /origTCP
  Frame act_calibBase, act_origBase;

  int ret = 0;
  bool URJnt = d_traj.rows() <= 5 ? true : false;
  for (size_t i = 0; i < numPts; i++) {
    Eigen::VectorXd cur_pt = d_traj.col(i);
    strs.str("");
    strs << "cur_pt = " << cur_pt.transpose() << std::endl;
    LOG_INFO(strs);
    if (URJnt) {  // if desired traj contains desired UR joint traj
      // note: here act_UR is already the modified UR traj
      qUR_cal = cur_pt.segment(xyz_calib->GetDoF(), ur_calib->GetDoF());
      qUR_orig = qUR_cal;  // if desired traj was given as (x,y,z,Ud,Rd), then
                           // qUR_cal = qUR_orig={Ud, Rd}
    } else {  // input traj contains UR desired orientation info w.r.t. current
              // subTCP
      Quaternion qtmp(cur_pt(3), cur_pt(4), cur_pt(5), cur_pt(6));
      Vec v(0, 0, 0);
      size_t inFlag = cur_pt(7);
      // for scara, there is only 1 branch flag: elbow (up or down), and for
      // six-axis robot, there are 3 flags
      std::vector<int> branchFlags;
      ConvertBranchFlag(inFlag, &branchFlags);
      // convert joint turn data into ikJointTurns
      size_t tFlag = cur_pt(8);
      std::vector<int> ikJointTurns(DoF_, 0);
      ConvertMultiTurnFlag(tFlag, &ikJointTurns);
      std::vector<int> ikTurns_UR(ur_calib->GetDoF(), 0);
      for (size_t j = 0; j < ur_calib->GetDoF(); j++) {
        ikTurns_UR[j] = ikJointTurns[j + xyz_calib->GetDoF()];
      }

      Frame tmp_UR_fm(qtmp, v);  // initially set v as 0, and just use ideal
                                 // orientation to compute qUR_orig and qUR_cal
      rf_pose_UR_orig.setFrame(tmp_UR_fm);
      rf_pose_UR_orig.setBranchFlags(branchFlags);
      rf_pose_UR_orig.setJointTurns(ikTurns_UR);
      rf_pose_UR_orig.setTool(
          newBaseTCP);  // here we have use newBaseTCP, but not oldBaseTCP,
                        // because otherwise there will be more err.

      // default pose in default base and tool
      rf_pose_UR_orig.getDefaultPose(&df_pose_UR_orig);
      ur_calib->SetUsingCalibratedModel(false);  // using uncalibrated model
      ret = ur_calib->CartToJnt(
          df_pose_UR_orig,
          &qUR_orig);  // compute ideal joint from idela orientation
      if (ret < 0) {
        strs.str("");
        strs << GetName() << " IK error, code  " << ret << ", cart is "
             << df_pose_UR_orig.ToString(false)
             << "can not do error compensation in" << __FUNCTION__ << ", line "
             << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return ret;
      }
      Eigen::VectorXd qUR_orig_vec;
      StdVec2EigenVec(qUR_orig, &qUR_orig_vec);
      strs.str("");
      strs << GetName() << ":, ideal UR joint=" << qUR_orig_vec << std::endl;
      LOG_INFO(strs);

      ret = ur_calib->OptimizeJntAfterCalib(qUR_orig, rf_pose_UR_orig, &qUR_cal);
      if (ret < 0) {
        strs.str("");
        strs << GetName() << ":"
             << "UR OptimizeJntAfterCalib error, code  " << ret
             << "can not do error compensation in " << __FUNCTION__ << ", line "
             << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return ret;
      }
      StdVec2EigenVec(qUR_cal, &act_UR);
      strs.str("");
      strs << GetName() << ":, compensated UR joint=" << act_UR << std::endl;
      LOG_INFO(strs);
    }
    // using uncalibrated model to compute the comp (or modified) orientation
    ur_calib->SetUsingCalibratedModel(false);
    ret = ur_calib->JntToCart(qUR_cal, &df_pose_UR_calib);
    if (ret < 0) {
      strs.str("");
      strs << GetName() << " FK error, code  " << ret << ", jnt is " << act_UR
           << ", can not do error compensation in" << __FUNCTION__ << ", line "
           << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return ret;
    }
    refPose def_pose;
    def_pose.setDefaultPose(df_pose_UR_calib);
    def_pose.getPoseUnderNewRef(Frame(), oldBaseTCP, &rf_pose_UR_calib);
    strs.str("");
    strs << GetName() << ", rf_pose_UR_calib inside URJnt="
         << rf_pose_UR_calib.ToString(true) << std::endl;
    LOG_INFO(strs);

    // using calibrated model to compute the ideal UR pose
    ur_calib->SetUsingCalibratedModel(true);
    ret = ur_calib->JntToCart(qUR_cal, &df_pose_UR_orig);
    if (ret < 0) {
      strs.str("");
      strs << GetName() << " FK error, code  " << ret << ", jnt is " << act_UR
           << ", can not do error compensation in" << __FUNCTION__ << ", line "
           << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return ret;
    }
    def_pose.setDefaultPose(df_pose_UR_orig);
    def_pose.getPoseUnderNewRef(Frame(), newBaseTCP, &rf_pose_UR_orig);
    strs.str("");
    strs << GetName()
         << ", rf_pose_UR_orig inside URJnt=" << rf_pose_UR_orig.ToString(true)
         << std::endl;
    LOG_INFO(strs);

    // ideal UR turns and flags
    std::vector<int> UR_flags_orig, UR_flags_cal;
    std::vector<int> UR_turns_orig, UR_turns_cal;
    rf_pose_UR_orig.getBranchFlags(&UR_flags_orig);
    rf_pose_UR_orig.getJointTurns(&UR_turns_orig);
    rf_pose_UR_calib.getBranchFlags(&UR_flags_cal);
    rf_pose_UR_calib.getJointTurns(&UR_turns_cal);

    Eigen::VectorXd d_UR_j_traj(ur_calib->GetDoF()), md_UR_j_traj(ur_calib->GetDoF());
    StdVec2EigenVec(qUR_orig, &d_UR_j_traj);  // ideal model  joint traj
    StdVec2EigenVec(qUR_cal, &md_UR_j_traj);  // compensated joint traj

    // now we able able to compute act_calibBase and act_origBase
    // **** note *** python GUI need to add option to set up newBaseTCP and
    // oldBaseTCP if 8pt is not used
    Frame tmp_orig;
    df_pose_UR_orig.getFrame(&tmp_orig);
    act_calibBase = tmp_orig * newBaseTCP;  // ideal base for XYZ
    act_origBase = tmp_orig * oldBaseTCP;   // error base for XYZ   (note: here
                                           // we use both pose_UR_orig (ideal UR
                                           // pose) to decouple UR and XYZ comp)
    strs.str("");
    strs << "frame_UR_orig=" << tmp_orig.ToString(true) << std::endl;
    strs << GetName() << ": newBaseTCP=" << newBaseTCP.ToString(true)
         << ", oldBaseTCP=" << oldBaseTCP.ToString(true) << std::endl;
    strs << GetName() << ", act_calibBase=" << act_calibBase.ToString(true)
         << ", act_origBase=" << act_origBase.ToString(true) << ", at function "
         << __FUNCTION__ << ", line" << __LINE__ << std::endl;
    LOG_INFO(strs);

    // the following is just simply using XYZ API ErrCompsationDH to compensate
    // XYZ traj
    Vec v1(cur_pt.segment(0, 3));
    Frame d_traj_fm;
    d_traj_fm.setTranslation(v1);
    Rotation r = act_calibBase.getRotation()
                     .Inverse();  // combined with act_calibBase, their rotation
                                  // should be identity, because xyz only
                                  // accepts identity orientation
    d_traj_fm.setRotation(r);

    std::vector<int> ikXYZTurns(xyz_calib->GetDoF(), 0);
    std::vector<int> branchXYZ(3, 0);
    std::vector<int> XYZURTurns_orig, XYZURTurns_cal;
    XYZURTurns_orig.insert(XYZURTurns_orig.end(), ikXYZTurns.begin(),
                           ikXYZTurns.end());
    XYZURTurns_orig.insert(XYZURTurns_orig.end(), UR_turns_orig.begin(),
                           UR_turns_orig.end());
    XYZURTurns_cal.insert(XYZURTurns_cal.end(), ikXYZTurns.begin(),
                          ikXYZTurns.end());
    XYZURTurns_cal.insert(XYZURTurns_cal.end(), UR_turns_cal.begin(),
                          UR_turns_cal.end());

    // desired relative pose w.r.t. block base
    Pose dPose(d_traj_fm, branchXYZ, ikXYZTurns);
    // modified pose relative to old block base/ actual pose relative calib
    // block base
    Pose mdPose, aPose;
    // ideal uncompensated xyz traj / compensated xyz traj
    Eigen::VectorXd d_XYZ_j_traj(xyz_calib->GetDoF());
    Eigen::VectorXd md_XYZ_j_traj(xyz_calib->GetDoF());

    // step 2: call xyz_calib->CompSateEachPt (to be done in serialArm.cpp)
    ret =
        xyz_calib->ErrCompensationDH(act_calibBase, act_origBase, newXYZ_Tool, dPose,
                               &mdPose, &d_XYZ_j_traj, &md_XYZ_j_traj, &aPose);
    if (ret < 0) {
      strs.str("");
      strs << GetName() << " xyz_calib->ErrCompensationDH error  " << ret
           << ", cart is " << dPose.ToString(true) << __FUNCTION__ << ", line "
           << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return ret;
    }
    d_j_traj->col(i).segment(0, xyz_calib->GetDoF()) = d_XYZ_j_traj;
    d_j_traj->col(i).segment(xyz_calib->GetDoF(), ur_calib->GetDoF()) = d_UR_j_traj;

    dPose.setQuaternion(df_pose_UR_orig.getQuaternion());
    dPose.setBranchFlags(UR_flags_orig);
    dPose.setJointTurns(XYZURTurns_orig);
    mdPose.setQuaternion(df_pose_UR_calib.getQuaternion());

    mdPose.setBranchFlags(UR_flags_cal);
    mdPose.setJointTurns(XYZURTurns_cal);
    aPose.setQuaternion(df_pose_UR_orig.getQuaternion());
    aPose.setBranchFlags(UR_flags_orig);
    aPose.setJointTurns(XYZURTurns_orig);
    md_traj->col(i) = mdPose.ToEigenVecPose();
    a_traj->col(i) =
        aPose.ToEigenVecPose();  // a_traj will return desired traj, aPose is
                                 // just for logging purpose

    md_j_traj->col(i).segment(0, xyz_calib->GetDoF()) = md_XYZ_j_traj;
    md_j_traj->col(i).segment(xyz_calib->GetDoF(), ur_calib->GetDoF()) = md_UR_j_traj;

    strs.str("");
    strs << GetName() << " pt  " << i << " desired joint "
         << d_j_traj->col(i).transpose() << ", comp joint "
         << md_j_traj->col(i).transpose() << std::endl
         << " desired cart " << dPose.ToString(true) << ", comp cart "
         << mdPose.ToString(true) << ", actual cart " << aPose.ToString(true)
         << std::endl
         << ", at function " << __FUNCTION__ << ", line" << __LINE__
         << std::endl;
    LOG_INFO(strs);
  }
  return 0;
}

int XyzUrCalib::PathFiltering(
    const Eigen::VectorXd
        &calibTCP,  //  calibrated TCP of block w.r.t. UR flange
    const Eigen::VectorXd &origTCP,  // original TCP of block w.r.t. UR flange
    const Eigen::VectorXd &bestTool, const Eigen::MatrixXd &d_traj,
    Eigen::MatrixXd *md_traj, Eigen::MatrixXd *d_j_traj,
    Eigen::MatrixXd *md_j_traj, Eigen::MatrixXd *a_traj) {
  std::ostringstream strs;
  if (!md_traj || !d_j_traj || !md_j_traj || !a_traj) {
    strs.str("");
    strs << GetName() << ":"
         << "Input pointer is null"
         << " so can not do compensation, in function " << __FUNCTION__
         << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_INPUT_POINTER_NULL;
  }
  if (!isDHCalibrated_) {
    strs.str("");
    strs << GetName() << ":"
         << " robot is not calibrated, "
         << "can not do error compensation in" << __FUNCTION__ << ", line "
         << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_COMP_WITHOUT_CALIB;
  }
  // generate base and tool frame from the input data
  if (calibTCP.rows() != 7 || origTCP.rows() != 7 || bestTool.rows() != 7) {
    strs.str("");
    strs << GetName() << ":"
         << "Input Tcp/tool data is not translation + quaternion"
         << " so can not do calibration, in function " << __FUNCTION__
         << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_INPUT_PARA_WRONG_DIM;
  }
  strs.str("");
  strs << GetName() << ":"
       << "In traj. compensation: calib UR Tcp= " << calibTCP
       << ", uncalibrated UR tcp= " << origTCP << ", desired tool=" << bestTool
       << std::endl;
  LOG_INFO(strs);

  size_t numPts = d_traj.cols();
  size_t numRows = d_traj.rows();
  // resize output variables
  md_traj->resize(9, numPts);
  d_j_traj->resize(DoF_, numPts);
  md_j_traj->resize(DoF_, numPts);
  a_traj->resize(9, numPts);

  // XYZ tool
  Frame newXYZ_Tool;
  Vec t(bestTool.segment(0, 3));
  newXYZ_Tool.setTranslation(t);
  Quaternion q(bestTool(3), bestTool(4), bestTool(5), bestTool(6));
  newXYZ_Tool.setQuaternion(q);

  // calibrated TCP of block
  Vec bv(calibTCP(0), calibTCP(1), calibTCP(2));
  Quaternion bq(calibTCP(3), calibTCP(4), calibTCP(5), calibTCP(6));
  Frame newBaseTCP(bq, bv);

  // original TCP of block
  Vec bv_old(origTCP(0), origTCP(1), origTCP(2));
  Quaternion bq_old(origTCP(3), origTCP(4), origTCP(5), origTCP(6));
  Frame oldBaseTCP(bq_old, bv_old);

  // At every pt in desired traj, we need to  compute ref UR pose in calibrated
  // model, and in orig model
  refPose rf_pose_UR_calib, rf_pose_UR_orig;

  // default pose in orig and calib
  Pose df_pose_UR_orig, df_pose_UR_calib;

  // compensated UR joints
  Eigen::VectorXd act_UR;

  // At every pt in desired traj, what is calibrated (accurate) UR joint and
  // orig (or errored/not accurate) UR joint
  Eigen::VectorXd qUR_cal(ur_calib->GetDoF(), 0), qUR_orig(ur_calib->GetDoF(), 0);
  // next we need to compute, at very pt, the calib / orig base of block w.r.t.
  // XYZ base using pose_UR_calib and pose_UR_orig and calibTCP /origTCP
  Frame calibBase, origBase;

  int ret = 0;
  // boolean to denote whether desired traj file contains UR desired joint or
  // desired orientation
  bool URJnt = d_traj.rows() <= 5 ? true : false;
  for (size_t i = 0; i < numPts; i++) {
    Eigen::VectorXd cur_pt = d_traj.col(i);
    strs.str("");
    strs << "cur_pt = " << cur_pt.transpose() << std::endl;
    LOG_INFO(strs);
    if (URJnt) {  // if desired traj contains desired UR joint traj
      // note: here act_UR is already the modified UR traj
      act_UR = cur_pt.segment(xyz_calib->GetDoF(), ur_calib->GetDoF());
      EigenVec2StdVec(act_UR, &qUR_cal);

      qUR_orig = qUR_cal;  // if desired traj was given as (x,y,z,Ud,Rd), then
                           // qUR_cal = qUR_orig={Ud, Rd}
    } else {  // input traj contains UR desired orientation info w.r.t. current
              // subTCP
      Quaternion qtmp(cur_pt(3), cur_pt(4), cur_pt(5), cur_pt(6));
      Vec v(0, 0, 0);
      size_t inFlag = cur_pt(7);
      // for scara, there is only 1 branch flag: elbow (up or down), and for
      // six-axis robot, there are 3 flags
      std::vector<int> branchFlags;
      ConvertBranchFlag(inFlag, &branchFlags);
      // convert joint turn data into ikJointTurns
      size_t tFlag = cur_pt(8);
      std::vector<int> ikJointTurns(DoF_, 0);
      ConvertMultiTurnFlag(tFlag, &ikJointTurns);
      std::vector<int> ikTurns_UR(ur_calib->GetDoF(), 0);
      for (size_t j = 0; j < ur_calib->GetDoF(); j++) {
        ikTurns_UR[j] = ikJointTurns[j + xyz_calib->GetDoF()];
      }

      Frame tmp_UR_fm(qtmp, v);  // initially set v as 0, and just use ideal
                                 // orientation to compute qUR_orig and qUR_cal
      rf_pose_UR_orig.setFrame(tmp_UR_fm);
      rf_pose_UR_orig.setBranchFlags(branchFlags);
      rf_pose_UR_orig.setJointTurns(ikTurns_UR);
      rf_pose_UR_orig.setTool(
          newBaseTCP);  // here we have use newBaseTCP, but not oldBaseTCP,
                        // because otherwise there will be more err.

      // default pose in default base and tool
      rf_pose_UR_orig.getDefaultPose(&df_pose_UR_orig);
      ur_calib->SetUsingCalibratedModel(false);  // using uncalibrated model
      ret = ur_calib->CartToJnt(
          df_pose_UR_orig,
          &qUR_orig);  // compute ideal joint from idela orientation
      if (ret < 0) {
        strs.str("");
        strs << GetName() << " IK error, code  " << ret << ", cart is "
             << df_pose_UR_orig.ToString(false)
             << "can not do error compensation in" << __FUNCTION__ << ", line "
             << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return ret;
      }
      Eigen::VectorXd qUR_orig_vec;
      StdVec2EigenVec(qUR_orig, &qUR_orig_vec);
      strs.str("");
      strs << GetName() << ":, ideal UR joint=" << qUR_orig_vec << std::endl;
      LOG_INFO(strs);

      /*
      strs.str("");
      strs << GetName() << " after canonical IK() pose_UR_orig  " <<
      pose_UR_orig.ToString(true) << ", quR_orig= " <<  qUR_orig_vec
            << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_INFO(strs);
       */

      // using uncalibrated model to compute the final ideal pose (because whose
      // trans might not be 0)
      /*
      ret = ur_calib->JntToCart(qUR_orig, &df_Pos_UR_orig);
      if (ret < 0) {
        strs.str("");
        strs << GetName() << " FK error, code  " << ret << ", jnt is " <<
      qUR_orig_vec
            << ", can not do error compensation in"
            << __FUNCTION__ << ", line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return ret;
      } */

      /*
      strs.str("");
      strs << GetName() << " after canonical FK() pose_UR_orig  " <<
      pose_UR_orig.ToString(true) << ", quR_orig= " <<  qUR_orig_vec
            << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_INFO(strs);
      */

      // set 2: compute the compensated jnt
      // refPose  ref_pose_UR_orig;
      // refPose  tmpRefOrig;
      // tmpRefOrig.setDefaultPose(df_pose_UR_orig);
      // tmpRefOrig.getPoseUnderNewRef(Frame(), newBaseTCP, &rf_pose_UR_orig);
      ret = ur_calib->OptimizeJntAfterCalib(qUR_orig, rf_pose_UR_orig, &qUR_cal);
      if (ret < 0) {
        strs.str("");
        strs << GetName() << ":"
             << "UR OptimizeJntAfterCalib error, code  " << ret
             << "can not do error compensation in " << __FUNCTION__ << ", line "
             << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return ret;
      }
      StdVec2EigenVec(qUR_cal, &act_UR);
      strs.str("");
      strs << GetName() << ":, compensated UR joint=" << act_UR << std::endl;
      LOG_INFO(strs);
    }
    // using uncalibrated model to compute the comp (or modified) orientation
    ur_calib->SetUsingCalibratedModel(false);
    ret = ur_calib->JntToCart(qUR_cal, &df_pose_UR_calib);
    if (ret < 0) {
      strs.str("");
      strs << GetName() << " FK error, code  " << ret << ", jnt is " << act_UR
           << ", can not do error compensation in" << __FUNCTION__ << ", line "
           << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return ret;
    }
    refPose def_pose;
    def_pose.setDefaultPose(df_pose_UR_calib);
    def_pose.getPoseUnderNewRef(Frame(), oldBaseTCP, &rf_pose_UR_calib);
    strs.str("");
    strs << GetName() << ", rf_pose_UR_calib inside URJnt="
         << rf_pose_UR_calib.ToString(true) << std::endl;
    LOG_INFO(strs);

    // using calibrated model to compute the ideal UR pose
    ur_calib->SetUsingCalibratedModel(true);
    ret = ur_calib->JntToCart(qUR_cal, &df_pose_UR_orig);
    if (ret < 0) {
      strs.str("");
      strs << GetName() << " FK error, code  " << ret << ", jnt is " << act_UR
           << ", can not do error compensation in" << __FUNCTION__ << ", line "
           << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return ret;
    }
    def_pose.setDefaultPose(df_pose_UR_orig);
    def_pose.getPoseUnderNewRef(Frame(), newBaseTCP, &rf_pose_UR_orig);
    strs.str("");
    strs << GetName()
         << ", rf_pose_UR_orig inside URJnt=" << rf_pose_UR_orig.ToString(true)
         << std::endl;
    LOG_INFO(strs);

    // ideal UR turns and flags
    std::vector<int> UR_flags_orig, UR_flags_cal;
    std::vector<int> UR_turns_orig, UR_turns_cal;
    rf_pose_UR_orig.getBranchFlags(&UR_flags_orig);
    rf_pose_UR_orig.getJointTurns(&UR_turns_orig);
    rf_pose_UR_calib.getBranchFlags(&UR_flags_cal);
    rf_pose_UR_calib.getJointTurns(&UR_turns_cal);

    Eigen::VectorXd d_UR_j_traj(ur_calib->GetDoF()), md_UR_j_traj(ur_calib->GetDoF());
    StdVec2EigenVec(qUR_orig, &d_UR_j_traj);  // ideal model  joint traj
    StdVec2EigenVec(qUR_cal, &md_UR_j_traj);  // compensated joint traj

    // now we able able to compute calibBase and origBase
    // **** note *** python GUI need to add option to set up newBaseTCP and
    // oldBaseTCP if 8pt is not used
    Frame tmp_orig;
    df_pose_UR_orig.getFrame(&tmp_orig);
    calibBase = tmp_orig * newBaseTCP;  // ideal base for XYZ
    origBase = tmp_orig * oldBaseTCP;   // error base for XYZ   (note: here we
                                        // use both pose_UR_orig (ideal UR pose)
                                        // to decouple UR and XYZ comp)
    strs.str("");
    strs << "frame_UR_orig=" << tmp_orig.ToString(true) << std::endl;
    strs << GetName() << ": newBaseTCP=" << newBaseTCP.ToString(true)
         << ", oldBaseTCP=" << oldBaseTCP.ToString(true) << std::endl;
    strs << GetName() << ", calibBase=" << calibBase.ToString(true)
         << ", origBase=" << origBase.ToString(true) << ", at function "
         << __FUNCTION__ << ", line" << __LINE__ << std::endl;
    LOG_INFO(strs);

    // the following is just simply using XYZ API ErrCompsationDH to compensate
    // XYZ traj
    Vec v1(cur_pt.segment(0, 3));
    Frame d_traj_fm;
    d_traj_fm.setTranslation(v1);
    Rotation r = calibBase.getRotation()
                     .Inverse();  // combined with calibBase, their rotation
                                  // should be identity, because xyz only
                                  // accepts identity orientation
    d_traj_fm.setRotation(r);

    std::vector<int> ikXYZTurns(xyz_calib->GetDoF(), 0);
    std::vector<int> branchXYZ(3, 0);
    std::vector<int> XYZURTurns_orig, XYZURTurns_cal;
    XYZURTurns_orig.insert(XYZURTurns_orig.end(), ikXYZTurns.begin(),
                           ikXYZTurns.end());
    XYZURTurns_orig.insert(XYZURTurns_orig.end(), UR_turns_orig.begin(),
                           UR_turns_orig.end());
    XYZURTurns_cal.insert(XYZURTurns_cal.end(), ikXYZTurns.begin(),
                          ikXYZTurns.end());
    XYZURTurns_cal.insert(XYZURTurns_cal.end(), UR_turns_cal.begin(),
                          UR_turns_cal.end());

    // desired relative pose w.r.t. block base
    Pose dPose(d_traj_fm, branchXYZ, ikXYZTurns);
    // modified pose relative to old block base/ actual pose relative calib
    // block base
    Pose mdPose, aPose;
    // ideal uncompensated xyz traj / compensated xyz traj
    Eigen::VectorXd d_XYZ_j_traj(xyz_calib->GetDoF());
    Eigen::VectorXd md_XYZ_j_traj(xyz_calib->GetDoF());

    // step 2: call xyz_calib->CompSateEachPt (to be done in serialArm.cpp)
    ret =
        xyz_calib->ErrCompensationDH(calibBase, origBase, newXYZ_Tool, dPose, &mdPose,
                               &d_XYZ_j_traj, &md_XYZ_j_traj, &aPose);
    if (ret < 0) {
      strs.str("");
      strs << GetName() << " xyz_calib->ErrCompensationDH error  " << ret
           << ", cart is " << dPose.ToString(true) << __FUNCTION__ << ", line "
           << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return ret;
    }
    d_j_traj->col(i).segment(0, xyz_calib->GetDoF()) = d_XYZ_j_traj;
    d_j_traj->col(i).segment(xyz_calib->GetDoF(), ur_calib->GetDoF()) = d_UR_j_traj;

    dPose.setQuaternion(df_pose_UR_orig.getQuaternion());
    dPose.setBranchFlags(UR_flags_orig);
    dPose.setJointTurns(XYZURTurns_orig);
    mdPose.setQuaternion(df_pose_UR_calib.getQuaternion());

    mdPose.setBranchFlags(UR_flags_cal);
    mdPose.setJointTurns(XYZURTurns_cal);
    aPose.setQuaternion(df_pose_UR_orig.getQuaternion());
    aPose.setBranchFlags(UR_flags_orig);
    aPose.setJointTurns(XYZURTurns_orig);
    md_traj->col(i) = mdPose.ToEigenVecPose();
    a_traj->col(i) =
        aPose.ToEigenVecPose();  // a_traj will return desired traj, aPose is
                                 // just for logging purpose

    md_j_traj->col(i).segment(0, xyz_calib->GetDoF()) = md_XYZ_j_traj;
    md_j_traj->col(i).segment(xyz_calib->GetDoF(), ur_calib->GetDoF()) = md_UR_j_traj;

    strs.str("");
    strs << GetName() << " pt  " << i << " desired joint "
         << d_j_traj->col(i).transpose() << ", comp joint "
         << md_j_traj->col(i).transpose() << std::endl
         << " desired cart " << dPose.ToString(true) << ", comp cart "
         << mdPose.ToString(true) << ", actual cart " << aPose.ToString(true)
         << std::endl
         << ", at function " << __FUNCTION__ << ", line" << __LINE__
         << std::endl;
    LOG_INFO(strs);
  }
  return 0;
}

double XyzUrCalib::CalibrateLaserOrientation(
    const EigenDRef<Eigen::MatrixXd>
        &jnt_measure,  //  | DoF * 6 | DoF * 6 | DoF * 6 | DoF * 6 | ..... Every
                       //  surface: 1 column points
    const EigenDRef<Eigen::MatrixXd>
        &cart_measure,  // | 8 * 6| 8 * 6| 8 * 6| 8 * 36|  .... every surface:
                        // numPtsInEachPlane pts,  so here each group is 8 * (2
                        // numPtsInEachPlane | p1_low, p2 _low, p3_low, p1_high,
                        // p2_high, p3_high |
    const EigenDRef<Eigen::VectorXd> &laserMat_z_measure,  // not used
    const int laser_channel, const double laser_scale,
    const double laser_value,  // to be finished tomorrow, could be
                               // reconfigurable from GUI
    // const EigenDRef<Eigen::Vector3d> &init_normal,   // init normal vector
    const double max_laser_dist,  // maximal laser distance, under which, laser
                                  // reading is 0
    const int numPtsInEachPlane, const std::vector<int> surfaceArrays) {
  std::ostringstream strs;
  if (!initialized_) {
    strs.str("");
    strs << GetName() << " geometric parameters are not initialized"
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_PARAM_NOT_INITIALIZED;
  }
  // now processing  cart_measure and laserMat_Z_measure to generate cart
  // sequences in planes
  size_t num_row_jnt = jnt_measure.rows();
  size_t num_col_jnt = jnt_measure.cols();

  size_t num_row_cart = cart_measure.rows();
  size_t num_col_cart = cart_measure.cols();

  size_t num_col_laser = laserMat_z_measure.size();

  // either cart measure cols == 3 * jnt measure cols (given one fixed jnt vec,
  // we need at least 3 col cart)
  bool sizeOK =
      ((num_col_cart == num_col_jnt) && (num_col_cart == num_col_laser) &&
       (numPtsInEachPlane >= 3) && (laser_channel >= 0 && laser_channel < 3));
  if (num_row_cart < 3 || num_row_jnt < DoF_ || !sizeOK || num_col_jnt < 16 ||
      (num_col_jnt % numPtsInEachPlane != 0)) {
    strs.str("");
    strs << GetName() << ":"
         << "CalibrateLaserOrientation: input data dimension is not matching"
         << " so can not do calibration, in function " << __FUNCTION__
         << ", line " << __LINE__ << "num_col_cart=" << num_col_cart
         << ", num_col_jnt=" << num_col_jnt
         << ", num_col_laser=" << num_col_laser
         << ", num_row_cart=" << num_row_cart << ", num_row_jnt=" << num_row_jnt
         << ", value of modular=" << num_col_jnt % numPtsInEachPlane
         << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM;
  }
  size_t numPlanes = num_col_jnt / numPtsInEachPlane;
  Eigen::MatrixXd UR_jnt_measure(ur_calib->GetDoF(), numPlanes);
  Eigen::MatrixXd UR_cart_measure(num_row_cart, numPtsInEachPlane * numPlanes);

  double laser0 = laser_value;
  // std::vector<Eigen::Vector3d>  ni(numPlanes);
  // Eigen::VectorXd   bi(numPlanes, 0);
  for (size_t i = 0; i < numPlanes; i++) {
    Eigen::MatrixXd jnts = jnt_measure.block(
        xyz_calib->GetDoF(), i * numPtsInEachPlane, ur_calib->GetDoF(), numPtsInEachPlane);
    Eigen::MatrixXd carts = cart_measure.block(0, i * numPtsInEachPlane,
                                               num_row_cart, numPtsInEachPlane);
    Eigen::MatrixXd laser =
        laserMat_z_measure.segment(i * numPtsInEachPlane, numPtsInEachPlane);
    // first check if all jnts are same
    for (size_t j = 1; j < numPtsInEachPlane; j++) {
      Eigen::VectorXd diff_jnt = jnts.col(j) - jnts.col(0);
      if (diff_jnt.norm() > K_EPSILON) {
        strs.str("");
        strs << GetName() << ":"
             << "CalibrateLaserOrientation: UR joints in each plane is not same"
             << " so can not do calibration, in function " << __FUNCTION__
             << ", line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return -2001;
      }
    }

    // now modify the carts to make sure they are in the same plane
    UR_jnt_measure.col(i) = jnts.col(0);
    for (size_t j = 0; j < numPtsInEachPlane; j++) {
      Eigen::VectorXd cart = carts.col(j);
      cart(laser_channel) =
          cart(laser_channel) - laser_scale * (laser(j) - laser0);
      Eigen::VectorXd tmpJnt, tmpCart;
      xyz_calib->SetUsingCalibratedModel(false);
      if (!xyz_calib->GetJntFromPose(cart, &tmpJnt)) {
        strs.str("");
        strs << "xyz_calib->GetJntFromPose error with cart=" << cart
             << ", in function" << __FUNCTION__ << " at line " << __LINE__
             << std::endl;
        LOG_ERROR(strs);
        return -2002;
      }
      xyz_calib->SetUsingCalibratedModel(true);
      if (!xyz_calib->GetPoseFromJnt(tmpJnt, &cart)) {
        strs.str("");
        strs << "xyz_calib->GetPoseFromJnt error with jnt=" << tmpJnt
             << ", in function" << __FUNCTION__ << " at line " << __LINE__
             << std::endl;
        LOG_ERROR(strs);
        return -2002;
      }
      UR_cart_measure.col(i * numPtsInEachPlane + j) = cart;
    }
  }

  xyz_calib->SetUsingCalibratedModel(false);  // reset into uncalibrated mode
  double ret = ur_calib->CalibrateLaserOrientation(
      UR_jnt_measure, UR_cart_measure, laserMat_z_measure, laser_channel,
      laser_scale, laser_value, max_laser_dist, numPtsInEachPlane,
      surfaceArrays);
  if (ret >= 0) {  // means UR has been calibrated
    if (xyz_calib->isCalibrated()) {
      isDHCalibrated_ = true;  // set entire mechanisms as calibrated
    }
  }
  return ret;
}

int XyzUrCalib::GenerateOriginMeasures(const EigenDRef<Eigen::MatrixXd> &jnt_in,
                                   const int numPtsInPlanes,
                                   EigenDRef<Eigen::MatrixXd> *jnt_out) {
  std::ostringstream strs;
  if (!initialized_ || !jnt_out) {
    strs.str("");
    strs << GetName() << " geometric parameters are not initialized"
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_PARAM_NOT_INITIALIZED;
  }
  size_t numPts = jnt_in.cols();
  if (numPts <= numPtsInPlanes) {
    strs.str("");
    strs << GetName() << "Number of Input joints " << numPts << " is less than "
         << numPtsInPlanes << " points in a plane"
         << ", in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return -1030;
  }
  size_t numRows = jnt_in.rows();
  size_t numPlanes = numPts / numPtsInPlanes;
  jnt_out->resize(numRows, numPts);

  // first of all, using first plane to compute the relative coordinates
  Eigen::MatrixXd relPt(numRows, numPtsInPlanes);
  size_t xyz_dof = xyz_calib->GetDoF();
  size_t ur_dof = ur_calib->GetDoF();
  // grab the first 4 ur_jnts and xyz_jnts
  Eigen::MatrixXd ur_jnts = jnt_in.block(xyz_dof, 0, ur_dof, numPtsInPlanes);
  Eigen::VectorXd jnt_ur_eig = ur_jnts.col(0);
  Eigen::MatrixXd xyz_jnts = jnt_in.block(0, 0, xyz_dof, numPtsInPlanes);
  Eigen::VectorXd jnt_xyz_eig = xyz_jnts.col(0);
  // get first ur vector
  Eigen::VectorXd jnt_xyz, jnt_ur;
  EigenVec2StdVec(jnt_ur_eig, &jnt_ur);

  Pose ps_ur, ps_xyz;
  // set as using calibrated models
  xyz_calib->SetUsingCalibratedModel(true);
  ur_calib->SetUsingCalibratedModel(true);
  // compute the first flange frame
  int ret = ur_calib->JntToCart(jnt_ur, &ps_ur);
  if (ret < 0) {
    strs.str("");
    strs << GetName() << ":"
         << "UR FK error, code  " << ret << ", input jnt= " << jnt_ur_eig
         << ", in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return ret;
  }
  Frame tmp;
  ps_ur.getFrame(&tmp);
  Vec subtcp_vec = tmp.getTranslation();
  Rotation subtcp_rot = tmp.getRotation();

  // now  maps first 4 xyz vectors back to w.r.t. flange frame
  for (size_t i = 0; i < numPtsInPlanes; i++) {
    jnt_xyz_eig = xyz_jnts.col(i);
    EigenVec2StdVec(jnt_xyz_eig, &jnt_xyz);
    ret = xyz_calib->JntToCart(jnt_xyz, &ps_xyz);
    if (ret < 0) {
      strs.str("");
      strs << GetName() << ":"
           << "XYZ FK error, code  " << ret
           << "can not do error compensation in, input jnt=" << jnt_xyz_eig
           << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return ret;
    }
    Vec xyz_vec = ps_xyz.getTranslation();
    Vec rel_pt = subtcp_rot.Transpose() * (xyz_vec - subtcp_vec);
    Eigen::VectorXd relPathPt(5), absPathPt(5);
    relPathPt.segment(0, xyz_dof) = rel_pt.ToEigenVec();
    relPathPt.segment(xyz_dof, ur_dof) = jnt_ur_eig;
    relPt.col(i) = relPathPt;
    absPathPt.segment(0, xyz_dof) = jnt_xyz_eig;
    absPathPt.segment(xyz_dof, ur_dof) = jnt_ur_eig;
    jnt_out->col(i) = absPathPt;
  }
  strs.str("");
  strs << GetName() << ": relPt=" << relPt << std::endl;
  LOG_INFO(strs);
  //  reassign xyz_jnts as relative pts
  xyz_jnts = relPt.block(0, 0, xyz_dof, numPtsInPlanes);
  // then using relative coordinates maps to designated probing points
  for (size_t i = 1; i < numPlanes; i++) {
    ur_jnts = jnt_in.block(xyz_dof, i * numPtsInPlanes, ur_dof, numPtsInPlanes);
    // xyz_jnts = jnt_in.block(0, i * numPtsInPlanes, xyz_dof, numPtsInPlanes);
    jnt_ur_eig = ur_jnts.col(0);
    EigenVec2StdVec(jnt_ur_eig, &jnt_ur);
    // using calibrated model to compute the ideal UR pose
    ret = ur_calib->JntToCart(jnt_ur, &ps_ur);
    if (ret < 0) {
      strs.str("");
      strs << GetName() << " FK error, code  " << ret << ", jnt is "
           << jnt_ur_eig << ", can not do error compensation in" << __FUNCTION__
           << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return ret;
    }
    refPose rf_pose_UR_orig;
    rf_pose_UR_orig.setDefaultPose(ps_ur);
    strs.str("");
    strs << GetName()
         << ", rf_pose_UR_orig inside URJnt=" << rf_pose_UR_orig.ToString(true)
         << std::endl;
    LOG_INFO(strs);

    // ideal UR turns and flags
    std::vector<int> UR_flags_orig, UR_flags_cal;
    std::vector<int> UR_turns_orig, UR_turns_cal;
    rf_pose_UR_orig.getBranchFlags(&UR_flags_orig);
    rf_pose_UR_orig.getJointTurns(&UR_turns_orig);
    rf_pose_UR_orig.getBranchFlags(&UR_flags_cal);
    rf_pose_UR_orig.getJointTurns(&UR_turns_cal);

    // Eigen::VectorXd  d_UR_j_traj(ur_calib->GetDoF()), md_UR_j_traj(ur_calib->GetDoF());
    // StdVec2EigenVec(jnt_ur, &d_UR_j_traj); // compensated joint traj
    // StdVec2EigenVec(jnt_ur, &md_UR_j_traj); // compensated joint traj

    // now we able able to compute calibBase and origBase
    Frame calibBase, origBase;
    rf_pose_UR_orig.getFrame(&calibBase);
    origBase = calibBase;
    strs.str("");
    strs << GetName() << ", calibBase=" << calibBase.ToString(true)
         << ", origBase=" << origBase.ToString(true) << ", at function "
         << __FUNCTION__ << ", line" << __LINE__ << std::endl;
    LOG_INFO(strs);

    // the following is just simply using XYZ API ErrCompsationDH to compensate
    // XYZ traj
    for (size_t j = 0; j < numPtsInPlanes; j++) {
      jnt_xyz_eig = xyz_jnts.col(j);
      Vec v1(jnt_xyz_eig);
      Frame d_traj_fm;
      d_traj_fm.setTranslation(v1);
      std::vector<int> ikXYZTurns(xyz_calib->GetDoF(), 0);
      std::vector<int> branchXYZ(3, 0);
      std::vector<int> XYZURTurns_orig, XYZURTurns_cal;
      XYZURTurns_orig.insert(XYZURTurns_orig.end(), ikXYZTurns.begin(),
                             ikXYZTurns.end());
      XYZURTurns_orig.insert(XYZURTurns_orig.end(), UR_turns_orig.begin(),
                             UR_turns_orig.end());
      XYZURTurns_cal.insert(XYZURTurns_cal.end(), ikXYZTurns.begin(),
                            ikXYZTurns.end());
      XYZURTurns_cal.insert(XYZURTurns_cal.end(), UR_turns_cal.begin(),
                            UR_turns_cal.end());

      // desired relative pose w.r.t. block base
      Pose dPose(d_traj_fm, branchXYZ, ikXYZTurns);
      // modified pose relative to old block base/ actual pose relative calib
      // block base
      Pose mdPose, aPose;
      // ideal uncompensated xyz traj / compensated xyz traj
      Eigen::VectorXd d_XYZ_j_traj(xyz_calib->GetDoF());
      Eigen::VectorXd md_XYZ_j_traj(xyz_calib->GetDoF());

      // step 2: call xyz_calib->CompSateEachPt (to be done in serialArm.cpp)
      ret = xyz_calib->ErrCompensationDH(calibBase, origBase, Frame(), dPose, &mdPose,
                                   &d_XYZ_j_traj, &md_XYZ_j_traj, &aPose);
      if (ret < 0) {
        strs.str("");
        strs << GetName() << " xyz_calib->ErrCompensationDH error  " << ret
             << ", cart is " << dPose.ToString(true) << __FUNCTION__
             << ", line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return ret;
      }

      dPose.setQuaternion(rf_pose_UR_orig.getQuaternion());
      dPose.setBranchFlags(UR_flags_orig);
      dPose.setJointTurns(XYZURTurns_orig);
      mdPose.setQuaternion(rf_pose_UR_orig.getQuaternion());
      mdPose.setBranchFlags(UR_flags_cal);
      mdPose.setJointTurns(XYZURTurns_cal);
      aPose.setQuaternion(rf_pose_UR_orig.getQuaternion());
      aPose.setBranchFlags(UR_flags_orig);
      aPose.setJointTurns(XYZURTurns_orig);
      Eigen::VectorXd absPathPt(5);
      absPathPt.segment(0, xyz_dof) = md_XYZ_j_traj;
      absPathPt.segment(xyz_dof, ur_dof) = jnt_ur_eig;
      jnt_out->col(numPtsInPlanes * i + j) = absPathPt;

      strs.str("");
      strs << GetName() << " pt ( " << i << "," << j << ")"
           << " desired joint " << absPathPt.transpose() << std::endl
           << " desired cart " << dPose.ToString(true) << ", comp cart "
           << mdPose.ToString(true) << ", actual cart " << aPose.ToString(true)
           << std::endl
           << ", at function " << __FUNCTION__ << ", line" << __LINE__
           << std::endl;
      LOG_INFO(strs);
    }
  }
  return 0;
}

int XyzUrCalib::GenerateOriginMeasures(const Eigen::MatrixXd &jnt_in,
                                   const int numPtsInPlanes,
                                   Eigen::MatrixXd *jnt_out) {
  std::ostringstream strs;
  if (!initialized_ || !jnt_out) {
    strs.str("");
    strs << GetName() << " geometric parameters are not initialized"
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_PARAM_NOT_INITIALIZED;
  }
  size_t numPts = jnt_in.cols();
  if (numPts <= numPtsInPlanes) {
    strs.str("");
    strs << GetName() << "Number of Input joints " << numPts << " is less than "
         << numPtsInPlanes << " points in a plane"
         << ", in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return -1030;
  }
  size_t numRows = jnt_in.rows();
  size_t numPlanes = numPts / numPtsInPlanes;
  jnt_out->resize(numRows, numPts);

  // first of all, using first plane to compute the relative coordinates
  Eigen::MatrixXd relPt(numRows, numPtsInPlanes);
  size_t xyz_dof = xyz_calib->GetDoF();
  size_t ur_dof = ur_calib->GetDoF();
  // grab the first 4 ur_jnts and xyz_jnts
  Eigen::MatrixXd ur_jnts = jnt_in.block(xyz_dof, 0, ur_dof, numPtsInPlanes);
  Eigen::VectorXd jnt_ur_eig = ur_jnts.col(0);
  Eigen::MatrixXd xyz_jnts = jnt_in.block(0, 0, xyz_dof, numPtsInPlanes);
  Eigen::VectorXd jnt_xyz_eig = xyz_jnts.col(0);
  // get first ur vector
  Eigen::VectorXd jnt_xyz, jnt_ur;
  EigenVec2StdVec(jnt_ur_eig, &jnt_ur);

  Pose ps_ur, ps_xyz;
  // set as using calibrated models
  xyz_calib->SetUsingCalibratedModel(true);
  ur_calib->SetUsingCalibratedModel(true);
  // compute the first flange frame
  int ret = ur_calib->JntToCart(jnt_ur, &ps_ur);
  if (ret < 0) {
    strs.str("");
    strs << GetName() << ":"
         << "UR FK error, code  " << ret << ", input jnt= " << jnt_ur_eig
         << ", in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return ret;
  }
  Frame tmp;
  ps_ur.getFrame(&tmp);
  Vec subtcp_vec = tmp.getTranslation();
  Rotation subtcp_rot = tmp.getRotation();

  // now  maps first 4 xyz vectors back to w.r.t. flange frame
  for (size_t i = 0; i < numPtsInPlanes; i++) {
    jnt_xyz_eig = xyz_jnts.col(i);
    EigenVec2StdVec(jnt_xyz_eig, &jnt_xyz);
    ret = xyz_calib->JntToCart(jnt_xyz, &ps_xyz);
    if (ret < 0) {
      strs.str("");
      strs << GetName() << ":"
           << "XYZ FK error, code  " << ret
           << "can not do error compensation in, input jnt=" << jnt_xyz_eig
           << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return ret;
    }
    Vec xyz_vec = ps_xyz.getTranslation();
    Vec rel_pt = subtcp_rot.Transpose() * (xyz_vec - subtcp_vec);
    Eigen::VectorXd relPathPt(5), absPathPt(5);
    relPathPt.segment(0, xyz_dof) = rel_pt.ToEigenVec();
    relPathPt.segment(xyz_dof, ur_dof) = jnt_ur_eig;
    relPt.col(i) = relPathPt;
    absPathPt.segment(0, xyz_dof) = jnt_xyz_eig;
    absPathPt.segment(xyz_dof, ur_dof) = jnt_ur_eig;
    jnt_out->col(i) = absPathPt;
  }

  //  reassign xyz_jnts as relative pts
  xyz_jnts = relPt.block(0, 0, xyz_dof, numPtsInPlanes);
  // then using relative coordinates maps to designated probing points
  for (size_t i = 1; i < numPlanes; i++) {
    ur_jnts = jnt_in.block(xyz_dof, i * numPtsInPlanes, ur_dof, numPtsInPlanes);
    // xyz_jnts = jnt_in.block(0, i * numPtsInPlanes, xyz_dof, numPtsInPlanes);
    jnt_ur_eig = ur_jnts.col(0);
    EigenVec2StdVec(jnt_ur_eig, &jnt_ur);
    // using calibrated model to compute the ideal UR pose
    ret = ur_calib->JntToCart(jnt_ur, &ps_ur);
    if (ret < 0) {
      strs.str("");
      strs << GetName() << " FK error, code  " << ret << ", jnt is "
           << jnt_ur_eig << ", can not do error compensation in" << __FUNCTION__
           << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return ret;
    }
    refPose rf_pose_UR_orig;
    rf_pose_UR_orig.setDefaultPose(ps_ur);
    strs.str("");
    strs << GetName()
         << ", rf_pose_UR_orig inside URJnt=" << rf_pose_UR_orig.ToString(true)
         << std::endl;
    LOG_INFO(strs);

    // ideal UR turns and flags
    std::vector<int> UR_flags_orig, UR_flags_cal;
    std::vector<int> UR_turns_orig, UR_turns_cal;
    rf_pose_UR_orig.getBranchFlags(&UR_flags_orig);
    rf_pose_UR_orig.getJointTurns(&UR_turns_orig);
    rf_pose_UR_orig.getBranchFlags(&UR_flags_cal);
    rf_pose_UR_orig.getJointTurns(&UR_turns_cal);

    // Eigen::VectorXd  d_UR_j_traj(ur_calib->GetDoF()), md_UR_j_traj(ur_calib->GetDoF());
    // StdVec2EigenVec(jnt_ur, &d_UR_j_traj); // compensated joint traj
    // StdVec2EigenVec(jnt_ur, &md_UR_j_traj); // compensated joint traj

    // now we able able to compute calibBase and origBase
    Frame calibBase, origBase;
    rf_pose_UR_orig.getFrame(&calibBase);
    origBase = calibBase;
    strs.str("");
    strs << GetName() << ", calibBase=" << calibBase.ToString(true)
         << ", origBase=" << origBase.ToString(true) << ", at function "
         << __FUNCTION__ << ", line" << __LINE__ << std::endl;
    LOG_INFO(strs);

    // the following is just simply using XYZ API ErrCompsationDH to compensate
    // XYZ traj
    for (size_t j = 0; j < numPtsInPlanes; j++) {
      jnt_xyz_eig = xyz_jnts.col(j);
      Vec v1(jnt_xyz_eig);
      Frame d_traj_fm;
      d_traj_fm.setTranslation(v1);
      std::vector<int> ikXYZTurns(xyz_calib->GetDoF(), 0);
      std::vector<int> branchXYZ(3, 0);
      std::vector<int> XYZURTurns_orig, XYZURTurns_cal;
      XYZURTurns_orig.insert(XYZURTurns_orig.end(), ikXYZTurns.begin(),
                             ikXYZTurns.end());
      XYZURTurns_orig.insert(XYZURTurns_orig.end(), UR_turns_orig.begin(),
                             UR_turns_orig.end());
      XYZURTurns_cal.insert(XYZURTurns_cal.end(), ikXYZTurns.begin(),
                            ikXYZTurns.end());
      XYZURTurns_cal.insert(XYZURTurns_cal.end(), UR_turns_cal.begin(),
                            UR_turns_cal.end());

      // desired relative pose w.r.t. block base
      Pose dPose(d_traj_fm, branchXYZ, ikXYZTurns);
      // modified pose relative to old block base/ actual pose relative calib
      // block base
      Pose mdPose, aPose;
      // ideal uncompensated xyz traj / compensated xyz traj
      Eigen::VectorXd d_XYZ_j_traj(xyz_calib->GetDoF());
      Eigen::VectorXd md_XYZ_j_traj(xyz_calib->GetDoF());

      // step 2: call xyz_calib->CompSateEachPt (to be done in serialArm.cpp)
      ret = xyz_calib->ErrCompensationDH(calibBase, origBase, Frame(), dPose, &mdPose,
                                   &d_XYZ_j_traj, &md_XYZ_j_traj, &aPose);
      if (ret < 0) {
        strs.str("");
        strs << GetName() << " xyz_calib->ErrCompensationDH error  " << ret
             << ", cart is " << dPose.ToString(true) << __FUNCTION__
             << ", line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return ret;
      }

      dPose.setQuaternion(rf_pose_UR_orig.getQuaternion());
      dPose.setBranchFlags(UR_flags_orig);
      dPose.setJointTurns(XYZURTurns_orig);
      mdPose.setQuaternion(rf_pose_UR_orig.getQuaternion());
      mdPose.setBranchFlags(UR_flags_cal);
      mdPose.setJointTurns(XYZURTurns_cal);
      aPose.setQuaternion(rf_pose_UR_orig.getQuaternion());
      aPose.setBranchFlags(UR_flags_orig);
      aPose.setJointTurns(XYZURTurns_orig);
      Eigen::VectorXd absPathPt(5);
      absPathPt.segment(0, xyz_dof) = md_XYZ_j_traj;
      absPathPt.segment(xyz_dof, ur_dof) = jnt_ur_eig;
      jnt_out->col(numPtsInPlanes * i + j) = absPathPt;

      strs.str("");
      strs << GetName() << " pt ( " << i << "," << j << ")"
           << " desired joint " << absPathPt.transpose() << std::endl
           << " desired cart " << dPose.ToString(true) << ", comp cart "
           << mdPose.ToString(true) << ", actual cart " << aPose.ToString(true)
           << std::endl
           << ", at function " << __FUNCTION__ << ", line" << __LINE__
           << std::endl;
      LOG_INFO(strs);
    }
  }
  return 0;
}

double XyzUrCalib::LaserCalibrateOrientation(
    const Eigen::MatrixXd &jnt_measure, const Eigen::MatrixXd &cart_measure,
    const Eigen::VectorXd &laserMat_z_measure,
    // const EigenDRef<Eigen::Vector3d> &init_normal,   // init normal vector
    const int laser_channel, const double laser_scale,
    const double laser_value,  // to be finished tomorrow, could be
                               // reconfigurable from GUI
    const double
        max_laser_dist,  // maximal laser dist, under which, laser reading is 0
    const int numPtsInEachPlane, const std::vector<int> surfaceArrays) {
  std::ostringstream strs;
  if (!initialized_) {
    strs.str("");
    strs << GetName() << " geometric parameters are not initialized"
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_PARAM_NOT_INITIALIZED;
  }
  // now processing  cart_measure and laserMat_Z_measure to generate cart
  // sequences in planes
  size_t num_row_jnt = jnt_measure.rows();
  size_t num_col_jnt = jnt_measure.cols();

  size_t num_row_cart = cart_measure.rows();
  size_t num_col_cart = cart_measure.cols();

  size_t num_col_laser = laserMat_z_measure.size();

  // either cart measure cols == 3 * jnt measure cols (given one fixed jnt vec,
  // we need at least 3 col cart)
  bool sizeOK =
      ((num_col_cart == num_col_jnt) && (num_col_cart == num_col_laser) &&
       (numPtsInEachPlane >= 3) && (laser_channel >= 0 && laser_channel < 3));
  if (num_row_cart < 3 || num_row_jnt < DoF_ || !sizeOK || num_col_jnt < 16 ||
      (num_col_jnt % numPtsInEachPlane != 0)) {
    strs.str("");
    strs << GetName() << ":"
         << "CalibrateLaserOrientation: input data dimension is not matching"
         << " so can not do calibration, in function " << __FUNCTION__
         << ", line " << __LINE__ << "num_col_cart=" << num_col_cart
         << ", num_col_jnt=" << num_col_jnt
         << ", num_col_laser=" << num_col_laser
         << ", num_row_cart=" << num_row_cart << ", num_row_jnt=" << num_row_jnt
         << ", value of modular=" << num_col_jnt % numPtsInEachPlane
         << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM;
  }
  size_t numPlanes = num_col_jnt / numPtsInEachPlane;
  Eigen::MatrixXd UR_jnt_measure(ur_calib->GetDoF(), numPlanes);
  Eigen::MatrixXd UR_cart_measure(num_row_cart, numPtsInEachPlane * numPlanes);

  double laser0 = laser_value;
  // std::vector<Eigen::Vector3d>  ni(numPlanes);
  // Eigen::VectorXd   bi(numPlanes, 0);
  for (size_t i = 0; i < numPlanes; i++) {
    Eigen::MatrixXd jnts = jnt_measure.block(
        xyz_calib->GetDoF(), i * numPtsInEachPlane, ur_calib->GetDoF(), numPtsInEachPlane);
    Eigen::MatrixXd carts = cart_measure.block(0, i * numPtsInEachPlane,
                                               num_row_cart, numPtsInEachPlane);
    Eigen::MatrixXd laser =
        laserMat_z_measure.segment(i * numPtsInEachPlane, numPtsInEachPlane);
    // first check if all jnts are same
    for (size_t j = 1; j < numPtsInEachPlane; j++) {
      Eigen::VectorXd diff_jnt = jnts.col(j) - jnts.col(0);
      if (diff_jnt.norm() > K_EPSILON) {
        strs.str("");
        strs << GetName() << ":"
             << "CalibrateLaserOrientation: UR joints in each plane is not same"
             << " so can not do calibration, in function " << __FUNCTION__
             << ", line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return -2001;
      }
    }

    // now modify the carts to make sure they are in the same plane
    UR_jnt_measure.col(i) = jnts.col(0);
    for (size_t j = 0; j < numPtsInEachPlane; j++) {
      Eigen::VectorXd cart = carts.col(j);
      cart(laser_channel) =
          cart(laser_channel) - laser_scale * (laser(j) - laser0);
      Eigen::VectorXd tmpJnt, tmpCart;
      xyz_calib->SetUsingCalibratedModel(false);
      if (!xyz_calib->GetJntFromPose(cart, &tmpJnt)) {
        strs.str("");
        strs << "xyz_calib->GetJntFromPose error with cart=" << cart
             << ", in function" << __FUNCTION__ << " at line " << __LINE__
             << std::endl;
        LOG_ERROR(strs);
        return -2002;
      }
      xyz_calib->SetUsingCalibratedModel(true);
      if (!xyz_calib->GetPoseFromJnt(tmpJnt, &cart)) {
        strs.str("");
        strs << "xyz_calib->GetPoseFromJnt error with jnt=" << tmpJnt
             << ", in function" << __FUNCTION__ << " at line " << __LINE__
             << std::endl;
        LOG_ERROR(strs);
        return -2002;
      }
      UR_cart_measure.col(i * numPtsInEachPlane + j) = cart;
    }
  }

  xyz_calib->SetUsingCalibratedModel(false);  // reset into uncalibrated mode
  double ret = ur_calib->LaserCalibrateOrientation(
      UR_jnt_measure, UR_cart_measure, laserMat_z_measure, laser_channel,
      laser_scale, laser_value, max_laser_dist, numPtsInEachPlane,
      surfaceArrays);
  if (ret >= 0) {  // means UR has been calibrated
    if (xyz_calib->isCalibrated()) {
      isDHCalibrated_ = true;  // set entire mechanisms as calibrated
    }
  }
  return ret;
}

double XyzUrCalib::CalibrateLaserOrigin(
    // jnt measure matrix  j1 / j2 /j3 /j4 /j5
    const EigenDRef<Eigen::MatrixXd> &jnt_measure,
    // cart     x/y/z/Orient, |p1, p2, p3, p4| for A1  |p5, p6, p7, p8| for A1'
    const EigenDRef<Eigen::MatrixXd> &cart_measure,
    // z laser values
    const EigenDRef<Eigen::VectorXd> &laserMat_z_measure,
    const int laser_channel,
    const double laser_scale,     // laser scale for the above laser channel
    const double laser_value,     // to be finished tomorrow, could be
                                  // reconfigurable from GUI
    const double max_laser_dist,  // maximal laser distance, under which, laser
                                  // reading is 0
    // 4 in a plane in each direction or 8 for a complete plane, then totally 8
    // * 3, 24 pts
    const int numPtsInEachOrientPlane) {
  std::ostringstream strs;
  if (!initialized_) {
    strs.str("");
    strs << GetName() << " geometric parameters are not initialized"
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_PARAM_NOT_INITIALIZED;
  }
  try {
    // now processing  cart_measure and laserMat_Z_measure to generate cart
    // sequences in planes
    size_t num_row_jnt = jnt_measure.rows();
    size_t num_col_jnt = jnt_measure.cols();

    size_t num_row_cart = cart_measure.rows();
    size_t num_col_cart = cart_measure.cols();

    size_t num_col_laser = laserMat_z_measure.size();

    // either cart measure cols == 3 * jnt measure cols (given one fixed jnt
    // vec, we need at least 3 col cart)
    bool sizeOK =
        ((num_col_cart == num_col_jnt) && (num_col_cart == num_col_laser) &&
         (numPtsInEachOrientPlane >= 4) &&
         (laser_channel >= 0 && laser_channel < 3));
    if (num_row_cart < 3 || num_row_jnt < DoF_ || !sizeOK || num_col_jnt < 16 ||
        (num_col_jnt % numPtsInEachOrientPlane != 0)) {
      strs.str("");
      strs << GetName() << ":"
           << "CalibrateLaserOrigin: input data dimension is not matching"
           << " so can not do calibration, in function " << __FUNCTION__
           << ", line " << __LINE__ << "num_col_cart=" << num_col_cart
           << ", num_col_jnt=" << num_col_jnt
           << ", num_col_laser=" << num_col_laser
           << ", num_row_cart=" << num_row_cart
           << ", num_row_jnt=" << num_row_jnt << ", value of modular="
           << num_col_jnt % (2 * numPtsInEachOrientPlane) << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM;
    }
    size_t numPlanes = num_col_jnt / numPtsInEachOrientPlane;
    if (numPlanes < 5) {
      strs.str("");
      strs << GetName() << ":"
           << "CalibrateLaserOrigin: there are only " << numPlanes
           << " planes in origin measuring, in " << __FUNCTION__ << ", line "
           << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM;
    }

    // try to make sure all points have same z laser reading values
    double max_laser = laser_value;  // laserMat_z_measure.maxCoeff();  // need
                                     // to be changed into configurable value
    double laser_length = max_laser_dist - laser_value;
    strs.str("");
    strs << GetName() << ": max laser=" << max_laser
         << ", laser scale= " << laser_scale
         << ", laser_length=" << laser_length << std::endl;
    LOG_INFO(strs);

    // data for compute o, a1, vd (new algorithm)
    Eigen::MatrixXd MM(numPlanes, 5);
    Eigen::VectorXd bb(numPlanes);

    Eigen::VectorXd UR_jnt;
    ur_calib->SetUsingCalibratedModel(true);
    Eigen::Vector3d dv1, dv2, cN, cN1, cN2, cN3;
    std::vector<Eigen::Vector3d> pp(numPtsInEachOrientPlane);
    for (size_t i = 0; i < numPlanes; i++) {
      Eigen::MatrixXd carts =
          cart_measure.block(0, i * numPtsInEachOrientPlane, num_row_cart,
                             numPtsInEachOrientPlane);
      Eigen::MatrixXd laser = laserMat_z_measure.segment(
          i * numPtsInEachOrientPlane, numPtsInEachOrientPlane);
      Eigen::MatrixXd jnts =
          jnt_measure.block(xyz_calib->GetDoF(), i * numPtsInEachOrientPlane,
                            ur_calib->GetDoF(), numPtsInEachOrientPlane);
      Eigen::MatrixXd jntxyz =
          jnt_measure.block(0, i * numPtsInEachOrientPlane, xyz_calib->GetDoF(),
                            numPtsInEachOrientPlane);
      UR_jnt = jnts.rowwise().mean();  // jnts.col(0);
      // first check if all jnts are same for half points in the plane
      for (size_t j = 1; j < numPtsInEachOrientPlane; j++) {
        Eigen::VectorXd diff_jnt = jnts.col(j) - jnts.col(0);
        if (diff_jnt.norm() > K_EPSILON) {
          strs.str("");
          strs << GetName() << ":"
               << "CalibrateLaserOrigin: UR joints in each plane is not same, "
                  "epsilon="
               << diff_jnt.norm() << ", which is greater than " << K_EPSILON
               << " so can not do calibration, in function " << __FUNCTION__
               << ", line " << __LINE__ << std::endl;
          LOG_ERROR(strs);
          // return -2001;
        }
      }

      // modify the cart so that they are in the same with same z value
      // compute the first corner of plane 1

      std::vector<Vec> p(numPtsInEachOrientPlane);
      for (size_t j = 0; j < numPtsInEachOrientPlane; j++) {
        Eigen::VectorXd cart = carts.col(j);
        cart(laser_channel) =
            cart(laser_channel) -
            laser_scale * (laser(j) - max_laser -
                           laser_length);  // make sure laser light length = 0
        strs.str("");
        Eigen::VectorXd tmpJnt;
        xyz_calib->SetUsingCalibratedModel(false);
        if (!xyz_calib->GetJntFromPose(cart, &tmpJnt)) {
          strs.str("");
          strs << "xyz_calib->GetJntFromPose error with cart=" << cart
               << ", in function" << __FUNCTION__ << " at line " << __LINE__
               << std::endl;
          LOG_ERROR(strs);
          return -2002;
        }
        strs << "xyzjnt=" << jntxyz.col(j)
             << ", original cart=" << cart.segment(0, 3)
             << ", tmpJnt=" << tmpJnt << std::endl;
        xyz_calib->SetUsingCalibratedModel(true);
        if (!xyz_calib->GetPoseFromJnt(tmpJnt, &cart)) {
          strs.str("");
          strs << "xyz_calib->GetPoseFromJnt error with jnt=" << tmpJnt
               << ", in function" << __FUNCTION__ << " at line " << __LINE__
               << std::endl;
          LOG_ERROR(strs);
          return -2002;
        }
        p[j] = Vec(cart.segment(0, 3));
        pp[j] = p[j].ToEigenVec();
        strs << " pj =" << p[j].ToString() << ", pp[j]=" << pp[j] << std::endl;
        LOG_INFO(strs);
      }

      dv1 = pp[1] - pp[0];
      dv2 = pp[2] - pp[1];
      cN = dv1.cross(dv2);
      double norm_nd = cN.norm();
      cN /= norm_nd;                // measured normal
      if (cN(laser_channel) < 0) {  // if cN is not facing same direction as
                                    // positive Z, then reverse cN
        cN = -cN;
      }
      strs.str("");
      strs << "dv1=" << dv1 << ", dv2=" << dv2 << ",cN=" << cN
           << ", norm_nd=" << norm_nd << std::endl;
      LOG_INFO(strs);

      dv1 = pp[3] - pp[2];
      dv2 = pp[0] - pp[3];
      cN1 = dv1.cross(dv2);
      norm_nd = cN1.norm();
      cN1 /= norm_nd;
      if (cN1(laser_channel) < 0) {
        cN1 = -cN1;
      }

      dv1 = pp[2] - pp[1];
      dv2 = pp[3] - pp[2];
      cN2 = dv1.cross(dv2);
      norm_nd = cN2.norm();
      cN2 /= norm_nd;
      if (cN2(laser_channel) < 0) {
        cN2 = -cN2;
      }

      dv1 = pp[0] - pp[3];
      dv2 = pp[1] - pp[0];
      cN3 = dv1.cross(dv2);
      norm_nd = cN3.norm();
      cN3 /= norm_nd;
      if (cN3(laser_channel) < 0) {
        cN3 = -cN3;
      }

      strs.str("");
      strs << "second normal: dv1=" << dv1 << ", dv2=" << dv2 << ",cN1=" << cN1
           << ", cN2=" << cN2 << ", cN3=" << cN3 << std::endl;
      // now doing average of the two normal
      cN = (cN + cN1 + cN2 + cN3) / 4.0;
      norm_nd = cN.norm();
      cN /= norm_nd;  // measured normal
      strs << "average cN=" << cN << "average norm=" << norm_nd << std::endl;
      LOG_INFO(strs);

      Frame fm2;
      Eigen::VectorXd ur_jnt_vec;
      EigenVec2StdVec(UR_jnt, &ur_jnt_vec);
      if (!ur_calib->GetDHFrame(ur_jnt_vec, 0, &fm2)) {
        strs.str("");
        strs << GetName() << "ur_calib->GetDHFrame error with jnt=" << UR_jnt
             << ", at index 1"
             << ", in function" << __FUNCTION__ << " at line " << __LINE__
             << std::endl;
        LOG_ERROR(strs);
        return -2002;
      }
      Rotation r1 = fm2.getRotation();
      Vec x1 = r1.UnitX();
      Eigen::VectorXd eigx1 = x1.ToEigenVec();

      Eigen::VectorXd mcol(5);
      mcol.segment(0, 3) = cN;
      mcol(3) = cN.dot(eigx1);
      mcol(4) = 1.0;

      MM.row(i) = mcol.transpose();
      Eigen::Vector3d meanPt = (pp[0] + pp[1] + pp[2] + pp[3]) / 4.0;
      // meanPt(2) -= fabs(laser_scale) * laser_length;  this is commented out
      // because pp[i] has already been set to laser light length= 0 note above
      // trying to consider laser tcp with laser light length=0    ****** very
      // important ***** then 8pt user frame, also adjust pt coordinate so that
      // laser light length = 0 then in traj compensation, we set tcp(2) = -
      // |laser_scale| * (35-laser_reading) in user tool setting
      bb(i) = cN.dot(meanPt);

      strs.str("");
      strs << "i row of MM = " << mcol.transpose() << ", eigx1=" << eigx1
           << ", meanPt=" << meanPt << ", bb(" << i << ")=" << bb(i)
           << ",meanpt off z=" << laser_length * (1 - cN(2)) / cN(2)
           << std::endl;
      LOG_INFO(strs);
    }

    // 3rd method
    Eigen::MatrixXd MMTMM = MM.transpose() * MM;
    strs.str("");
    strs << "MM = " << MM << ", MMTMM=" << MMTMM << ", bb=" << bb.transpose()
         << std::endl;
    LOG_INFO(strs);
    double det = MMTMM.determinant();
    if (det < CALIB_SINGULAR_CONST) {
      strs.str("");
      strs << GetName() << "determinant of MMTMM (method 4)=" << det
           << " is singular , in function" << __FUNCTION__ << " at line "
           << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -2002;
    }

    // now using 3 rotation matrix, and 12 corner points, compute the center of
    // rotation
    Eigen::VectorXd origin_f3 = MMTMM.inverse() * MM.transpose() * bb;

    // we will reuse the x of this origin
    Eigen::Vector3d origin = origin_f3.segment(0, 3);
    // double d1 = origin_f3(4);
    double a1 = origin_f3(3);
    double v1 = origin_f3(4);
    // double ox = origin(0); // from here we get x coordinates

    // now from circ_data, compute the center point
    strs.str("");
    strs << GetName()
         << " final origin_f3 from transform method 4=" << origin_f3
         << std::endl;
    Eigen::VectorXd errorOrig = MM * origin_f3 - bb;
    strs << "errorOrigf3 =" << errorOrig << ", norm=" << errorOrig.norm()
         << std::endl;
    strs << "a1=" << a1 << ", v1=" << v1 << std::endl;
    LOG_INFO(strs);

    Eigen::MatrixXd planeCenter(5, numPlanes);
    Eigen::MatrixXd planeNormal(3, numPlanes);
    // now verify the results
    for (size_t i = 0; i < numPlanes; i++) {
      Eigen::MatrixXd carts =
          cart_measure.block(0, i * numPtsInEachOrientPlane, num_row_cart,
                             numPtsInEachOrientPlane);
      Eigen::MatrixXd laser = laserMat_z_measure.segment(
          i * numPtsInEachOrientPlane, numPtsInEachOrientPlane);
      Eigen::MatrixXd jnts =
          jnt_measure.block(xyz_calib->GetDoF(), i * numPtsInEachOrientPlane,
                            ur_calib->GetDoF(), numPtsInEachOrientPlane);
      UR_jnt = jnts.rowwise().mean();  // jnts.col(0);
      // modify the cart so that they are in the same with same z value
      // compute the first corner of plane 1

      std::vector<Vec> p(numPtsInEachOrientPlane);
      for (size_t j = 0; j < numPtsInEachOrientPlane; j++) {
        Eigen::VectorXd cart = carts.col(j);
        cart(laser_channel) =
            cart(laser_channel) -
            laser_scale * (laser(j) - max_laser -
                           laser_length);  // make sure laser light length = 0

        Eigen::VectorXd tmpJnt;
        xyz_calib->SetUsingCalibratedModel(false);
        if (!xyz_calib->GetJntFromPose(cart, &tmpJnt)) {
          strs.str("");
          strs << "xyz_calib->GetJntFromPose error with cart=" << cart
               << ", in function" << __FUNCTION__ << " at line " << __LINE__
               << std::endl;
          LOG_ERROR(strs);
          return -2002;
        }
        xyz_calib->SetUsingCalibratedModel(true);
        if (!xyz_calib->GetPoseFromJnt(tmpJnt, &cart)) {
          strs.str("");
          strs << "xyz_calib->GetPoseFromJnt error with jnt=" << tmpJnt
               << ", in function" << __FUNCTION__ << " at line " << __LINE__
               << std::endl;
          LOG_ERROR(strs);
          return -2002;
        }
        p[j] = Vec(cart.segment(0, 3));
        pp[j] = p[j].ToEigenVec();
      }

      dv1 = pp[1] - pp[0];
      dv2 = pp[2] - pp[1];
      cN = dv1.cross(dv2);
      double norm_nd = cN.norm();
      cN /= norm_nd;                // measured normal
      if (cN(laser_channel) < 0) {  // if cN is not facing same direction as
                                    // positive Z, then reverse cN
        cN = -cN;
      }

      dv1 = pp[3] - pp[2];
      dv2 = pp[0] - pp[3];
      cN1 = dv1.cross(dv2);
      norm_nd = cN1.norm();
      cN1 /= norm_nd;
      if (cN1(laser_channel) < 0) {
        cN1 = -cN1;
      }

      dv1 = pp[2] - pp[1];
      dv2 = pp[3] - pp[2];
      cN2 = dv1.cross(dv2);
      norm_nd = cN2.norm();
      cN2 /= norm_nd;
      if (cN2(laser_channel) < 0) {
        cN2 = -cN2;
      }

      dv1 = pp[0] - pp[3];
      dv2 = pp[1] - pp[0];
      cN3 = dv1.cross(dv2);
      norm_nd = cN3.norm();
      cN3 /= norm_nd;
      if (cN3(laser_channel) < 0) {
        cN3 = -cN3;
      }

      // now doing average of the two normal
      cN = (cN + cN1 + cN2 + cN3) / 4.0;
      norm_nd = cN.norm();
      cN /= norm_nd;  // measured normal
      planeNormal.col(i) = cN;

      Frame fm2;
      Eigen::VectorXd ur_jnt_vec;
      EigenVec2StdVec(UR_jnt, &ur_jnt_vec);
      if (!ur_calib->GetDHFrame(ur_jnt_vec, 0, &fm2)) {
        strs.str("");
        strs << GetName() << "ur_calib->GetDHFrame error with jnt=" << UR_jnt
             << ", at index 1"
             << ", in function" << __FUNCTION__ << " at line " << __LINE__
             << std::endl;
        LOG_ERROR(strs);
        return -2002;
      }
      Rotation r1 = fm2.getRotation();
      Vec x1 = r1.UnitX();
      Eigen::VectorXd eigx1 = x1.ToEigenVec();

      Eigen::Vector3d meanPt = (pp[0] + pp[1] + pp[2] + pp[3]) / 4.0;
      // meanPt(2) -= fabs(laser_scale) * laser_length;  this is commented out
      // because pp[i] has already been set to laser light length= 0 note above
      // trying to consider laser tcp with laser light length=0    ****** very
      // important ***** then 8pt user frame, also adjust pt coordinate so that
      // laser light length = 0 then in traj compensation, we set tcp(2) = -
      // |laser_scale| * (35-laser_reading) in user tool setting

      Eigen::Vector3d pred_pt = origin + a1 * eigx1;  //  + cN * v1;

      strs.str("");
      strs << "plane " << i << ", eigx1=" << eigx1 << ", cN=" << cN
           << ", meanPt=" << meanPt << ", pred_pt =" << pred_pt
           << ", actual-pt=" << meanPt << ", error=" << meanPt - pred_pt
           << ", error_proj=" << cN.dot(meanPt - pred_pt) - v1 << std::endl;

      LOG_INFO(strs);

      // now put center data into matrix, and then save into csv file
      Eigen::VectorXd cT(5);
      cT.segment(0, 3) = pred_pt + cN * v1;
      cT.segment(3, 2) = UR_jnt;
      planeCenter.col(i) = cT;
    }

    // now writing planceCenter to csv file
    RPE_Utility util;
    std::string centerFile("planeCenter.txt");
    util.WriteCSVFile(centerFile, planeCenter);
    std::string normalFile("planeNormal.txt");
    util.WriteCSVFile(normalFile, planeNormal);

    // now compute U axis direction, for now just log it
    size_t numGroups = numPlanes / 4;  // each group contains 4 planes
    Eigen::MatrixXd ptGroup(3, 4), ptGroup2(3, 4);
    Eigen::MatrixXd uAxis(3, 4), uAxis2(3, 4);
    for (size_t j = 0; j < numGroups; j++) {
      for (size_t i = 0; i < 4; i++) {
        ptGroup.col(i) = planeCenter.col(4 * i + j).segment(0, 3);
        ptGroup2.col(i) = planeNormal.col(4 * i + j).segment(0, 3);
      }

      // now computing the unit norm of ptGroup
      Eigen::VectorXd meanPt = ptGroup.rowwise().mean();
      Eigen::MatrixXd tmp = ptGroup.colwise() - meanPt;

      // using svd decompsition to find the normal of 8-pt plane
      Eigen::JacobiSVD<Eigen::MatrixXd> svd2(
          tmp.transpose(), Eigen::ComputeFullV | Eigen::ComputeFullU);
      Eigen::MatrixXd tmpV = svd2.matrixV();
      Eigen::Vector3d pnz = tmpV.col(2);
      strs.str("");
      strs << "U group" << j << ", meanPtz=" << meanPt.transpose()
           << ", pnz=" << pnz.transpose() << std::endl;
      LOG_INFO(strs);
      uAxis.col(j) = pnz;

      meanPt = ptGroup2.rowwise().mean();
      tmp = ptGroup2.colwise() - meanPt;

      // using svd decompsition to find the normal of 8-pt plane
      Eigen::JacobiSVD<Eigen::MatrixXd> svd22(
          tmp.transpose(), Eigen::ComputeFullV | Eigen::ComputeFullU);
      tmpV = svd22.matrixV();
      pnz = tmpV.col(2);
      strs.str("");
      strs << "U group using normal" << j << ", meanPtz=" << meanPt.transpose()
           << ", pnz=" << pnz.transpose() << std::endl;
      LOG_INFO(strs);
      uAxis2.col(j) = pnz;
    }
    std::string uAxisFile("uAxis.txt");
    util.WriteCSVFile(uAxisFile, uAxis);
    Eigen::Vector3d uFinalAxis = uAxis.rowwise().mean();
    uFinalAxis /= uFinalAxis.norm();

    std::string uAxisFile2("uAxis2.txt");
    util.WriteCSVFile(uAxisFile2, uAxis2);

    Eigen::Vector3d uFinalAxis2 = uAxis2.rowwise().mean();
    uFinalAxis2 /= uFinalAxis2.norm();

    strs.str("");
    strs << "uFinalAxis=" << uFinalAxis.transpose() << std::endl;
    strs << "uFinalAxis2=" << uFinalAxis2.transpose() << std::endl;
    LOG_INFO(strs);

    // now compute the zero of U axis
    // first compute the direction of all rAxis with given fixed Us
    Eigen::MatrixXd rAxis(3, 4), rAxis2(3, 4);
    Eigen::VectorXd angleRU(4), angleRU2(4);  // angles between U and R
    for (size_t j = 0; j < numGroups; j++) {
      for (size_t i = 0; i < 4; i++) {
        ptGroup.col(i) = planeCenter.col(4 * j + i).segment(0, 3);
        ptGroup2.col(i) = planeNormal.col(4 * j + i).segment(0, 3);
      }

      // now computing the unit norm of ptGroup
      Eigen::VectorXd meanPt = ptGroup.rowwise().mean();
      Eigen::MatrixXd tmp = ptGroup.colwise() - meanPt;

      // using svd decompsition to find the normal of 8-pt plane
      Eigen::JacobiSVD<Eigen::MatrixXd> svd2(
          tmp.transpose(), Eigen::ComputeFullV | Eigen::ComputeFullU);
      Eigen::MatrixXd tmpV = svd2.matrixV();
      Eigen::Vector3d pnz = tmpV.col(2);
      strs.str("");
      strs << "R group" << j << ", meanPtz=" << meanPt.transpose()
           << ", pnz=" << pnz.transpose() << std::endl;
      LOG_INFO(strs);
      rAxis.col(j) = pnz;
      angleRU(j) = acos(pnz.dot(uFinalAxis));

      meanPt = ptGroup2.rowwise().mean();
      tmp = ptGroup2.colwise() - meanPt;

      // using svd decompsition to find the normal of 8-pt plane
      Eigen::JacobiSVD<Eigen::MatrixXd> svd22(
          tmp.transpose(), Eigen::ComputeFullV | Eigen::ComputeFullU);
      tmpV = svd22.matrixV();
      pnz = tmpV.col(2);
      strs.str("");
      strs << "R group normal" << j << ", meanPtz=" << meanPt.transpose()
           << ", pnz=" << pnz.transpose() << std::endl;
      LOG_INFO(strs);
      rAxis2.col(j) = pnz;
      angleRU2(j) = acos(pnz.dot(uFinalAxis2));
    }
    std::string rAxisFile("rAxis.txt");
    util.WriteCSVFile(rAxisFile, rAxis);

    std::string rAxisFile2("rAxis2.txt");
    util.WriteCSVFile(rAxisFile2, rAxis2);

    std::string angleRUFile("angleRU.txt");
    util.WriteCSVFile(angleRUFile, angleRU);

    std::string angleRUFile2("angleRU2.txt");
    util.WriteCSVFile(angleRUFile2, angleRU2);

    Eigen::VectorXd urDH;
    ur_calib->GetCalibParamSet(&urDH);
    urDH(3) = a1;  // alpha0, alpha1, a0, a1, t0, t1, d0, d1, b0, b1
    // urDH(7) = d1;
    Eigen::VectorXd urDHVec;
    EigenVec2StdVec(urDH, &urDHVec);
    ur_calib->LoadCalibParamSet(urDHVec);  // update calibrated DH parameter

    Frame tmp_baseoff, tmp_subbaseoff;
    ur_calib->GetDefaultBaseOffFrame(&tmp_baseoff, &tmp_subbaseoff);

    // now UR mechanism set translational part of baseOffset
    // Frame tmpBase = sub_defaultBaseOff_;
    strs.str("");
    strs << GetName()
         << ": UR orig defaultSubBaseOff=" << tmp_baseoff.ToString(true)
         << std::endl;
    LOG_INFO(strs);
    Vec t(origin);
    tmp_baseoff.setTranslation(t);
    Eigen::VectorXd tmpBaseVec = tmp_baseoff.ToEigenVecQuat();
    strs.str("");
    strs << GetName() << ": UR baseoff is changed into "
         << tmp_baseoff.ToString(true)
         << ", in eigen vector=" << tmpBaseVec.transpose() << std::endl;
    LOG_INFO(strs);
    ur_calib->SetDefaultBaseOff(tmpBaseVec,
                          tmpBaseVec);  // modify the default base of UR obj

    // Vec t1(tmp_baseoff.segment(0,3));
    // Quaternion q1(tmp_baseoff(3), tmp_baseoff(4), tmp_baseoff(5),
    // tmp_baseoff(6));
    sub_defaultBaseOff_ = tmp_baseoff;  // modify sub base off of XYZUR too
    // sub_defaultBaseOff_.setQuaternion(q1);
    strs.str("");
    strs << GetName() << ": modify subdefulatBaseoff into"
         << sub_defaultBaseOff_.ToString(true)
         << ", while defualtbaseoff is still " << defaultBaseOff_.ToString(true)
         << std::endl;
    LOG_INFO(strs);

    ur_calib->updateCalibrateStatus(true);
    if (xyz_calib->isCalibrated()) {
      isDHCalibrated_ = true;  // set entire mechanism is calibrated
    }

    // sub_defaultBaseOff_ = tmpBase;  // modify the default subBase of XYZ_UR
    // obj, defaultBase for XYZ is not changed
    return 0;
  } catch (...) {
    strs.str("");
    strs << GetName() << " gets exception in function " << __FUNCTION__
         << std::endl;
    LOG_ERROR(strs);
    return -1;
  }
}

double XyzUrCalib::LaserCalibrateOrigin(
    // jnt measure matrix  j1 / j2 /j3 /j4 /j5
    const Eigen::MatrixXd &jnt_measure,
    // cart     x/y/z/Orient, |p1, p2, p3, p4| for A1  |p5, p6, p7, p8| for A1'
    const Eigen::MatrixXd &cart_measure,
    // z laser values
    const Eigen::VectorXd &laserMat_z_measure,
    const int laser_channel,   // using which channel of lasr (x or y or z) for
                               // measuring
    const double laser_scale,  // laser scale for the above laser channel
    const double laser_value,  // to be finished tomorrow, could be
                               // reconfigurable from GUI
    const double
        max_laser_dist,  // maximal laser dist, under which, laser reading is 0
    // 4 in a plane in each direction or 8 for a complete plane, then totally 8
    // * 3, 24 pts
    const int numPtsInEachOrientPlane) {
  std::ostringstream strs;
  if (!initialized_) {
    strs.str("");
    strs << GetName() << " geometric parameters are not initialized"
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_PARAM_NOT_INITIALIZED;
  }
  try {
    // now processing  cart_measure and laserMat_Z_measure to generate cart
    // sequences in planes
    size_t num_row_jnt = jnt_measure.rows();
    size_t num_col_jnt = jnt_measure.cols();

    size_t num_row_cart = cart_measure.rows();
    size_t num_col_cart = cart_measure.cols();

    size_t num_col_laser = laserMat_z_measure.size();

    // either cart measure cols == 3 * jnt measure cols (given one fixed jnt
    // vec, we need at least 3 col cart)
    bool sizeOK =
        ((num_col_cart == num_col_jnt) && (num_col_cart == num_col_laser) &&
         (numPtsInEachOrientPlane >= 4) &&
         (laser_channel >= 0 && laser_channel < 3));
    if (num_row_cart < 3 || num_row_jnt < DoF_ || !sizeOK || num_col_jnt < 16 ||
        (num_col_jnt % numPtsInEachOrientPlane != 0)) {
      strs.str("");
      strs << GetName() << ":"
           << "CalibrateLaserOrigin: input data dimension is not matching"
           << " so can not do calibration, in function " << __FUNCTION__
           << ", line " << __LINE__ << "num_col_cart=" << num_col_cart
           << ", num_col_jnt=" << num_col_jnt
           << ", num_col_laser=" << num_col_laser
           << ", num_row_cart=" << num_row_cart
           << ", num_row_jnt=" << num_row_jnt << ", value of modular="
           << num_col_jnt % (2 * numPtsInEachOrientPlane) << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM;
    }
    size_t numPlanes = num_col_jnt / numPtsInEachOrientPlane;
    if (numPlanes < 5) {
      strs.str("");
      strs << GetName() << ":"
           << "CalibrateLaserOrigin: there are only " << numPlanes
           << " planes in origin measuring, in " << __FUNCTION__ << ", line "
           << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM;
    }

    // try to make sure all points have same z laser reading values
    double max_laser = laser_value;  // laserMat_z_measure.maxCoeff();  // need
                                     // to be changed into configurable value
    double laser_length = max_laser_dist - laser_value;
    strs.str("");
    strs << GetName() << ": max laser=" << max_laser
         << ", laser scale= " << laser_scale
         << ", laser_length=" << laser_length << std::endl;
    LOG_INFO(strs);

    // data for compute o, a1, vd (new algorithm)
    Eigen::MatrixXd MM(numPlanes, 5);
    Eigen::VectorXd bb(numPlanes);

    Eigen::VectorXd UR_jnt;
    ur_calib->SetUsingCalibratedModel(true);
    Eigen::Vector3d dv1, dv2, cN, cN1, cN2, cN3;
    std::vector<Eigen::Vector3d> pp(numPtsInEachOrientPlane);
    for (size_t i = 0; i < numPlanes; i++) {
      Eigen::MatrixXd carts =
          cart_measure.block(0, i * numPtsInEachOrientPlane, num_row_cart,
                             numPtsInEachOrientPlane);
      Eigen::MatrixXd laser = laserMat_z_measure.segment(
          i * numPtsInEachOrientPlane, numPtsInEachOrientPlane);
      Eigen::MatrixXd jnts =
          jnt_measure.block(xyz_calib->GetDoF(), i * numPtsInEachOrientPlane,
                            ur_calib->GetDoF(), numPtsInEachOrientPlane);
      Eigen::MatrixXd jntxyz =
          jnt_measure.block(0, i * numPtsInEachOrientPlane, xyz_calib->GetDoF(),
                            numPtsInEachOrientPlane);
      UR_jnt = jnts.rowwise().mean();  // UR_jnt = jnts.col(0);
      // first check if all jnts are same for half points in the plane
      for (size_t j = 1; j < numPtsInEachOrientPlane; j++) {
        Eigen::VectorXd diff_jnt = jnts.col(j) - jnts.col(0);
        if (diff_jnt.norm() > K_EPSILON) {
          strs.str("");
          strs << GetName() << ":"
               << "CalibrateLaserOrigin: UR joints in each plane is not same, "
                  "epsilon="
               << diff_jnt.norm() << ", which is greater than " << K_EPSILON
               << " so can not do calibration, in function " << __FUNCTION__
               << ", line " << __LINE__ << std::endl;
          LOG_ERROR(strs);
          // return -2001;
        }
      }

      // modify the cart so that they are in the same with same z value
      // compute the first corner of plane 1

      std::vector<Vec> p(numPtsInEachOrientPlane);
      for (size_t j = 0; j < numPtsInEachOrientPlane; j++) {
        Eigen::VectorXd cart = carts.col(j);
        cart(laser_channel) =
            cart(laser_channel) -
            laser_scale * (laser(j) - max_laser -
                           laser_length);  // make sure laser light length = 0
        strs.str("");
        Eigen::VectorXd tmpJnt;
        xyz_calib->SetUsingCalibratedModel(false);
        if (!xyz_calib->GetJntFromPose(cart, &tmpJnt)) {
          strs.str("");
          strs << "xyz_calib->GetJntFromPose error with cart=" << cart
               << ", in function" << __FUNCTION__ << " at line " << __LINE__
               << std::endl;
          LOG_ERROR(strs);
          return -2002;
        }
        strs << "xyzjnt=" << jntxyz.col(j)
             << ", original cart=" << cart.segment(0, 3)
             << ", tmpJnt=" << tmpJnt << std::endl;
        xyz_calib->SetUsingCalibratedModel(true);
        if (!xyz_calib->GetPoseFromJnt(tmpJnt, &cart)) {
          strs.str("");
          strs << "xyz_calib->GetPoseFromJnt error with jnt=" << tmpJnt
               << ", in function" << __FUNCTION__ << " at line " << __LINE__
               << std::endl;
          LOG_ERROR(strs);
          return -2002;
        }
        p[j] = Vec(cart.segment(0, 3));
        pp[j] = p[j].ToEigenVec();

        strs << " pj =" << p[j].ToString() << ", pp[j]=" << pp[j] << std::endl;
        LOG_INFO(strs);
      }

      dv1 = pp[1] - pp[0];
      dv2 = pp[2] - pp[1];
      cN = dv1.cross(dv2);
      double norm_nd = cN.norm();
      cN /= norm_nd;                // measured normal
      if (cN(laser_channel) < 0) {  // if cN is not facing same direction as
                                    // positive Z, then reverse cN
        cN = -cN;
      }
      strs.str("");
      strs << "dv1=" << dv1 << ", dv2=" << dv2 << ",cN=" << cN
           << ", norm_nd=" << norm_nd << std::endl;
      LOG_INFO(strs);

      dv1 = pp[3] - pp[2];
      dv2 = pp[0] - pp[3];
      cN1 = dv1.cross(dv2);
      norm_nd = cN1.norm();
      cN1 /= norm_nd;
      if (cN1(laser_channel) < 0) {
        cN1 = -cN1;
      }

      dv1 = pp[2] - pp[1];
      dv2 = pp[3] - pp[2];
      cN2 = dv1.cross(dv2);
      norm_nd = cN2.norm();
      cN2 /= norm_nd;
      if (cN2(laser_channel) < 0) {
        cN2 = -cN2;
      }

      dv1 = pp[0] - pp[3];
      dv2 = pp[1] - pp[0];
      cN3 = dv1.cross(dv2);
      norm_nd = cN3.norm();
      cN3 /= norm_nd;
      if (cN3(laser_channel) < 0) {
        cN3 = -cN3;
      }

      strs.str("");
      strs << "second normal: dv1=" << dv1 << ", dv2=" << dv2 << ",cN1=" << cN1
           << ", cN2=" << cN2 << ", cN3=" << cN3 << std::endl;
      // now doing average of the two normal
      cN = (cN + cN1 + cN2 + cN3) / 4.0;
      norm_nd = cN.norm();
      cN /= norm_nd;  // measured normal
      strs << "average cN=" << cN << "average norm=" << norm_nd << std::endl;
      LOG_INFO(strs);

      Frame fm2;
      Eigen::VectorXd ur_jnt_vec;
      EigenVec2StdVec(UR_jnt, &ur_jnt_vec);
      if (!ur_calib->GetDHFrame(ur_jnt_vec, 0, &fm2)) {
        strs.str("");
        strs << GetName() << "ur_calib->GetDHFrame error with jnt=" << UR_jnt
             << ", at index 1"
             << ", in function" << __FUNCTION__ << " at line " << __LINE__
             << std::endl;
        LOG_ERROR(strs);
        return -2002;
      }
      Rotation r1 = fm2.getRotation();
      Vec x1 = r1.UnitX();
      Eigen::VectorXd eigx1 = x1.ToEigenVec();

      Eigen::VectorXd mcol(5);
      mcol.segment(0, 3) = cN;
      mcol(3) = cN.dot(eigx1);
      mcol(4) = 1.0;

      MM.row(i) = mcol.transpose();
      Eigen::Vector3d meanPt = (pp[0] + pp[1] + pp[2] + pp[3]) / 4.0;
      // meanPt(2) -= fabs(laser_scale) * laser_length;  this is commented out
      // because pp[i] has already been set to laser light length= 0 note above
      // trying to consider laser tcp with laser light length=0    ****** very
      // important ***** then 8pt user frame, also adjust pt coordinate so that
      // laser light length = 0 then in traj compensation, we set tcp(2) = -
      // |laser_scale| * (35-laser_reading) in user tool setting
      bb(i) = cN.dot(meanPt);

      strs.str("");
      strs << "i row of MM = " << mcol.transpose() << ", eigx1=" << eigx1
           << ", meanPt=" << meanPt << ", bb(" << i << ")=" << bb(i)
           << ",meanpt off z=" << laser_length * (1 - cN(2)) / cN(2)
           << std::endl;
      LOG_INFO(strs);
    }

    // 3rd method
    Eigen::MatrixXd MMTMM = MM.transpose() * MM;
    strs.str("");
    strs << "MM = " << MM << ", MMTMM=" << MMTMM << ", bb=" << bb.transpose()
         << std::endl;
    LOG_INFO(strs);
    double det = MMTMM.determinant();
    if (det < CALIB_SINGULAR_CONST) {
      strs.str("");
      strs << GetName() << "determinant of MMTMM (method 4)=" << det
           << " is singular , in function" << __FUNCTION__ << " at line "
           << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -2002;
    }

    // now using 3 rotation matrix, and 12 corner points, compute the center of
    // rotation
    Eigen::VectorXd origin_f3 = MMTMM.inverse() * MM.transpose() * bb;

    // we will reuse the x of this origin
    Eigen::Vector3d origin = origin_f3.segment(0, 3);
    // double d1 = origin_f3(4);
    double a1 = origin_f3(3);
    double v1 = origin_f3(4);
    // double ox = origin(0); // from here we get x coordinates

    // now from circ_data, compute the center point
    strs.str("");
    strs << GetName()
         << " final origin_f3 from transform method 4=" << origin_f3
         << std::endl;
    Eigen::VectorXd errorOrig = MM * origin_f3 - bb;
    strs << "errorOrigf3 =" << errorOrig << ", norm=" << errorOrig.norm()
         << std::endl;
    strs << "a1=" << a1 << ", v1=" << v1 << std::endl;
    LOG_INFO(strs);

    // now verify the results
    Eigen::MatrixXd planeCenter(5, numPlanes);
    Eigen::MatrixXd planeNormal(3, numPlanes);
    for (size_t i = 0; i < numPlanes; i++) {
      Eigen::MatrixXd carts =
          cart_measure.block(0, i * numPtsInEachOrientPlane, num_row_cart,
                             numPtsInEachOrientPlane);
      Eigen::MatrixXd laser = laserMat_z_measure.segment(
          i * numPtsInEachOrientPlane, numPtsInEachOrientPlane);
      Eigen::MatrixXd jnts =
          jnt_measure.block(xyz_calib->GetDoF(), i * numPtsInEachOrientPlane,
                            ur_calib->GetDoF(), numPtsInEachOrientPlane);
      UR_jnt = jnts.rowwise().mean();  // UR_jnt = jnts.col(0);
      // modify the cart so that they are in the same with same z value
      // compute the first corner of plane 1

      std::vector<Vec> p(numPtsInEachOrientPlane);
      for (size_t j = 0; j < numPtsInEachOrientPlane; j++) {
        Eigen::VectorXd cart = carts.col(j);
        cart(laser_channel) =
            cart(laser_channel) -
            laser_scale * (laser(j) - max_laser -
                           laser_length);  // make sure laser light length = 0

        Eigen::VectorXd tmpJnt;
        xyz_calib->SetUsingCalibratedModel(false);
        if (!xyz_calib->GetJntFromPose(cart, &tmpJnt)) {
          strs.str("");
          strs << "xyz_calib->GetJntFromPose error with cart=" << cart
               << ", in function" << __FUNCTION__ << " at line " << __LINE__
               << std::endl;
          LOG_ERROR(strs);
          return -2002;
        }
        xyz_calib->SetUsingCalibratedModel(true);
        if (!xyz_calib->GetPoseFromJnt(tmpJnt, &cart)) {
          strs.str("");
          strs << "xyz_calib->GetPoseFromJnt error with jnt=" << tmpJnt
               << ", in function" << __FUNCTION__ << " at line " << __LINE__
               << std::endl;
          LOG_ERROR(strs);
          return -2002;
        }
        p[j] = Vec(cart.segment(0, 3));
        pp[j] = p[j].ToEigenVec();
      }

      dv1 = pp[1] - pp[0];
      dv2 = pp[2] - pp[1];
      cN = dv1.cross(dv2);
      double norm_nd = cN.norm();
      cN /= norm_nd;                // measured normal
      if (cN(laser_channel) < 0) {  // if cN is not facing same direction as
                                    // positive Z, then reverse cN
        cN = -cN;
      }

      dv1 = pp[3] - pp[2];
      dv2 = pp[0] - pp[3];
      cN1 = dv1.cross(dv2);
      norm_nd = cN1.norm();
      cN1 /= norm_nd;
      if (cN1(laser_channel) < 0) {
        cN1 = -cN1;
      }

      dv1 = pp[2] - pp[1];
      dv2 = pp[3] - pp[2];
      cN2 = dv1.cross(dv2);
      norm_nd = cN2.norm();
      cN2 /= norm_nd;
      if (cN2(laser_channel) < 0) {
        cN2 = -cN2;
      }

      dv1 = pp[0] - pp[3];
      dv2 = pp[1] - pp[0];
      cN3 = dv1.cross(dv2);
      norm_nd = cN3.norm();
      cN3 /= norm_nd;
      if (cN3(laser_channel) < 0) {
        cN3 = -cN3;
      }

      // now doing average of the two normal
      cN = (cN + cN1 + cN2 + cN3) / 4.0;
      norm_nd = cN.norm();
      cN /= norm_nd;  // measured normal
      planeNormal.col(i) = cN;

      Frame fm2;
      Eigen::VectorXd ur_jnt_vec;
      EigenVec2StdVec(UR_jnt, &ur_jnt_vec);
      if (!ur_calib->GetDHFrame(ur_jnt_vec, 0, &fm2)) {
        strs.str("");
        strs << GetName() << "ur_calib->GetDHFrame error with jnt=" << UR_jnt
             << ", at index 1"
             << ", in function" << __FUNCTION__ << " at line " << __LINE__
             << std::endl;
        LOG_ERROR(strs);
        return -2002;
      }
      Rotation r1 = fm2.getRotation();
      Vec x1 = r1.UnitX();
      Eigen::VectorXd eigx1 = x1.ToEigenVec();

      Eigen::Vector3d meanPt = (pp[0] + pp[1] + pp[2] + pp[3]) / 4.0;
      // meanPt(2)  -= fabs(laser_scale) * laser_length;   this is commented out
      // because pp[i] has already been set to laser light length= 0 note above
      // trying to consider laser tcp with laser light length=0    ****** very
      // important ***** then 8pt user frame, also adjust pt coordinate so that
      // laser light length = 0 then in traj compensation, we set tcp(2) = -
      // |laser_scale| * (35-laser_reading) in user tool setting

      Eigen::Vector3d pred_pt = origin + a1 * eigx1;  //  + cN * v1;

      strs.str("");
      strs << "plane " << i << ", eigx1=" << eigx1 << ", cN=" << cN
           << ", meanPt=" << meanPt << ", pred_pt =" << pred_pt
           << ", actual-pt=" << meanPt << ", error=" << meanPt - pred_pt
           << ", error_proj=" << cN.dot(meanPt - pred_pt) - v1 << std::endl;

      LOG_INFO(strs);
      // now put center data into matrix, and then save into csv file
      Eigen::VectorXd cT(5);
      cT.segment(0, 3) = pred_pt + cN * v1;
      cT.segment(3, 2) = UR_jnt;
      planeCenter.col(i) = cT;
    }

    // now writing planceCenter to csv file
    RPE_Utility util;
    std::string normalFile("planeNormal.txt");
    util.WriteCSVFile(normalFile, planeNormal);
    std::string centerFile("planeCenter.txt");
    util.WriteCSVFile(centerFile, planeCenter);

    // now compute U axis direction, for now just log it
    size_t numGroups = numPlanes / 4;  // each group contains 4 planes
    Eigen::MatrixXd ptGroup(3, 4), ptGroup2(3, 4);
    Eigen::MatrixXd uAxis(3, 4), uAxis2(3, 4);
    for (size_t j = 0; j < numGroups; j++) {
      for (size_t i = 0; i < 4; i++) {
        ptGroup.col(i) = planeCenter.col(4 * i + j).segment(0, 3);
        ptGroup2.col(i) = planeCenter.col(4 * i + j).segment(0, 3);
      }

      // now computing the unit norm of ptGroup
      Eigen::VectorXd meanPt = ptGroup.rowwise().mean();
      Eigen::MatrixXd tmp = ptGroup.colwise() - meanPt;

      // using svd decompsition to find the normal of 8-pt plane
      Eigen::JacobiSVD<Eigen::MatrixXd> svd2(
          tmp.transpose(), Eigen::ComputeFullV | Eigen::ComputeFullU);
      Eigen::MatrixXd tmpV = svd2.matrixV();
      Eigen::Vector3d pnz = tmpV.col(2);
      strs.str("");
      strs << "U group" << j << ", meanPtz=" << meanPt.transpose()
           << ", pnz=" << pnz.transpose() << std::endl;
      LOG_INFO(strs);
      uAxis.col(j) = pnz;

      meanPt = ptGroup2.rowwise().mean();
      tmp = ptGroup2.colwise() - meanPt;

      // using svd decompsition to find the normal of 8-pt plane
      Eigen::JacobiSVD<Eigen::MatrixXd> svd22(
          tmp.transpose(), Eigen::ComputeFullV | Eigen::ComputeFullU);
      tmpV = svd22.matrixV();
      pnz = tmpV.col(2);
      strs.str("");
      strs << "U group using normal" << j << ", meanPtz=" << meanPt.transpose()
           << ", pnz=" << pnz.transpose() << std::endl;
      LOG_INFO(strs);
      uAxis2.col(j) = pnz;
    }
    std::string uAxisFile("uAxis.txt");
    util.WriteCSVFile(uAxisFile, uAxis);

    std::string uAxisFile2("uAxis2.txt");
    util.WriteCSVFile(uAxisFile2, uAxis2);
    Eigen::Vector3d uFinalAxis = uAxis.rowwise().mean();
    uFinalAxis /= uFinalAxis.norm();

    Eigen::Vector3d uFinalAxis2 = uAxis2.rowwise().mean();
    uFinalAxis2 /= uFinalAxis2.norm();

    strs.str("");
    strs << "uFinalAxis=" << uFinalAxis.transpose() << std::endl;
    strs << "uFinalAxis2=" << uFinalAxis2.transpose() << std::endl;
    LOG_INFO(strs);

    // now compute the zero of U axis
    // first compute the direction of all rAxis with given fixed Us
    Eigen::MatrixXd rAxis(3, 4), rAxis2(3, 4);
    Eigen::VectorXd angleRU(4), angleRU2(4);  // angles between U and R
    for (size_t j = 0; j < numGroups; j++) {
      for (size_t i = 0; i < 4; i++) {
        ptGroup.col(i) = planeCenter.col(4 * j + i).segment(0, 3);
        ptGroup2.col(i) = planeNormal.col(4 * j + i).segment(0, 3);
      }

      // now computing the unit norm of ptGroup
      Eigen::VectorXd meanPt = ptGroup.rowwise().mean();
      Eigen::MatrixXd tmp = ptGroup.colwise() - meanPt;

      // using svd decompsition to find the normal of 8-pt plane
      Eigen::JacobiSVD<Eigen::MatrixXd> svd2(
          tmp.transpose(), Eigen::ComputeFullV | Eigen::ComputeFullU);
      Eigen::MatrixXd tmpV = svd2.matrixV();
      Eigen::Vector3d pnz = tmpV.col(2);
      strs.str("");
      strs << "R group" << j << ", meanPtz=" << meanPt.transpose()
           << ", pnz=" << pnz.transpose() << std::endl;
      LOG_INFO(strs);
      rAxis.col(j) = pnz;
      angleRU(j) = acos(pnz.dot(uFinalAxis));

      meanPt = ptGroup2.rowwise().mean();
      tmp = ptGroup2.colwise() - meanPt;

      // using svd decompsition to find the normal of 8-pt plane
      Eigen::JacobiSVD<Eigen::MatrixXd> svd22(
          tmp.transpose(), Eigen::ComputeFullV | Eigen::ComputeFullU);
      tmpV = svd22.matrixV();
      pnz = tmpV.col(2);
      strs.str("");
      strs << "R group normal" << j << ", meanPtz=" << meanPt.transpose()
           << ", pnz=" << pnz.transpose() << std::endl;
      LOG_INFO(strs);
      rAxis2.col(j) = pnz;
      angleRU2(j) = acos(pnz.dot(uFinalAxis2));
    }
    std::string rAxisFile("rAxis.txt");
    util.WriteCSVFile(rAxisFile, rAxis);

    std::string rAxisFile2("rAxis2.txt");
    util.WriteCSVFile(rAxisFile2, rAxis2);

    std::string angleRUFile("angleRU.txt");
    util.WriteCSVFile(angleRUFile, angleRU);

    std::string angleRUFile2("angleRU2.txt");
    util.WriteCSVFile(angleRUFile2, angleRU2);

    Eigen::VectorXd urDH;
    ur_calib->GetCalibParamSet(&urDH);
    urDH(3) = a1;  // alpha0, alpha1, a0, a1, t0, t1, d0, d1, b0, b1
    // urDH(7) = d1;
    Eigen::VectorXd urDHVec;
    EigenVec2StdVec(urDH, &urDHVec);
    ur_calib->LoadCalibParamSet(urDHVec);  // update calibrated DH parameter

    Frame tmp_baseoff, tmp_subbaseoff;
    ur_calib->GetDefaultBaseOffFrame(&tmp_baseoff, &tmp_subbaseoff);

    // now UR mechanism set translational part of baseOffset
    // Frame tmpBase = sub_defaultBaseOff_;
    strs.str("");
    strs << GetName()
         << ": UR orig defaultSubBaseOff=" << tmp_baseoff.ToString(true)
         << std::endl;
    LOG_INFO(strs);
    Vec t(origin);
    tmp_baseoff.setTranslation(t);
    Eigen::VectorXd tmpBaseVec = tmp_baseoff.ToEigenVecQuat();
    strs.str("");
    strs << GetName() << ": UR baseoff is changed into "
         << tmp_baseoff.ToString(true)
         << ", in eigen vector=" << tmpBaseVec.transpose() << std::endl;
    LOG_INFO(strs);
    ur_calib->SetDefaultBaseOff(tmpBaseVec,
                          tmpBaseVec);  // modify the default base of UR obj

    // Vec t1(tmp_baseoff.segment(0,3));
    // Quaternion q1(tmp_baseoff(3), tmp_baseoff(4), tmp_baseoff(5),
    // tmp_baseoff(6));
    sub_defaultBaseOff_ = tmp_baseoff;  // modify sub base off of XYZUR too
    // sub_defaultBaseOff_.setQuaternion(q1);
    strs.str("");
    strs << GetName() << ": modify subdefulatBaseoff into"
         << sub_defaultBaseOff_.ToString(true)
         << ", while defualtbaseoff is still " << defaultBaseOff_.ToString(true)
         << std::endl;
    LOG_INFO(strs);

    ur_calib->updateCalibrateStatus(true);
    if (xyz_calib->isCalibrated()) {
      isDHCalibrated_ = true;  // set entire mechanism is calibrated
    }
    // sub_defaultBaseOff_ = tmpBase;  // modify the default subBase of XYZ_UR
    // obj, defaultBase for XYZ is not changed
    return 0;
  } catch (...) {
    strs.str("");
    strs << GetName() << " gets exception in function " << __FUNCTION__
         << std::endl;
    LOG_ERROR(strs);
    return -1;
  }
}

void XyzUrCalib::SetDefaultBaseOff(const EigenDRef<Eigen::VectorXd> &baseoff,
                               const EigenDRef<Eigen::VectorXd> &subbaseoff) {
  std::ostringstream strs;
  if (!initialized_) {
    strs.str("");
    strs << GetName() << ":"
         << "XYZ_UR geometric parameters are not initialized"
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return;
  }
  ur_calib->SetDefaultBaseOff(subbaseoff, subbaseoff);
  strs.str("");
  strs << GetName() << ": UR Set defaultBaesoff=" << subbaseoff.transpose()
       << std::endl;

  xyz_calib->SetDefaultBaseOff(baseoff, baseoff);
  strs.str("");
  strs << GetName() << ": XYZ Set defaultBaesoff=" << baseoff.transpose()
       << std::endl;
  // also set itself
  BaseKinematicMap::SetDefaultBaseOff(baseoff, subbaseoff);
}

void XyzUrCalib::SetDefaultBaseOff(const Eigen::VectorXd &baseoff,
                               const Eigen::VectorXd &subbaseoff) {
  std::ostringstream strs;
  if (!initialized_) {
    strs.str("");
    strs << GetName() << ":"
         << "XYZ_UR geometric parameters are not initialized"
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return;
  }
  ur_calib->SetDefaultBaseOff(subbaseoff, subbaseoff);
  strs.str("");
  strs << GetName() << ": UR Set defaultBaesoff=" << subbaseoff.transpose()
       << std::endl;

  xyz_calib->SetDefaultBaseOff(baseoff, baseoff);
  strs.str("");
  strs << GetName() << ": XYZ Set defaultBaesoff=" << baseoff.transpose()
       << std::endl;
  // also set itself
  BaseKinematicMap::SetDefaultBaseOff(baseoff, subbaseoff);
}

void XyzUrCalib::GetDefaultBaseOff(EigenDRef<Eigen::VectorXd> *baseoff,
                               EigenDRef<Eigen::VectorXd> *subbaseoff) {
  std::ostringstream strs;
  if (!initialized_) {
    strs.str("");
    strs << GetName() << ":"
         << "XYZ_UR geometric parameters are not initialized"
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return;
  }
  Eigen::VectorXd tmp_baseoff, tmp_subbaseoff;
  xyz_calib->GetDefaultBaseOff(baseoff, subbaseoff);
  tmp_baseoff = *baseoff;
  ur_calib->GetDefaultBaseOff(baseoff, subbaseoff);
  tmp_subbaseoff = *baseoff;
  *baseoff = tmp_baseoff;
  *subbaseoff = tmp_subbaseoff;
  strs.str("");
  strs << GetName() << ": baseoff=" << *baseoff
       << ", subbaseoff=" << *subbaseoff << std::endl;
  LOG_INFO(strs);
}

int XyzUrCalib::SetDependJacobianColumns(const std::vector<size_t> &d_cols) {
  std::ostringstream strs;
  if (!initialized_) {
    strs.str("");
    strs << GetName() << ":"
         << "XYZ_UR geometric parameters are not initialized"
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return -1;
  }
  std::vector<size_t> d_col_base, d_col_XYZ, d_col_UR, d_col_matA_XYZ,
      d_col_matA_UR;
  for (size_t i = 0; i < d_cols.size(); i++) {
    size_t col = d_cols[i];
    if (col < 6) {  // base
      d_col_base.push_back(col);
    } else if (col < 6 + xyz_calib->GetDoF() * 5) {
      d_col_XYZ.push_back(col);
    } else if (col < 6 + DoF_ * 5) {
      d_col_UR.push_back(col - xyz_calib->GetDoF() * 5);
    } else {
      d_col_matA_XYZ.push_back(col - ur_calib->GetDoF() * 5);
      d_col_matA_UR.push_back(col - xyz_calib->GetDoF() * 5);
    }
  }

  std::vector<size_t> d_XYZ, d_UR;
  d_XYZ.insert(d_XYZ.end(), d_col_base.begin(), d_col_base.end());
  d_XYZ.insert(d_XYZ.end(), d_col_XYZ.begin(), d_col_XYZ.end());
  d_XYZ.insert(d_XYZ.end(), d_col_matA_XYZ.begin(), d_col_matA_XYZ.end());

  // d_col_base.clear(); // UR needs to update base
  d_UR.insert(d_UR.end(), d_col_base.begin(), d_col_base.end());
  d_UR.insert(d_UR.end(), d_col_UR.begin(), d_col_UR.end());
  d_UR.insert(d_UR.end(), d_col_matA_UR.begin(), d_col_matA_UR.end());

  // logging d_XYZ, and d_UR
  // Eigen::VectorXd vec_XYZ, vec_UR;
  // StdVec2EigenVec(d_XYZ, &vec_XYZ);
  // StdVec2EigenVec(d_UR, &vec_UR);
  /*
  strs.str("");
  strs << "dep_col_XYZ=";
  for (size_t i=0; i < d_XYZ.size(); i++) {
      strs << d_XYZ[i] << ",";
  }
  strs <<", dep_col_UR=";
  for (size_t i=0; i < d_UR.size(); i++) {
      strs << d_UR[i] << ",";
  }
  strs << std::endl;
  LOG_INFO(strs);
  */
  xyz_calib->SetDependJacobianColumns(d_XYZ);
  ur_calib->SetDependJacobianColumns(d_UR);
  return 0;
}

bool XyzUrCalib::GetCalibParamSet(EigenDRef<Eigen::VectorXd> *cal_DH) {
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
  Eigen::VectorXd cal_DH_XYZ(5 * dof_XYZ + 14);
  Eigen::VectorXd cal_DH_UR(5 * dof_UR + 14);
  if (!xyz_calib->GetCalibParamSet(&cal_DH_XYZ)) {
    return false;
  }
  if (!ur_calib->GetCalibParamSet(&cal_DH_UR)) {
    return false;
  }
  Eigen::VectorXd alpha(DoF_), a(DoF_), theta(DoF_), d(DoF_), beta(DoF_);

  for (size_t i = 0; i < dof_XYZ; i++) {
    alpha[i] = cal_DH_XYZ[i];
    a[i] = cal_DH_XYZ[dof_XYZ + i];
    theta[i] = cal_DH_XYZ[2 * dof_XYZ + i];
    d[i] = cal_DH_XYZ[3 * dof_XYZ + i];
    beta[i] = cal_DH_XYZ[4 * dof_XYZ + i];
  }

  for (size_t i = 0; i < dof_UR; i++) {
    alpha[dof_XYZ + i] = cal_DH_UR[i];
    a[dof_XYZ + i] = cal_DH_UR[dof_UR + i];
    theta[dof_XYZ + i] = cal_DH_UR[2 * dof_UR + i];
    d[dof_XYZ + i] = cal_DH_UR[3 * dof_UR + i];
    beta[dof_XYZ + i] = cal_DH_UR[4 * dof_UR + i];
  }

  if (cal_DH->size() < 5 * DoF_ + 14) {
    strs.str("");
    strs << GetName() << ":"
         << "The input vector pointer has wrong dimension in function "
         << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  Eigen::VectorXd tmp;
  tmp.insert(tmp.end(), alpha.begin(), alpha.end());
  tmp.insert(tmp.end(), a.begin(), a.end());
  tmp.insert(tmp.end(), theta.begin(), theta.end());
  tmp.insert(tmp.end(), d.begin(), d.end());
  tmp.insert(tmp.end(), beta.begin(), beta.end());
  for (size_t i = 0; i < 5 * DoF_; i++) {
    (*cal_DH)(i) = tmp[i];
  }
  // get defaultBaseoff and subdefaultBaseoff
  cal_DH->segment(5 * DoF_, 7) = cal_DH_XYZ.segment(5 * dof_XYZ, 7);
  cal_DH->segment(5 * DoF_ + 7, 7) = cal_DH_UR.segment(5 * dof_UR, 7);
  strs.str("");
  strs << GetName() << ": GetCalibPram success" << std::endl;
  LOG_INFO(strs);
  return true;
}

bool XyzUrCalib::LoadCalibParamSet(const Eigen::VectorXd &kine_para) {
  std::ostringstream strs;
  if (!initialized_) {
    strs.str("");
    strs << GetName() << " geometric parameters are not initialized"
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  Eigen::VectorXd alpha(DoF_), a(DoF_), theta(DoF_), d(DoF_), beta(DoF_);
  for (size_t i = 0; i < DoF_; i++) {
    alpha[i] = kine_para[i];
    a[i] = kine_para[DoF_ + i];
    theta[i] = kine_para[2 * DoF_ + i];
    d[i] = kine_para[3 * DoF_ + i];
    beta[i] = kine_para[4 * DoF_ + i];
  }
  Eigen::VectorXd dh_XYZ, dh_UR;
  dh_XYZ.insert(dh_XYZ.end(), alpha.begin(), alpha.begin() + 3);
  dh_XYZ.insert(dh_XYZ.end(), a.begin(), a.begin() + 3);
  dh_XYZ.insert(dh_XYZ.end(), theta.begin(), theta.begin() + 3);
  dh_XYZ.insert(dh_XYZ.end(), d.begin(), d.begin() + 3);
  dh_XYZ.insert(dh_XYZ.end(), beta.begin(), beta.begin() + 3);
  dh_XYZ.insert(dh_XYZ.end(), kine_para.begin() + 5 * DoF_,
                kine_para.begin() + 5 * DoF_ + 7);
  dh_XYZ.insert(dh_XYZ.end(), kine_para.begin() + 5 * DoF_,
                kine_para.begin() + 5 * DoF_ + 7);
  dh_UR.insert(dh_UR.end(), alpha.begin() + 3, alpha.end());
  dh_UR.insert(dh_UR.end(), a.begin() + 3, a.end());
  dh_UR.insert(dh_UR.end(), theta.begin() + 3, theta.end());
  dh_UR.insert(dh_UR.end(), d.begin() + 3, d.end());
  dh_UR.insert(dh_UR.end(), beta.begin() + 3, beta.end());
  dh_UR.insert(dh_UR.end(), kine_para.begin() + 5 * DoF_ + 7, kine_para.end());
  dh_UR.insert(dh_UR.end(), kine_para.begin() + 5 * DoF_ + 7, kine_para.end());
  if (xyz_calib->LoadCalibParamSet(dh_XYZ) && ur_calib->LoadCalibParamSet(dh_UR)) {
    this->isDHCalibrated_ = true;
  }
  return true;
}

void XyzUrCalib::SetUsingCalibratedModel(bool useCalibratedModel) {
  std::ostringstream strs;
  if (!initialized_) {
    strs.str("");
    strs << GetName() << " geometric parameters are not initialized"
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return;
  }
  xyz_calib->SetUsingCalibratedModel(useCalibratedModel);
  ur_calib->SetUsingCalibratedModel(useCalibratedModel);
  useCalibrated_ = useCalibratedModel;
}

//! has robot been calibrated
bool XyzUrCalib::isCalibrated() {
  std::ostringstream strs;
  if (!initialized_) {
    strs.str("");
    strs << GetName() << " geometric parameters are not initialized"
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  // std::cout << "XYZ clibrated: "  << xyz_calib->isCalibrated() << ", UR calibrated:
  // " << ur_calib->isCalibrated()  << std::endl; std::cout << GetName() << ":
  // isCalibrated() =" << isDHCalibrated_ << std::endl;
  return isDHCalibrated_;
}

//! is parameter initialized
bool XyzUrCalib::isInitialized() const {
  if (!initialized_) {
    return false;
  }
  return xyz_calib->isInitialized() && ur_calib->isInitialized();
}
//! reset calibration model
bool XyzUrCalib::resetCalibration() {
  std::ostringstream strs;
  if (!initialized_) {
    strs.str("");
    strs << GetName() << " geometric parameters are not initialized"
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  isDHCalibrated_ = false;
  return xyz_calib->resetCalibration() && ur_calib->resetCalibration();
}

}  // namespace kinematics_lib