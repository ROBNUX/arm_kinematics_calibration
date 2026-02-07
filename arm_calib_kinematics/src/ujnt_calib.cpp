#include "arm_calib_kinematics/ujnt_calib.hpp"

#include "robnux_kdl_common/pose.hpp"

// register plugin
PLUGINLIB_EXPORT_CLASS(kinematics_lib::UjntCalib,
                       kinematics_lib::BaseKinematicMap)
PLUGINLIB_EXPORT_CLASS(kinematics_lib::UjntCalib,
                       kinematics_lib::BaseCalibration)

namespace kinematics_lib {

UjntCalib::UjntCalib() : SerialArmCalib(2) {}

UjntCalib::UjntCalib(const Eigen::VectorXd &kine_para)
    : SerialArmCalib(kine_para) {}

int UjntCalib::CartToJnt(const Pose &pos, Eigen::VectorXd *q) {
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
    strs << "UJNT geometric parameters are not initialized"
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_PARAM_NOT_INITIALIZED;
  }

  std::vector<int> branch;
  pos.getBranchFlags(&branch);
  if (branch.empty()) {
    strs.str("");
    strs << "input pose has no branch information"
         << " in " << __FUNCTION__ << " at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_NO_BRANCH_INFO;
  }

  // r.GetEulerZYX(&yaw, &pitch, &roll);
  if (q->size() != DoF_) {
    q->resize(DoF_);
  }

  // get default frame
  Frame tip;
  pos.getFrame(&tip);

  // get relative frame of tip w.r.t. robot base
  Frame relTip = defaultBaseOff_.Inverse() * tip;
  Rotation r = relTip.getRotation();

  Vec rx = r.UnitX();  // we solve yaw and pitch from rx
  Vec z1(0, 0, 1);

  double calpha = rx.dot(z1);
  double yaw, pitch;
  pitch = acos(calpha) - M_PI / 2.0;

  Vec rx_proj = rx - calpha * z1;
  yaw = atan2(rx_proj.y(), rx_proj.x());
  if (branch[2]) {  // nonflip
    (*q)(0) = yaw;
    (*q)(1) = -pitch;
  } else {  // flip solution
    (*q)(1) = -(M_PI - pitch);
    if (yaw >= 0) {
      (*q)(0) = yaw - M_PI;
    } else {
      (*q)(0) = yaw + M_PI;
    }
  }

  std::vector<int> jointTurns;
  pos.getJointTurns(&jointTurns);

  for (size_t i = 0; i < DoF_; i++) {
    // note sometimes, joint 0 could be running more than [-pi, pi]
    double turn = std::floor((*q)(i) / (2 * M_PI));
    double tmp_q = (*q)(i)-turn * 2 * M_PI;
    // then make sure [-PI, PI]
    // to have 0 turn here
    if (tmp_q > M_PI) {
      turn += 1;
    }
    (*q)(i) += (jointTurns[i] - turn) * 2 * M_PI;
    // subtract theta offset
    (*q)(i) = ((*q)(i)-theta_[i]) / pitch_(i);
  }
  return 0;
}

int UjntCalib::JntToCart(const Eigen::VectorXd &q, Pose *p) {
  int ret = SerialArmCalib::JntToCart(q, p);
  if (ret < 0) {
    return ret;
  }

  // multiply this to make sure  the final rotation
  // is R_x(alpha[0])R_z(yaw)R_y(pitch) for easy IK
  Rotation r = Rotation::RotX(-M_PI / 2.0);
  Frame tmp1;
  tmp1.setRotation(r);
  Frame tmp;
  p->getFrame(&tmp);
  tmp = tmp * tmp1;
  p->setFrame(tmp);
  return 0;
}

int UjntCalib::CalcJacobian(const Eigen::VectorXd &kine_para, Pose *p,
                            Eigen::MatrixXd *Jp_t, Eigen::MatrixXd *Jp_r,
                            const bool reduction) {
  int ret = SerialArmCalib::CalcJacobian(kine_para, p, Jp_t, Jp_r, reduction);
  if (ret < 0) {
    return ret;
  }
  Rotation r = Rotation::RotX(-M_PI / 2.0);
  Frame tmp1;
  tmp1.setRotation(r);
  Frame tmp;
  p->getFrame(&tmp);
  tmp = tmp * tmp1;
  p->setFrame(tmp);
  return 0;
}

bool UjntCalib::PickRotSubJacobian(const Eigen::MatrixXd &Jp_t,
                                   const Eigen::MatrixXd &Jp_r,
                                   Eigen::MatrixXd *Js_t,
                                   Eigen::MatrixXd *Js_r) {
  std::ostringstream strs;
  if (!Js_t || !Js_r) {
    strs.str("");
    strs << GetName() << " input pointer is null"
         << " in " << __FUNCTION__ << " at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  size_t row_r = Jp_r.rows();
  size_t col_r = Jp_r.cols();

  if (row_r < 3 || col_r < DoF_) {
    strs.str("");
    strs << GetName() << " input Jacobian matrices have wrong dimension "
         << " in " << __FUNCTION__ << " at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return false;
  }

  Js_r->resize(3, DoF_);
  for (size_t i = 0; i < DoF_; i++) {
    (*Js_r).col(i) = Jp_r.col(5 * i + 2) * pitch_(i);
  }
  return true;
}

// nominal velocity IK
int UjntCalib::CartToJnt(const Pose &p, const Twist &v, Eigen::VectorXd *q,
                         Eigen::VectorXd *qdot) {
  std::ostringstream strs;
  if (!q || !qdot) {
    strs.str("");
    strs << "Input q and qdot vectors are null"
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
  // this analytic IK is using canonical robot model, for non-canonical
  // model (e.g., model after calibration) requires using GI-based iteration
  // method
  if (useCalibrated_) {
    strs.str("");
    strs << "default CartToJnt function must using canonical kinematic"
            " model in"
         << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_DEFAULT_IK_NOT_CANO;
  }
  int ret = CartToJnt(p, q);
  if (ret < 0) {
    return ret;
  }
  if (qdot->size() != DoF_) {
    qdot->resize(DoF_);
  }

  // need to compute the Jacobian
  Eigen::VectorXd d_tmp, theta_tmp;
  d_tmp = d_;
  theta_tmp = theta_;
  UpdateDH(*q, &theta_tmp, &d_tmp);
  Eigen::VectorXd kine_para(5 * DoF_);
  kine_para.segment(0, DoF_) = alpha_;
  kine_para.segment(DoF_, DoF_) = a_;
  kine_para.segment(2 * DoF_, DoF_) = theta_tmp;
  kine_para.segment(3 * DoF_, DoF_) = d_tmp;
  kine_para.segment(4 * DoF_, DoF_) = beta_;

  Twist v1 = defaultBaseOff_.Inverse() * v;

  Pose tmp_p;
  Eigen::MatrixXd Jp_t, Jp_r;
  ret = CalcJacobian(kine_para, &tmp_p, &Jp_t, &Jp_r, true);
  if (ret < 0) {
    return ret;
  }
  // picking submatrix of Jp_t and Jp_r, and then multiplying qdot
  // to obtain twist
  Eigen::MatrixXd M;
  Eigen::VectorXd b;
  Eigen::MatrixXd Js_t, Js_r;
  PickRotSubJacobian(Jp_t, Jp_r, &Js_t, &Js_r);

  size_t rowTrans = Js_t.rows();
  size_t rowRot = Js_r.rows();
  M.resize(rowTrans + rowRot, rowTrans + rowRot);
  b.resize(rowTrans + rowRot);
  if (rowTrans > 0) {
    M.block(0, 0, rowTrans, rowTrans + rowRot) = Js_t;
  }
  if (rowRot > 0) {
    M.block(rowTrans, 0, rowRot, rowTrans + rowRot) = Js_r;
  }
  Eigen::MatrixXd MTM = M.transpose() * M;
  double det = MTM.determinant();
  if (fabs(det) < K_EPSILON) {
    std::ostringstream strs;
    strs << GetName() << " is singular, can not compute IK "
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    return -ERR_ROB_JACOBIAN_IK_SINGULAR;
  }

  Vec t_v = v1.getLinearVel();
  Vec t_w = v1.getAngularVel();

  double spdNorm = PickRotCartErr(t_v.ToEigenVec(), t_w.ToEigenVec(), &b);
  *qdot = MTM.inverse() * M.transpose() * b;
  return 0;
}

void UjntCalib::UpdateConfigTurn(const Eigen::VectorXd &theta,
                                 const Eigen::VectorXd &d,
                                 std::vector<int> *branchFlags,
                                 std::vector<int> *jointTurns) const {
  std::ostringstream strs;
  if (!branchFlags || !jointTurns) {
    strs.str("");
    strs << GetName() << "Input branchFlags and jointTurns are null"
         << " in " << __FUNCTION__ << " at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return;
  }
  branchFlags->resize(3, eBranchLeft);
  Rotation r = Rotation::RotY(theta[1]);
  Vec v = r.UnitX();
  if (v.x() >= 0) {  //  non-flip  (right half circle, in zx plane)
    branchFlags->at(2) = eBranchRight;
  } else {  // flip (or left half circle in zx plane)
    branchFlags->at(2) = eBranchLeft;
  }
  // DoF turn flags, shall all be 0, because it is prismatic
  jointTurns->resize(DoF_, 0);
  double tmp_q = 0;
  for (size_t i = 0; i < DoF_; i++) {
    jointTurns->at(i) = std::floor(theta[i] / (2 * M_PI));
    tmp_q = theta[i] - jointTurns->at(i) * 2 * M_PI;
    // then make sure [-PI, PI]
    // to have 0 turn here
    if (tmp_q > M_PI) {
      jointTurns->at(i) += 1;
    }
  }
}

// using ballbar
bool UjntCalib::PickSubJacobian(const Eigen::MatrixXd &Jp_t,
                                const Eigen::MatrixXd &Jp_r,
                                Eigen::MatrixXd *Js_t, Eigen::MatrixXd *Js_r,
                                const bool reduction) {
  std::ostringstream strs;
  if (!Js_t || !Js_r) {
    strs.str("");
    strs << GetName() << " input pointer is null"
         << " in " << __FUNCTION__ << " at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  size_t row_t = Jp_t.rows();
  size_t col_t = Jp_t.cols();
  size_t row_r = Jp_r.rows();
  size_t col_r = Jp_r.cols();

  if (row_t < 3 || col_t < DoF_ || row_r < 3 || col_r < DoF_) {
    strs.str("");
    strs << GetName() << " input Jacobian matrices have wrong dimension "
         << " in " << __FUNCTION__ << " at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return false;
  }

  Js_t->resize(3, DoF_);
  for (size_t i = 0; i < DoF_; i++) {
    (*Js_t).col(i) = Jp_t.col(5 * i + 2) * pitch_(i);
  }
  Js_r->resize(3, DoF_);
  for (size_t i = 0; i < DoF_; i++) {
    (*Js_r).col(i) = Jp_r.col(5 * i + 2) * pitch_(i);
  }
  return true;
}

// given trans and euler angle error, pick a sub error vector matching
// robot model, and return the aboslute error norm (using ballbar)
double UjntCalib::PickCartErr(const Eigen::Vector3d &errT,
                              const Eigen::Vector3d &errR, Eigen::VectorXd *b,
                              const bool reduction) {
  std::ostringstream strs;
  double val = errT.norm();
  if (!b) {
    strs.str("");
    strs << GetName() << " input pointer is null"
         << " in " << __FUNCTION__ << " at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
  }
  b->resize(3);
  *b = errT;
  return val;
}

double UjntCalib::PickRotCartErr(const Eigen::Vector3d &errT,
                                 const Eigen::Vector3d &errR,
                                 Eigen::VectorXd *b) {
  std::ostringstream strs;
  double val = errR.norm();
  if (!b) {
    strs.str("");
    strs << GetName() << " input pointer is null"
         << " in " << __FUNCTION__ << " at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
  }
  b->resize(3);
  *b = errR;
  return val;
}

//! virtual function for updating actual DH parameters based upon joint feedback
// Note: alpha, a, theta, d, beta has their initial values, which will be
// updated based upon jnt input
void UjntCalib::UpdateDH(const Eigen::VectorXd &jnt, Eigen::VectorXd *theta,
                         Eigen::VectorXd *d) const {
  std::ostringstream strs;
  if (!theta || !d) {
    strs.str("");
    strs << GetName() << " input pointer is null"
         << " in " << __FUNCTION__ << " at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
  }
  if (jnt.size() < DoF_ || theta->size() != DoF_ || d->size() != DoF_) {
    strs.str("");
    strs << GetName() << " input parameters have wrong size "
         << " in " << __FUNCTION__ << " at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
  }

  for (size_t i = 0; i < DoF_; i++) {
    (*theta)(i) += jnt(i) * pitch_(i);
  }
}

bool UjntCalib::PickSubJacobianForPara(const Eigen::MatrixXd &Jp_t,
                                       const Eigen::MatrixXd &Jp_r,
                                       Eigen::MatrixXd *Js_t1,
                                       Eigen::MatrixXd *Js_r1,
                                       const bool reduction) {
  std::ostringstream strs;
  if (!Js_t1 || !Js_r1) {
    strs.str("");
    strs << GetName() << " input pointer is null"
         << " in " << __FUNCTION__ << " at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  size_t row_t = Jp_t.rows();
  size_t col_t = Jp_t.cols();
  size_t row_r = Jp_r.rows();
  size_t col_r = Jp_r.cols();

  Js_t1->resize(row_t, col_t);
  *Js_t1 = Jp_t;
  for (size_t i = 0; i < DoF_; i++) {
    (*Js_t1).col(5 * i + 2) = Jp_t.col(5 * i + 2) * pitch_(i);
  }
  Js_r1->resize(row_r, col_r);
  *Js_r1 = Jp_r;
  for (size_t i = 0; i < DoF_; i++) {
    (*Js_r1).col(5 * i + 2) = Jp_r.col(5 * i + 2) * pitch_(i);
  }
  return true;
}

bool UjntCalib::PickRotSubJacobianForPara(const Eigen::MatrixXd &Jp_t,
                                          const Eigen::MatrixXd &Jp_r,
                                          Eigen::MatrixXd *Js_t1,
                                          Eigen::MatrixXd *Js_r1) {
  std::ostringstream strs;
  if (!Js_t1 || !Js_r1) {
    strs.str("");
    strs << GetName() << " input pointer is null"
         << " in " << __FUNCTION__ << " at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  size_t row_r = Jp_r.rows();
  size_t col_r = Jp_r.cols();
  Js_r1->resize(row_r, col_r);
  *Js_r1 = Jp_r;
  for (size_t i = 0; i < DoF_; i++) {
    (*Js_r1).col(5 * i + 2) = Jp_r.col(5 * i + 2) * pitch_(i);
  }
  return true;
}

void UjntCalib::UpdateDH(const Eigen::VectorXd &orig_dh,
                         const Eigen::VectorXd &jnt,
                         Eigen::VectorXd *new_dh) const {
  std::ostringstream strs;
  if (!new_dh) {
    strs.str("");
    strs << GetName() << " input pointer is null"
         << " in " << __FUNCTION__ << " at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
  }
  if (orig_dh.size() != 5 * DoF_ || jnt.size() < DoF_) {
    strs.str("");
    strs << GetName() << " input parameters have wrong size, origin dh size= "
         << orig_dh.size() << ", jnt size =" << jnt.size() << " in "
         << __FUNCTION__ << " at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
  }
  *new_dh = orig_dh;
  for (size_t i = 0; i < DoF_; i++) {
    (*new_dh)(2 * DoF_ + i) = orig_dh(2 * DoF_ + i) + jnt(i) * pitch_(i);
  }
}

}  // namespace kinematics_lib