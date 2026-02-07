#include "arm_calib_kinematics/scara_calib.hpp"

#include "robnux_kdl_common/pose.hpp"

// register plugin
PLUGINLIB_EXPORT_CLASS(kinematics_lib::ScaraCalib,
                       kinematics_lib::BaseKinematicMap)
PLUGINLIB_EXPORT_CLASS(kinematics_lib::ScaraCalib,
                       kinematics_lib::BaseCalibration)

namespace kinematics_lib {

ScaraCalib::ScaraCalib() : SerialArmCalib(4) {}

ScaraCalib::ScaraCalib(const Eigen::VectorXd &kine_para)
    : SerialArmCalib(kine_para) {}

int ScaraCalib::CartToJnt(const Pose &ps, Eigen::VectorXd *q) {
  if (!q) {
    std::cout << GetName() << ":input joint angle pointer is null in "
              << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
    return -15;
  }
  if (!initialized_) {
    std::cout << GetName() << ":geometric parameters are not initialized"
              << " in function " << __FUNCTION__ << ", line " << __LINE__
              << std::endl;
    return -16;
  }
  Vec p = ps.getTranslation();
  Quaternion quat = ps.getQuaternion();

  std::vector<int> branch;
  ps.getBranchFlags(&branch);
  if (branch.empty()) {
    std::cout << GetName() << ":input pose has no branch information"
              << " in " << __FUNCTION__ << " at line " << __LINE__ << std::endl;
    return -49;
  }
  if (q->size() != DoF_) {
    q->resize(DoF_);
  }
  // compute prismatic joint value
  (*q)(2) = p.z() - (d_(0) + d_(1) + d_(3));
  double xy_length = sqrt(p.x() * p.x() + p.y() * p.y());
  if (xy_length > a_(1) + a_(2) ||
      xy_length < fabs(a_(1) - a_(2))) {  // then out of reach
    std::cout << GetName()
              << ":exception CartToJnt, scara, IK fails due to desire pose out"
              << " of reach in " << __FUNCTION__ << " at line " << __LINE__
              << std::endl;
    return -50;
  }
  // the orientation angle from origin to the origin of tool frame
  // in the XY-plane
  double angle_xy = atan2(p.y(), p.x());
  // angle between upper link and the vector (p.(x), p.(y))
  double offset_angle =
      acos((a_(1) * a_(1) + xy_length * xy_length - a_(2) * a_(2)) /
           (2 * a_(1) * xy_length));

  if (branch[0]) {  // if the configuration is righty
    (*q)(0) = angle_xy - offset_angle;
  } else {
    (*q)(0) = angle_xy + offset_angle;
  }
  (*q)(1) = atan2(p.y() - a_(1) * sin((*q)(0)), p.x() - a_(1) * cos((*q)(0))) -
            (*q)(0);

  double yaw, pitch, roll;
  if (!quat.GetEulerZYX(&yaw, &pitch, &roll)) {
    std::cout << "exception, IK fails due to Quaternion to YPR error"
              << " in " << __FUNCTION__ << " at line " << __LINE__ << std::endl;
    return -51;
  }
  (*q)(3) = yaw - (*q)(0) - (*q)(1);

  std::vector<int> jointTurns;
  ps.getBranchFlags(&jointTurns);
  for (size_t i = 0; i < DoF_; i++) {
    if (i != 2) {
      // first convert into [-pi, pi]
      double jointTurn = std::floor((*q)(i) / (2 * M_PI));
      double tmp_q = (*q)(i)-jointTurn * 2 * M_PI;
      // then make sure [-PI, PI]
      // to have 0 turn here
      if (tmp_q > M_PI) {
        jointTurn += 1;
      }
      // taken into account the desired turn
      (*q)(i) += (jointTurns[i] - jointTurn) * 2 * M_PI;
      // subtract theta offset
      (*q)(i) = ((*q)(i)-theta_[i]) / pitch_(i);
    } else {
      (*q)(i) = ((*q)(i)-d_[i]) / pitch_(i);
    }
  }
  return 0;
}

void ScaraCalib::UpdateConfigTurn(const Eigen::VectorXd &theta,
                                  const Eigen::VectorXd &d,
                                  std::vector<int> *branchFlags,
                                  std::vector<int> *jointTurns) const {
  std::ostringstream strs;
  if (!branchFlags || !jointTurns) {
    strs.str("");
    strs << "Input branchFlags and jointTurns are null"
         << " in " << __FUNCTION__ << " at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return;
  }
  // for scara, there is only 1 branch flag: elbow (up or down)
  branchFlags->resize(3, eBranchLeft);
  // 4 turn flags, actually only 3 turn flags (because joint 3 is prismatic
  jointTurns->resize(4, 0);
  Eigen::VectorXd q_tmp = theta;
  q_tmp[2] = d[2];
  double tmp_q = 0;
  for (size_t i = 0; i < DoF_; i++) {
    if (i != 2) {
      jointTurns->at(i) = std::floor(q_tmp[i] / (2 * M_PI));
      tmp_q = q_tmp[i] - jointTurns->at(i) * 2 * M_PI;
      // then make sure [-PI, PI]
      // to have 0 turn here
      if (tmp_q > M_PI) {
        jointTurns->at(i) += 1;
      }
    }
    if (i == 1) {
      if (tmp_q >= 0 && tmp_q < M_PI) {
        branchFlags->at(0) = eBranchRight;  // righty
      } else {
        branchFlags->at(0) = eBranchLeft;  // lefty
      }
    }
  }
}

bool ScaraCalib::PickSubJacobian(const Eigen::MatrixXd &Jp_t,
                                 const Eigen::MatrixXd &Jp_r,
                                 Eigen::MatrixXd *Js_t, Eigen::MatrixXd *Js_r,
                                 const bool reduction) {
  std::ostringstream strs;
  if (!Js_t || !Js_r) {
    strs.str("");
    strs << GetName() << ": input pointer is null"
         << " in " << __FUNCTION__ << " at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  size_t row_t = Jp_t.rows();
  size_t col_t = Jp_t.cols();
  size_t row_r = Jp_r.rows();
  size_t col_r = Jp_r.cols();
  if (row_t < 3 || col_t < 4 || row_r < 3 || col_r < 4) {
    strs.str("");
    strs << GetName() << " input Jacobian matrices have wrong dimension "
         << " in " << __FUNCTION__ << " at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return false;
  }

  Js_t->resize(3, 4);
  if (reduction) {
    Js_r->resize(1, 4);
  } else {
    Js_r->resize(3, 4);
  }
  for (size_t i = 0; i < DoF_; i++) {
    if (i != 2) {
      Js_t->col(i) = Jp_t.col(5 * i + 2) * pitch_(i);
      if (reduction) {
        (*Js_r)(i) = Jp_r(2, 5 * i + 2) * pitch_(i);
      } else {
        Js_r->col(i) = Jp_r.col(5 * i + 2) * pitch_(i);
      }
    } else {  // for d[2] parameter
      Js_t->col(i) = Jp_t.col(5 * i + 3) * pitch_(i);
      if (reduction) {
        (*Js_r)(i) = Jp_r(2, 5 * i + 3) * pitch_(i);
      } else {
        Js_r->col(i) = Jp_r.col(5 * i + 3) * pitch_(i);
      }
    }
  }
  return true;
}

bool ScaraCalib::PickSubJacobianForPara(const Eigen::MatrixXd &Jp_t,
                                        const Eigen::MatrixXd &Jp_r,
                                        Eigen::MatrixXd *Js_t1,
                                        Eigen::MatrixXd *Js_r1,
                                        const bool reduction) {
  std::ostringstream strs;
  if (!Js_t1 || !Js_r1) {
    strs.str("");
    strs << GetName() << ": input pointer is null"
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

  if (reduction) {
    Js_r1->resize(1, col_r);
    Js_r1->row(0) = Jp_r.row(2);
  } else {
    Js_r1->resize(row_r, col_r);
    *Js_r1 = Jp_r;
  }

  for (size_t i = 0; i < DoF_; i++) {
    if (i != 2) {
      Js_t1->col(5 * i + 2) *= pitch_(i);
      Js_r1->col(5 * i + 2) *= pitch_(i);
    } else {  // for d[2] parameter
      Js_t1->col(5 * i + 3) *= pitch_(i);
      Js_r1->col(5 * i + 3) *= pitch_(i);
    }
  }
  return true;
}

// given trans and euler angle error, pick a sub error vector matching
// robot model, and return the aboslute error norm
double ScaraCalib::PickCartErr(const Eigen::Vector3d &errT,
                               const Eigen::Vector3d &errR, Eigen::VectorXd *b,
                               const bool reduction) {
  std::ostringstream strs;
  if (!b) {
    strs.str("");
    strs << " input pointer is null"
         << " in " << __FUNCTION__ << " at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
  }
  double val;
  if (reduction) {
    b->resize(4);
    b->block(0, 0, 3, 1) = errT;
    (*b)(3) = errR(2);
    val = errT.norm() + fabs(errR(2));
  } else {
    b->resize(6);
    b->block(0, 0, 3, 1) = errT;
    b->block(3, 0, 3, 1) = errR;
    val = errT.norm() + errT.norm();
  }
  return val;
}

void ScaraCalib::UpdateDH(const Eigen::VectorXd &orig_dh,
                          const Eigen::VectorXd &jnt,
                          Eigen::VectorXd *new_dh) const {
  std::ostringstream strs;
  if (!new_dh) {
    strs.str("");
    strs << " input pointer is null"
         << " in " << __FUNCTION__ << " at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
  }
  if (orig_dh.size() != 5 * DoF_ || jnt.size() < DoF_) {
    strs.str("");
    strs << " input parameters have wrong size, origin dh size= "
         << orig_dh.size() << ", jnt size =" << jnt.size() << " in "
         << __FUNCTION__ << " at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
  }
  *new_dh = orig_dh;
  (*new_dh)(8) = orig_dh(8) + jnt(0) * pitch_(0);
  (*new_dh)(9) = orig_dh(9) + jnt(1) * pitch_(1);
  (*new_dh)(11) = orig_dh(11) + jnt(3) * pitch_(3);
  (*new_dh)(14) = orig_dh(14) + jnt(2) * pitch_(2);
}

//! virtual function for updating actual DH parameters based upon joint feedback
// Note: alpha, a, theta, d, beta has their initial values, which will be
// updated based upon jnt input
void ScaraCalib::UpdateDH(const Eigen::VectorXd &jnt, Eigen::VectorXd *theta,
                          Eigen::VectorXd *d) const {
  std::ostringstream strs;
  if (!theta || !d) {
    strs.str("");
    strs << " input pointer is null"
         << " in " << __FUNCTION__ << " at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
  }
  if (jnt.size() < DoF_ || theta->size() != DoF_ || d->size() != DoF_) {
    strs.str("");
    strs << " input parameters have wrong size "
         << " in " << __FUNCTION__ << " at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
  }
  (*d)(2) += jnt(2) * pitch_(2);
  (*theta)(0) += jnt(0) * pitch_(0);
  (*theta)(1) += jnt(1) * pitch_(1);
  (*theta)(3) += jnt(3) * pitch_(3);
}

}  // namespace kinematics_lib
