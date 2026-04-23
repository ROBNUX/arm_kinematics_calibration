#include "arm_calib_kinematics/serialarm_calib.hpp"

#include "robnux_kdl_common/pose.hpp"

// register plugin
PLUGINLIB_EXPORT_CLASS(kinematics_lib::SerialArmCalib,
                       kinematics_lib::BaseKinematicMap)
PLUGINLIB_EXPORT_CLASS(kinematics_lib::SerialArmCalib,
                       kinematics_lib::BaseCalibration)

namespace kinematics_lib {

SerialArmCalib::SerialArmCalib()
    : serialArm(0, 0), BaseCalibration(), resetCache_(true) {}

SerialArmCalib::SerialArmCalib(const size_t DoF)
    : serialArm(DoF, DoF),
      BaseCalibration(),
      alpha_(Eigen::VectorXd::Zero(DoF)),
      alpha_c_(Eigen::VectorXd::Zero(DoF)),
      a_(Eigen::VectorXd::Zero(DoF)),
      a_c_(Eigen::VectorXd::Zero(DoF)),
      beta_(Eigen::VectorXd::Zero(DoF)),
      beta_c_(Eigen::VectorXd::Zero(DoF)),
      d_(Eigen::VectorXd::Zero(DoF)),
      d_c_(Eigen::VectorXd::Zero(DoF)),
      theta_(Eigen::VectorXd::Zero(DoF)),
      theta_c_(Eigen::VectorXd::Zero(DoF)),
      pitch_(Eigen::VectorXd::Ones(DoF)),
      resetCache_(true) {
  jnt_names_.resize(DoF);
  for (size_t i = 0; i < DoF; i++) {
    jnt_names_[i] = "JOINT_" + std::to_string(i) + "_ACT";
  }
}

SerialArmCalib::SerialArmCalib(const Eigen::VectorXd &kine_para)
    : serialArm((kine_para.size() - 14) / 6, (kine_para.size() - 14) / 6),
      BaseCalibration(),
      resetCache_(true) {
  SetGeometry(kine_para);
  jnt_names_.resize(DoF_);
  for (size_t i = 0; i < DoF_; i++) {
    jnt_names_[i] = "JOINT_" + std::to_string(i) + "_ACT";
  }
}



bool SerialArmCalib::resetCalibration() {
  alpha_c_ = alpha_;
  a_c_ = a_;
  d_c_ = d_;
  theta_c_ = theta_;
  beta_c_ = beta_;
  isDHCalibrated_ = false;
  std::ostringstream strs;
  strs.str("");
  strs << GetName() << ": calib model is reset to initial model" << std::endl;
  LOG_INFO(strs);
  return true;
}


void SerialArmCalib::UpdateDH(const Eigen::VectorXd& delta_p,
                              Eigen::VectorXd& alpha, Eigen::VectorXd& a,
                              Eigen::VectorXd& theta, Eigen::VectorXd& d) {
  std::ostringstream strs;
  size_t numParam = delta_p.size();
  if (numParam != 4 * DoF_) {
    strs.str("");
    strs << GetName() << ":"
         << "number of hidden parameters " << numParam
         << "not equal to independent cols " << 4 * DoF_ << std::endl;
    LOG_ERROR(strs);
    return;
  }
  // now assign back all iterated parameters
  for (size_t i = 0; i < numParam; i++) {
    // 4 means each group has 4 dh parameters
    size_t group_id = floor(i / 4);
    size_t remain_id = i - group_id * 4;
    if (group_id < DoF_) {
      // means this parameter must belong to dh of the group group_id
      if (remain_id == 0) {  // alpha parameter
        alpha(group_id) += delta_p(i);
      } else if (remain_id == 1) {  // a parameter
        a(group_id) += delta_p(i);
      } else if (remain_id == 2) {  // theta parameter
        theta(group_id) += delta_p(i);
      } else {  // d parameter
        d(group_id) += delta_p(i);
      } 
    }
  }
}

bool SerialArmCalib::PickSubJacobianForPara(const Eigen::MatrixXd& Jt_p,
                                            const Eigen::MatrixXd& Jp_r,
                                            Eigen::MatrixXd& Js_t1,
                                            Eigen::MatrixXd& Js_r1,
                                            const bool reduction) {
  std::ostringstream strs;
  strs.str("");
  strs << GetName() << ":"
       << "This function shouldn't be called, should be implemented in"
       << " children class, in " << __FUNCTION__ << " line " << __LINE__
       << std::endl;
  LOG_ALARM(strs);
  return false;
}

bool SerialArmCalib::GetCalibParamSet(EigenDRef<Eigen::VectorXd>& cal_DH) {
  std::ostringstream strs;
  // additional 7 is for baseOffSet
  if (cal_DH.size() < 5 * DoF_ + 14) {
    strs.str("");
    strs << GetName() << ":"
         << "The input vector pointer has wrong dimension in function "
         << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  cal_DH.segment(0, DoF_) = alpha_c_;
  cal_DH.segment(DoF_, DoF_) = a_c_;
  cal_DH.segment(2 * DoF_, DoF_) = theta_c_;
  cal_DH.segment(3 * DoF_, DoF_) = d_c_;
  cal_DH.segment(4 * DoF_, DoF_) = pitch_;

  Eigen::VectorXd eigBaseOff = defaultBaseOff_.ToEigenVecQuat();
  cal_DH.segment(5 * DoF_, 7) = eigBaseOff;
  return true;
}

bool SerialArmCalib::LoadCalibParamSet(
    const EigenDRef<Eigen::VectorXd>& cal_DH) {
  std::ostringstream strs;
  if (cal_DH.size() < 5 * DoF_ + 7) {
    strs.str("");
    strs << GetName() << ":"
         << "The input vector has wrong dimension in function " << __FUNCTION__
         << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  alpha_c_ = cal_DH.segment(0, DoF_);
  a_c_ = cal_DH.segment(DoF_, DoF_);
  theta_c_ = cal_DH.segment(2 * DoF_, DoF_);
  d_c_ = cal_DH.segment(3 * DoF_, DoF_);
  pitch_ = cal_DH.segment(4 * DoF_, DoF_);
  strs.str("");
  strs << GetName() << ":"
       << "load calib: alpha_c: " << alpha_c_ << ", a_c: " << a_c_
       << ", theta_c: " << theta_c_ << ", d_c: " << d_c_
       << ", pitch=" << pitch_ << std::endl;
  LOG_INFO(strs);

  Eigen::VectorXd baseoff = cal_DH.segment(5 * DoF_, 7);
  Vec t(baseoff.segment(0, 3));
  Quaternion q(baseoff(3), baseoff(4), baseoff(5), baseoff(6));
  defaultBaseOff_.setTranslation(t);
  defaultBaseOff_.setQuaternion(q);
  strs.str("");
  strs << GetName() << ":"
       << "SetDefaultBaseOff = " << defaultBaseOff_.ToString(true)
       << ", quat=" << defaultBaseOff_.ToString(false)
       << ", in edgen=" << baseoff.transpose() << std::endl;
  LOG_INFO(strs);
  isDHCalibrated_ = true;
  return true;
}

double SerialArmCalib::LaserDistanceCalib(
    const Eigen::VectorXd &base_offset, const Eigen::VectorXd &tool_offset,
    const EigenDRef<Eigen::Matrix3d> &laser2CartMap,
    const EigenDRef<Eigen::MatrixXd> &cart_measure,
    const EigenDRef<Eigen::MatrixXd> &qa_array,
    const EigenDRef<Eigen::MatrixXd> &laser_measure) {
  // check if robot has been initialized, i.e. we need to know
  // the rough kinematic model
  std::ostringstream strs;
  if (!initialized_) {
    strs.str("");
    strs << GetName() << ":"
         << "geometric parameters are not initialized"
         << " so can not do calibration, in function " << __FUNCTION__
         << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_PARAM_NOT_INITIALIZED;
  }

  strs.str("");
  strs << GetName() << ":"
       << "before calib, base = " << base_offset << ", tool =" << tool_offset
       << std::endl;
  strs << ", LaserMatrix=" << laser2CartMap << std::endl;
  LOG_INFO(strs);

  size_t total_measures = laser_measure.cols();
  size_t total_jnts = qa_array.cols();

  // 2/3 samples used for calibration, and the remaining 1/3 for testing
  size_t num_measures = 2 * total_measures / 3;
  if (num_measures < 2) {
    strs.str("");
    strs << GetName() << ":"
         << "laser distance calibration: need at least two samples,"
         << " so can not do calibration, in function " << __FUNCTION__
         << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_CALIB_LASER_LESS_SAMPLES;
  }
  if (base_offset.size() != 7) {
    strs.str("");
    strs << GetName() << ":"
         << "Input base_offset has wrong dimension,"
         << " so can not do calibration, in function " << __FUNCTION__
         << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM;
  }
  // either laser measures == jnt measures, or jnt measures = laser measures + 8
  bool sizeOK = total_measures == total_jnts;
  if (total_measures != cart_measure.cols() || cart_measure.rows() < 3 ||
      !sizeOK || laser_measure.rows() < 3) {
    strs.str("");
    strs << GetName() << ":"
         << "Laser calibration: number of measure ee coordinates"
            " and number of joint records does not match,"
         << " so can not do calibration, in function " << __FUNCTION__
         << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM;
  }

  // compute base orientation, translation and frame using base_offset data
  Eigen::Vector3d pb = base_offset.block(0, 0, 3, 1);
  Quaternion qb = Quaternion::FromEigenVec(base_offset.block(3, 0, 4, 1));
  Rotation rb(qb);

  // initialize iter alg parameters
  opt_alg_.setParam(CALIB_DECENT_STEPSIZE, sam_region_scale_);
  resetCache_ = true;

  // in any case, we start with pre-calibrated model
  Eigen::VectorXd a_tmp = a_c_, alpha_tmp = alpha_c_, beta_tmp = beta_c_,
                  d_tmp = d_c_, theta_tmp = theta_c_;
  Eigen::VectorXd a_old = a_tmp, alpha_old = alpha_tmp, beta_old = beta_tmp,
                  d_old = d_tmp, theta_old = theta_tmp;
  // get the translational part of tool offset, only which affect
  // measurement point coordinates
  Eigen::Vector3d tcp_trans = tool_offset.segment(0, 3);

  // step 1, we need to define one matrix A and one vector b for regression
  // number of columns 5 * DoF_: alpha_ [DoF_], a_ [DoF_], theta_ [DoF_],
  // d_ [DoF_], beta_[DoF_]
  Eigen::MatrixXd A((num_measures - 1) * 3, 5 * DoF_);
  Eigen::VectorXd b((num_measures - 1) * 3);

  double previous_err = std::numeric_limits<double>::max();
  double estimation_err = 0.5 * previous_err;
  int cur_iter = 0;

  // because laser displacement sensors only measures the distances traveled
  // so we need to form relative coordinate vector change (so does Jacobian
  // change)
  Eigen::MatrixXd first_J;  // very first Jacobian
  Eigen::Vector3d first_p;  // first Cartesian EE coordinates

  // decay process until (1) estimation_err is less than a given limit
  // or (2) error change is less than a givne limit or maximal iteration reached
  while ((estimation_err > MAX_CALIB_STOP_ERR &&
              previous_err - estimation_err > MAX_CALIB_MATCHING_ERR ||
          !resetCache_) &&
         cur_iter < MAX_CALIB_OUTER_ITER) {
    // keep a copy of previous-step paramter vector
    if (resetCache_) {
      a_old = a_tmp;
      alpha_old = alpha_tmp;
      d_old = d_tmp;
      theta_old = theta_tmp;
      beta_old = beta_tmp;
      previous_err = estimation_err;
    }
    cur_iter++;

    Eigen::VectorXd kine_para(5 * DoF_),
        tmp_para(5 * DoF_);  // clearing kine_para, and tmp_para
    // fill in the value of alpha_tmp, a_tmp, theta_tmp, d_tmp
    kine_para.segment(0, DoF_) = alpha_tmp;
    kine_para.segment(DoF_, DoF_) = a_tmp;
    kine_para.segment(2 * DoF_, DoF_) = theta_tmp;
    kine_para.segment(3 * DoF_, DoF_) = d_tmp;
    kine_para.segment(4 * DoF_, DoF_) = beta_tmp;

    // with current set of DHs: kine_para,  compute A,b, and estimation_err
    estimation_err = 0;
    for (size_t i = 0; i < num_measures; i++) {
      UpdateDH(kine_para, qa_array.col(i), &tmp_para);
      Eigen::MatrixXd Jp_t, Jp_r;
      Pose p;
      // compute the expected values from known canonical kinematic parameters
      // i.e., not-calibrated parameter set
      int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r, true);
      if (ret < 0) {
        return ret;
      }
      // Compute the translation of measuring robot tip w.r.t. world frame
      Rotation r_n = p.getRotation();
      Vec t_n = p.getTranslation();
      Vec tcp_offset(tcp_trans);
      Vec t_e = r_n * tcp_offset + t_n;
      Vec tf = rb * t_e + pb;

      // next we compute translational Jacobian of robot tip w.r.t world frame
      Eigen::Matrix3d t_e_hat;
      t_e.ToHat(&t_e_hat);
      Eigen::MatrixXd current_J(3, 5 * DoF_);
      current_J = rb.ToEigenMat() * (Jp_t - t_e_hat * Jp_r);

      if (i == 0) {
        first_J = current_J;
        first_p = tf.ToEigenVec();
      } else {
        Eigen::Vector3d dp = tf.ToEigenVec() - first_p;
        Eigen::Vector3d dls1 = laser_measure.col(i) - laser_measure.col(0);
        Eigen::Vector3d dls = laser2CartMap * dls1;
        // laser rel. displacement  - robot reported displacement
        Eigen::Vector3d epsi = dls - dp;
        estimation_err += epsi.norm();
        A.block((i - 1) * 3, 0, 3, 5 * DoF_) = current_J - first_J;
        b.block((i - 1) * 3, 0, 3, 1) = epsi;
      }
    }

    // average estimation error for a single measurement
    estimation_err /= double(num_measures - 1);
    // using opt_alg_ to compute the best delta_para given the current para
    // used as an internal loop
    strs.str("");
    strs << GetName() << ":"
         << "Calib. through laser sensor, iteration no. = " << cur_iter
         << std::endl;
    LOG_INFO(strs);
    Eigen::VectorXd delta_p_old;
    if (resetCache_) {
      size_t numParam = A.cols();
      delta_p_old_cache_ = Eigen::VectorXd::Zero(numParam);
    }
    if (!opt_alg_.OptGradientVec(A, b, &delta_p_old)) {
      if (delta_p_old.size() == 0) {
        strs.str("");
        strs << "Gradient Decent fails" << std::endl;
        LOG_ERROR(strs);
        return -ERR_CALIB_REG_WRONG_DIM;
      }
      delta_p_old_cache_ = delta_p_old;
      UpdateDH(delta_p_old, &alpha_tmp, &a_tmp, &theta_tmp, &d_tmp, &beta_tmp);
      resetCache_ = false;
    } else {
      UpdateDH(delta_p_old - delta_p_old_cache_, &alpha_tmp, &a_tmp, &theta_tmp,
               &d_tmp, &beta_tmp);
      resetCache_ = true;
    }
  }
  if (cur_iter >= MAX_CALIB_OUTER_ITER) {
    strs.str("");
    strs << GetName() << ":"
         << "In laser calib: Iteration reaches maximal " << MAX_CALIB_OUTER_ITER
         << "With estimation error " << estimation_err << std::endl;
    LOG_ERROR(strs);
    // return -ERR_CALIB_REG_MAX_ITER;
  }
  if (estimation_err <= MAX_CALIB_STOP_ERR) {
    strs.str("");
    strs << GetName() << ":"
         << "In Laser calib: Iteration reaches estimation_err "
         << estimation_err << ", while the set limit is  " << MAX_CALIB_STOP_ERR
         << std::endl;
    LOG_ERROR(strs);
  }
  if (previous_err - estimation_err <= MAX_CALIB_MATCHING_ERR) {
    strs.str("");
    strs << GetName() << ":"
         << "In Laser calib: Iteration reaches err diff "
         << previous_err - estimation_err << ", while the set limit is  "
         << MAX_CALIB_MATCHING_ERR << std::endl;
    LOG_ERROR(strs);
  }

  alpha_c_ = alpha_old;
  a_c_ = a_old;
  theta_c_ = theta_old;
  d_c_ = d_old;
  beta_c_ = beta_old;
  strs.str("");
  strs << GetName() << ":"
       << "alpha_c: " << alpha_c_ << ", a_c: " << a_c_
       << ", theta_c: " << theta_c_ << ", d_c: " << d_c_
       << ", beta_c: " << beta_c_ << std::endl;
  strs << "tool = " << tool_offset << std::endl;
  strs << "base = " << base_offset << std::endl;
  strs << "final matching error=" << previous_err << std::endl;
  LOG_INFO(strs);
  isDHCalibrated_ = true;

  Eigen::VectorXd kine_para(5 * DoF_),
      tmp_para(5 * DoF_);  // clearing kine_para
  kine_para.segment(0, DoF_) = alpha_tmp;
  kine_para.segment(DoF_, DoF_) = a_tmp;
  kine_para.segment(2 * DoF_, DoF_) = theta_tmp;
  kine_para.segment(3 * DoF_, DoF_) = d_tmp;
  kine_para.segment(4 * DoF_, DoF_) = beta_tmp;
  double orig_err = 0, comp_err = 0;
  for (size_t i = num_measures; i < total_measures; i++) {
    UpdateDH(kine_para, qa_array.col(i), &tmp_para);
    Eigen::MatrixXd Jp_t, Jp_r;
    Pose p;
    // compute the expected values from known canonical kinematic parameters
    // i.e., not-calibrated parameter set
    int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r, true);
    if (ret < 0) {
      return ret;
    }
    // Compute the translation of measuring robot tip w.r.t. world frame
    Rotation r_n = p.getRotation();
    Vec t_n = p.getTranslation();
    Vec tcp_offset(tcp_trans);
    Vec t_e = r_n * tcp_offset + t_n;
    Vec tf = rb * t_e + pb;

    if (i == num_measures) {
      first_p = tf.ToEigenVec();
    } else {
      Eigen::Vector3d dp = tf.ToEigenVec() - first_p;

      Eigen::Vector3d dls = laser2CartMap * (laser_measure.col(i) -
                                             laser_measure.col(num_measures));
      comp_err += (dls - dp).norm();

      Eigen::VectorXd dcart =
          cart_measure.col(i) - cart_measure.col(num_measures);
      Eigen::Vector3d dc = dcart.block(0, 0, 3, 1);
      orig_err += (dls - dc).norm();
    }
  }
  double avg_orig_err = orig_err / (total_measures - num_measures - 1);
  double avg_comp_err = comp_err / (total_measures - num_measures - 1);
  double overall_comp_err =
      (previous_err * 2.0 + avg_comp_err) / 3.0;  // balanced overall comp err
  // average estimation error for a single measurement
  strs.str("");
  strs << GetName() << ":"
       << "accumulation error before comp. = " << avg_orig_err
       << ", after comp. the error = " << avg_comp_err << std::endl;
  LOG_INFO(strs);
  if (avg_comp_err >= avg_orig_err) {
    strs.str("");
    strs << GetName() << ":"
         << "Calib verification result is not satisfied" << std::endl;
    LOG_ALARM(strs);
  }
  return overall_comp_err;
}

Eigen::VectorXd SerialArmCalib::VerifyLaserDistanceCalib(
    const Eigen::VectorXd &base_offset, const Eigen::VectorXd &tool_offset,
    const EigenDRef<Eigen::Matrix3d> &laser2CartMap,
    const EigenDRef<Eigen::MatrixXd> &cart_measure,
    const EigenDRef<Eigen::MatrixXd> &qa_array,
    const EigenDRef<Eigen::MatrixXd> &laser_measure) {
  std::ostringstream strs;
  Eigen::VectorXd outData(2);
  // initialize the outData
  outData(0) = 0;  // no error
  outData(1) = 0;  // no verify calibration data
  // check if robot has been initialized, i.e. we need to know
  // the rough kinematic model
  if (!initialized_) {
    strs.str("");
    strs << GetName() << ":"
         << "geometric parameters are not initialized"
         << " so can not do calibration, in function " << __FUNCTION__
         << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);

    outData(0) = -ERR_ROB_PARAM_NOT_INITIALIZED;
    return outData;
  }
  size_t total_measures = qa_array.cols();
  if (total_measures < 2) {
    strs.str("");
    strs << GetName() << ":"
         << "No enough measurement points " << __FUNCTION__ << ", line "
         << __LINE__ << std::endl;
    LOG_ERROR(strs);

    outData(0) = -ERR_ROB_PARAM_NOT_INITIALIZED;
    return outData;
  }
  size_t measDoF = laser_measure.rows();
  if (total_measures != laser_measure.cols() || measDoF > 3) {
    strs.str("");
    strs << GetName() << ":"
         << "calibration: number of measure ee coordinates"
            " and number of joint records does not match, or meas has "
            "dimension > 3"
         << " so can not do calibration, in function " << __FUNCTION__
         << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    outData(0) = -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM;
    return outData;
  }

  Eigen::Vector3d pb = base_offset.block(0, 0, 3, 1);
  Quaternion qb = Quaternion::FromEigenVec(base_offset.block(3, 0, 4, 1));
  Rotation rb(qb);

  Eigen::VectorXd kine_para(5 * DoF_), tmp_para(5 * DoF_);
  kine_para.segment(0, DoF_) = alpha_c_;
  kine_para.segment(DoF_, DoF_) = a_c_;
  kine_para.segment(2 * DoF_, DoF_) = theta_c_;
  kine_para.segment(3 * DoF_, DoF_) = d_c_;
  kine_para.segment(4 * DoF_, DoF_) = beta_c_;

  Eigen::VectorXd tcp_trans = tool_offset.segment(0, 3);
  Eigen::VectorXd first_p;  // first Cartesian coordinate of measuring tip
  double orig_err = 0, comp_err = 0;
  for (size_t i = 0; i < total_measures; i++) {
    UpdateDH(kine_para, qa_array.col(i), &tmp_para);
    Eigen::MatrixXd Jp_t, Jp_r;
    Pose p;
    // compute the expected values from known canonical kinematic parameters
    // i.e., not-calibrated parameter set
    int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r, true);
    if (ret < 0) {
      outData(0) = ret;
      return outData;
    }
    // Compute the translation of measuring robot tip w.r.t. world frame
    Rotation r_n = p.getRotation();
    Vec t_n = p.getTranslation();
    Vec tool_offset(tcp_trans);
    Vec t_e = r_n * tool_offset + t_n;
    Vec tf = rb * t_e + pb;

    if (i == 0) {
      first_p = tf.ToEigenVec();
    } else {
      Eigen::Vector3d dp = tf.ToEigenVec() - first_p;

      Eigen::Vector3d dls =
          laser2CartMap * (laser_measure.col(i) - laser_measure.col(0));
      comp_err += (dls - dp).norm();

      Eigen::VectorXd dcart = cart_measure.col(i) - cart_measure.col(0);
      Eigen::Vector3d dc = dcart.block(0, 0, 3, 1);
      orig_err += (dls - dc).norm();
    }
  }
  // average estimation error for a single measurement
  strs.str("");
  strs << GetName() << ":"
       << "accumulation error before comp. = "
       << orig_err / (total_measures - 1)
       << ", after comp. the error = " << comp_err / (total_measures - 1)
       << std::endl;
  LOG_INFO(strs);

  outData(0) = orig_err / (total_measures - 1);
  if (comp_err >= orig_err) {
    strs.str("");
    strs << GetName() << ":"
         << "Calib verification result is not satisfied" << std::endl;
    LOG_ALARM(strs);
    outData(1) = -comp_err / (total_measures - 1);
  } else {
    outData(1) = comp_err / (total_measures - 1);
  }
  return outData;
}

double SerialArmCalib::DirectMesCalib(
    const Eigen::VectorXd &base_offset, const Eigen::VectorXd &tool_offset,
    const EigenDRef<Eigen::MatrixXd> &cart_measure,
    const EigenDRef<Eigen::MatrixXd> &measureMents,
    const EigenDRef<Eigen::MatrixXd> &qa_array) {
  std::ostringstream strs;
  // check if robot has been initialized, i.e. we need to know
  // the rough kinematic model
  if (!initialized_) {
    strs.str("");
    strs << GetName() << ":"
         << "Robot geometric parameters are not initialized"
         << " so can not do calibration, in function " << __FUNCTION__
         << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_PARAM_NOT_INITIALIZED;
  }

  strs.str("");
  strs << GetName() << ":"
       << "before calib, init base = " << base_offset
       << ", init tool =" << tool_offset << std::endl;
  size_t total_measures = measureMents.cols();
  size_t num_measures = 2 * total_measures / 3;
  size_t total_jnts = qa_array.cols();

  if (num_measures < 2) {
    strs.str("");
    strs << GetName() << ":"
         << "laser calibration: need at least two samples,"
         << " so can not do calibration, in function " << __FUNCTION__
         << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_CALIB_LASER_LESS_SAMPLES;
  }
  if (base_offset.size() != 7 || tool_offset.size() != 7) {
    strs.str("");
    strs << GetName() << ":"
         << "Input base_offset or tool_offset has wrong dimension,"
         << " so can not do calibration, in function " << __FUNCTION__
         << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM;
  }

  size_t measDoF = measureMents.rows();
  bool sizeOK = total_measures == total_jnts;
  if (total_measures != cart_measure.cols() || cart_measure.rows() < 3 ||
      !sizeOK || measDoF < 3) {
    strs.str("");
    strs << GetName() << ":"
         << "calibration: number of measure ee coordinates"
            " and number of joint records does not match, or each meas has "
            "wrong dimension,"
         << " so can not do calibration, in function " << __FUNCTION__
         << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM;
  }

  // initialize iter alg parameters
  opt_alg_.setParam(CALIB_DECENT_STEPSIZE, sam_region_scale_);
  resetCache_ = true;
  // in any case, we start with uncalibrated model
  Eigen::VectorXd a_tmp = a_c_, alpha_tmp = alpha_c_, beta_tmp = beta_c_,
                  d_tmp = d_c_, theta_tmp = theta_c_;
  Eigen::VectorXd a_old = a_tmp, alpha_old = alpha_tmp, beta_old = beta_tmp,
                  d_old = d_tmp, theta_old = theta_tmp;
  // get the translational part of tool offset, only which affect
  // measurement point coordinates
  Eigen::Vector3d tcp_trans = tool_offset.segment(0, 3);

  // compute base orientation, translation and frame using base_offset data
  Eigen::Vector3d pb = base_offset.block(0, 0, 3, 1);
  Quaternion qb = Quaternion::FromEigenVec(base_offset.block(3, 0, 4, 1));
  Rotation rb(qb);

  // step 1, using canonical FK to compute the corresponding
  // we need to define one matrix A and one vector b for regression
  // number of columns 5 * DoF_, alpha_ [DoF_], a_ [DoF_],
  // theta_ [DoF_], d_ [DoF_], beta_[DoF_]
  Eigen::MatrixXd A((num_measures - 1) * measDoF, 5 * DoF_);
  Eigen::VectorXd b((num_measures - 1) * measDoF);
  double previous_err = std::numeric_limits<double>::max();
  double estimation_err = 0.5 * previous_err;

  int cur_iter = 0;
  while ((estimation_err > MAX_CALIB_STOP_ERR &&
              previous_err - estimation_err > MAX_CALIB_MATCHING_ERR ||
          !resetCache_) &&
         cur_iter < MAX_CALIB_OUTER_ITER) {
    // assign previous value
    if (resetCache_) {
      a_old = a_tmp;
      alpha_old = alpha_tmp;
      d_old = d_tmp;
      theta_old = theta_tmp;
      beta_old = beta_tmp;
      previous_err = estimation_err;
    }
    cur_iter++;

    Eigen::VectorXd kine_para(5 * DoF_),
        tmp_para(5 * DoF_);  // clearing kine_para, and tmp_para
    // fill in the value of alpha_tmp, a_tmp, theta_tmp, d_tmp
    kine_para.segment(0, DoF_) = alpha_tmp;
    kine_para.segment(DoF_, DoF_) = a_tmp;
    kine_para.segment(2 * DoF_, DoF_) = theta_tmp;
    kine_para.segment(3 * DoF_, DoF_) = d_tmp;
    kine_para.segment(4 * DoF_, DoF_) = beta_tmp;
    estimation_err = 0;

    for (size_t i = 0; i < num_measures; i++) {
      UpdateDH(kine_para, qa_array.col(i), &tmp_para);
      Eigen::MatrixXd Jp_t, Jp_r;
      Pose p;
      // compute the expected values from known canonical kinematic parameters
      // i.e., not-calibrated parameter set
      int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r, true);
      if (ret < 0) {
        return ret;
      }

      // Compute the translation of measuring robot tip w.r.t. world frame
      Rotation r_n = p.getRotation();
      Vec t_n = p.getTranslation();
      Vec tcp_offset(tcp_trans);
      Vec t_e = r_n * tcp_offset + t_n;
      Vec tf = rb * t_e + pb;

      Eigen::Matrix3d t_e_hat;
      t_e.ToHat(&t_e_hat);
      Eigen::MatrixXd J = rb.ToEigenMat() * (Jp_t - t_e_hat * Jp_r);
      Eigen::Vector3d dp = measureMents.col(i) - tf.ToEigenVec();
      estimation_err += dp.norm();  // accu_error;
      A.block((i - 1) * measDoF, 0, measDoF, 5 * DoF_) =
          J.block(0, 0, measDoF, 5 * DoF_);
      b.block((i - 1) * measDoF, 0, measDoF, 1) = dp;
    }

    // average estimation error for a single measurement
    estimation_err /= double(num_measures - 1);

    // using opt_alg_ to compute the best delta_para given the current para
    // used as an internal loop
    strs.str("");
    strs << GetName() << ":"
         << "Calib. through laser sensor, iteration no. = " << cur_iter
         << std::endl;
    LOG_INFO(strs);
    Eigen::VectorXd delta_p_old;
    if (resetCache_) {
      size_t numParam = A.cols();
      delta_p_old_cache_ = Eigen::VectorXd::Zero(numParam);
    }
    if (!opt_alg_.OptGradientVec(A, b, &delta_p_old)) {
      if (delta_p_old.size() == 0) {
        strs.str("");
        strs << "Gradient Decent fails" << std::endl;
        LOG_ERROR(strs);
        return -ERR_CALIB_REG_WRONG_DIM;
      }
      delta_p_old_cache_ = delta_p_old;
      UpdateDH(delta_p_old, &alpha_tmp, &a_tmp, &theta_tmp, &d_tmp, &beta_tmp);
      resetCache_ = false;
    } else {
      UpdateDH(delta_p_old - delta_p_old_cache_, &alpha_tmp, &a_tmp, &theta_tmp,
               &d_tmp, &beta_tmp);
      resetCache_ = true;
    }
  }
  if (cur_iter >= MAX_CALIB_OUTER_ITER) {
    strs.str("");
    strs << GetName() << ":"
         << "In fullrange calib: Iteration reaches maximal "
         << MAX_CALIB_OUTER_ITER << "With estimation error " << estimation_err
         << std::endl;
    LOG_ERROR(strs);
    // return -ERR_CALIB_REG_MAX_ITER;
  }
  if (estimation_err <= MAX_CALIB_STOP_ERR) {
    strs.str("");
    strs << GetName() << ":"
         << "In fullrange calib: Iteration reaches estimation_err "
         << estimation_err << ", while the set limit is  " << MAX_CALIB_STOP_ERR
         << std::endl;
    LOG_ERROR(strs);
  }
  if (previous_err - estimation_err <= MAX_CALIB_MATCHING_ERR) {
    strs.str("");
    strs << GetName() << ":"
         << "In fullrange calib: Iteration reaches err diff "
         << previous_err - estimation_err << ", while the set limit is  "
         << MAX_CALIB_MATCHING_ERR << std::endl;
    LOG_ERROR(strs);
  }

  alpha_c_ = alpha_old;
  a_c_ = a_old;
  theta_c_ = theta_old;
  d_c_ = d_old;
  beta_c_ = beta_old;
  strs.str("");
  strs << GetName() << ":"
       << "alpha_c: " << alpha_c_ << ", a_c: " << a_c_
       << ", theta_c: " << theta_c_ << ", d_c: " << d_c_
       << ", beta_c: " << beta_c_ << std::endl;
  strs << "tool = " << tool_offset << std::endl;
  strs << "base = " << base_offset << std::endl;
  strs << "final matching error=" << previous_err << std::endl;
  LOG_INFO(strs);
  isDHCalibrated_ = true;

  Eigen::VectorXd kine_para(5 * DoF_),
      tmp_para(5 * DoF_);  // clearing kine_para
  kine_para.segment(0, DoF_) = alpha_tmp;
  kine_para.segment(DoF_, DoF_) = a_tmp;
  kine_para.segment(2 * DoF_, DoF_) = theta_tmp;
  kine_para.segment(3 * DoF_, DoF_) = d_tmp;
  kine_para.segment(4 * DoF_, DoF_) = beta_tmp;
  double orig_err = 0, comp_err = 0;
  for (size_t i = num_measures; i < total_measures; i++) {
    UpdateDH(kine_para, qa_array.col(i), &tmp_para);
    Eigen::MatrixXd Jp_t, Jp_r;
    Pose p;
    // compute the expected values from known canonical kinematic parameters
    // i.e., not-calibrated parameter set
    int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r, true);
    if (ret < 0) {
      return ret;
    }
    // Compute the translation of measuring robot tip w.r.t. world frame
    Rotation r_n = p.getRotation();
    Vec t_n = p.getTranslation();
    Vec tcp_offset(tcp_trans);
    Vec t_e = r_n * tcp_offset + t_n;
    Vec tf = rb * t_e + pb;

    Eigen::Vector3d dp = measureMents.col(i) - tf.ToEigenVec();
    comp_err += dp.norm();

    Eigen::VectorXd dcart = measureMents.col(i) - cart_measure.col(i);
    orig_err += dcart.norm();
  }
  double avg_orig_err = orig_err / (total_measures - num_measures - 1);
  double avg_comp_err = comp_err / (total_measures - num_measures - 1);
  double overall_comp_err =
      (previous_err * 2.0 + avg_comp_err) / 3.0;  // balanced overall comp err
  // average estimation error for a single measurement
  strs.str("");
  strs << GetName() << ":"
       << "accumulation error before comp. = " << avg_orig_err
       << ", after comp. the error = " << avg_comp_err << std::endl;
  LOG_INFO(strs);
  if (avg_comp_err >= avg_orig_err) {
    strs.str("");
    strs << GetName() << ":"
         << "Calib verification result is not satisfied" << std::endl;
    LOG_ALARM(strs);
  }
  return overall_comp_err;
}

Eigen::VectorXd SerialArmCalib::VerifyDirectMesCalib(
    const Eigen::VectorXd &base_offset, const Eigen::VectorXd &tool_offset,
    const EigenDRef<Eigen::MatrixXd> &cart_measure,
    const EigenDRef<Eigen::MatrixXd> &measureMents,
    const EigenDRef<Eigen::MatrixXd> &qa_array) {
  std::ostringstream strs;
  Eigen::VectorXd outData(2);
  // initialize the outData
  outData(0) = 0;  // no error
  outData(1) = 0;  // no verify calibration data
  // check if robot has been initialized, i.e. we need to know
  // the rough kinematic model
  if (!initialized_) {
    strs.str("");
    strs << GetName() << ":"
         << "geometric parameters are not initialized"
         << " so can not do calibration, in function " << __FUNCTION__
         << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);

    outData(0) = -ERR_ROB_PARAM_NOT_INITIALIZED;
    return outData;
  }
  size_t total_measures = measureMents.cols();
  size_t total_jnts = qa_array.cols();
  if (total_measures < 2) {
    strs.str("");
    strs << GetName() << ":"
         << "No enough measurement points " << __FUNCTION__ << ", line "
         << __LINE__ << std::endl;
    LOG_ERROR(strs);

    outData(0) = -ERR_ROB_PARAM_NOT_INITIALIZED;
    return outData;
  }
  size_t measDoF = measureMents.rows();
  bool sizeOK = total_measures == total_jnts;
  if (total_measures != cart_measure.cols() || cart_measure.rows() < 3 ||
      !sizeOK || measDoF < 3) {
    strs.str("");
    strs << GetName() << ":"
         << "calibration: number of measure ee coordinates"
            " and number of joint records does not match, or meas has "
            "dimension > 3"
         << " so can not do calibration, in function " << __FUNCTION__
         << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    outData(0) = -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM;
    return outData;
  }

  Eigen::Vector3d pb = base_offset.block(0, 0, 3, 1);
  Quaternion qb = Quaternion::FromEigenVec(base_offset.block(3, 0, 4, 1));
  Rotation rb(qb);

  Eigen::VectorXd kine_para(5 * DoF_), tmp_para(5 * DoF_);
  kine_para.segment(0, DoF_) = alpha_c_;
  kine_para.segment(DoF_, DoF_) = a_c_;
  kine_para.segment(2 * DoF_, DoF_) = theta_c_;
  kine_para.segment(3 * DoF_, DoF_) = d_c_;
  kine_para.segment(4 * DoF_, DoF_) = beta_c_;

  Eigen::VectorXd tcp_trans = tool_offset.segment(0, 3);
  Eigen::VectorXd first_p;  // first Cartesian coordinate of measuring tip
  double orig_err = 0, comp_err = 0;
  for (size_t i = 0; i < total_measures; i++) {
    UpdateDH(kine_para, qa_array.col(i), &tmp_para);
    Eigen::MatrixXd Jp_t, Jp_r;
    Pose p;
    // compute the expected values from known canonical kinematic parameters
    // i.e., not-calibrated parameter set
    int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r, true);
    if (ret < 0) {
      outData(0) = ret;
      return outData;
    }
    // Compute the translation of measuring robot tip w.r.t. world frame
    Rotation r_n = p.getRotation();
    Vec t_n = p.getTranslation();
    Vec tool_offset(tcp_trans);
    Vec t_e = r_n * tool_offset + t_n;
    Vec tf = rb * t_e + pb;

    Eigen::Vector3d dp = measureMents.col(i) - tf.ToEigenVec();
    comp_err += dp.norm();

    Eigen::VectorXd dcart = measureMents.col(i) - cart_measure.col(i);
    orig_err += dcart.norm();
  }
  // average estimation error for a single measurement
  strs.str("");
  strs << GetName() << ":"
       << "accumulation error before comp. = "
       << orig_err / (total_measures - 1)
       << ", after comp. the error = " << comp_err / (total_measures - 1)
       << std::endl;
  LOG_INFO(strs);

  outData(0) = orig_err / (total_measures - 1);
  if (comp_err >= orig_err) {
    strs.str("");
    strs << GetName() << ":"
         << "Calib verification result is not satisfied" << std::endl;
    LOG_ALARM(strs);
    outData(1) = -comp_err / (total_measures - 1);
  } else {
    outData(1) = comp_err / (total_measures - 1);
  }
  return outData;
}

int SerialArmCalib::CalibTCPDistMethod(
    const Eigen::VectorXd &base_offset,
    const EigenDRef<Eigen::MatrixXd> &qa_array,
    const EigenDRef<Eigen::VectorXd> &displace_array,
    const EigenDRef<Eigen::Vector3d> &init_normal,
    EigenDRef<Eigen::VectorXd> *final_tool_offset) {
  std::ostringstream strs;
  // check if robot has been initialized, i.e. we need to know
  // the rough kinematic model
  if (!initialized_) {
    strs.str("");
    strs << GetName() << ":"
         << "Robot geometric parameters are not initialized"
         << " so can not do calibration, in function " << __FUNCTION__
         << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_PARAM_NOT_INITIALIZED;
  }
  if (!final_tool_offset) {
    strs.str("");
    strs << GetName() << ":"
         << "Input final_tool_offset pointer is null"
         << " so can not do tool calibration, in function " << __FUNCTION__
         << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_INPUT_POINTER_NULL;
  }

  size_t num_jnt_measures = qa_array.cols();
  size_t num_dist_measures = displace_array.size();
  if (num_jnt_measures != num_dist_measures || num_jnt_measures < 3) {
    strs.str("");
    strs << GetName() << ":"
         << "input data dimension is not matching or too few measures"
         << " so can not do  tool calibration, in function " << __FUNCTION__
         << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM;
  }

  strs.str("");
  strs << GetName() << ":"
       << " init_normal =" << init_normal << std::endl;
  LOG_INFO(strs);

  // compute base orientation, translation and frame using base_offset data
  Eigen::Vector3d pb = base_offset.block(0, 0, 3, 1);
  Quaternion qb = Quaternion::FromEigenVec(base_offset.block(3, 0, 4, 1));
  Rotation rb(qb);

  Eigen::VectorXd a_tmp, alpha_tmp, beta_tmp, d_tmp, theta_tmp;
  a_tmp = a_c_;
  alpha_tmp = alpha_c_;
  beta_tmp = beta_c_;
  d_tmp = d_c_;
  theta_tmp = theta_c_;

  Eigen::MatrixXd A(num_jnt_measures - 1, 3);
  Eigen::VectorXd b(num_jnt_measures - 1);

  Eigen::VectorXd kine_para(5 * DoF_), tmp_para(5 * DoF_);
  kine_para.segment(0, DoF_) = alpha_tmp;
  kine_para.segment(DoF_, DoF_) = a_tmp;
  kine_para.segment(2 * DoF_, DoF_) = theta_tmp;
  kine_para.segment(3 * DoF_, DoF_) = d_tmp;
  kine_para.segment(4 * DoF_, DoF_) = beta_tmp;

  // very first trans vec and rotational matrix
  Eigen::Vector3d p1;
  Eigen::Matrix3d R1;

  for (size_t i = 0; i < num_jnt_measures; i++) {
    UpdateDH(kine_para, qa_array.col(i), &tmp_para);
    Eigen::MatrixXd Jp_t, Jp_r;
    Pose p;
    // compute the expected values from known canonical kinematic parameters
    // i.e., not-calibrated parameter set
    int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r, true);
    if (ret < 0) {
      strs.str("");
      strs << GetName() << ":"
           << " calJacobian returns error code " << ret << " in function "
           << __FUNCTION__ << " at line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return ret;
    }

    // Compute the translation of measuring robot tip w.r.t. world frame
    Rotation r_n = p.getRotation();
    Vec t_n = p.getTranslation();
    Vec tf = rb * t_n + pb;
    Eigen::Vector3d tf1 = tf.ToEigenVec();
    Rotation r = rb * r_n;
    Eigen::Matrix3d tr1 = r.ToEigenMat();
    if (i == 0) {
      p1 = tf1;
      R1 = tr1;
    } else {
      // computing the Coefficient matrix
      A.block((i - 1), 0, 1, 3) = init_normal.transpose() * (tr1 - R1);
      b(i - 1) = displace_array(i) - displace_array(0) -
                 init_normal.transpose() * (tf1 - p1);
    }
  }
  Eigen::MatrixXd BB = A;
  if (GetName() == "scara") {  // scara
    BB = A.block(0, 0, num_jnt_measures - 1, 2);
  }
  strs.str("");
  strs << GetName() << ":"
       << "B=" << BB << std::endl;
  strs << "b=" << b << std::endl;
  LOG_INFO(strs);
  Eigen::MatrixXd ATA = BB.transpose() * BB;
  const double detV = ATA.determinant();

  if (detV < CALIB_SINGULAR_CONST) {
    strs.str("");
    strs << GetName() << ":"
         << "determinat is " << detV << " too smaller than "
         << CALIB_SINGULAR_CONST << std::endl;
    LOG_ERROR(strs);
    return -1;  // error
  }
  Eigen::VectorXd outV = ATA.inverse() * BB.transpose() * b;
  final_tool_offset->setZero();  // init to 0
  for (size_t i = 0; i < outV.size(); i++) {
    (*final_tool_offset)(i) = outV(i);
  }

  // for this displacement measure sensor based tcp calibration, it only suits
  // with pinacle or needle type tcp, for which the tool offset is aligned with
  // one of the axis of tcp frame, for scara, this axis is x-axis, for
  // 6axis, this axis is z-axis
  Quaternion q;
  if (GetName() == "scara") {  // scara
    double yaw = atan2(outV(1), outV(0));
    q.SetEulerZYX(yaw, 0, 0);
  } else {  // 6 axis
    Vec v0(outV);
    Rotation r(v0);
    r.GetQuaternion(&q);
  }
  (*final_tool_offset)(3) = q.w();
  (*final_tool_offset)(4) = q.x();
  (*final_tool_offset)(5) = q.y();
  (*final_tool_offset)(6) = q.z();
  return 0;
}

void SerialArmCalib::SetUsingCalibratedModel(bool useCalibratedModel) {
  useCalibrated_ = useCalibratedModel;
}

int SerialArmCalib::CalibBaseFrame(
    const EigenDRef<Eigen::MatrixXd> &jnt_base_measures,
    const Eigen::VectorXd &mes_tool, EigenDRef<Eigen::VectorXd> *orig_base,
    EigenDRef<Eigen::VectorXd> *comp_base) {
  std::ostringstream strs;
  size_t numJnts = jnt_base_measures.cols();
  // here orig_base is the workpiece frame, should be reachable by IK
  if (jnt_base_measures.rows() < DoF_ || numJnts < 1) {
    strs.str("");
    strs << GetName() << ":"
         << "The input vector has wrong dimension in function " << __FUNCTION__
         << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
  }
  if (!orig_base || !comp_base) {
    strs.str("");
    strs << GetName() << ":"
         << "input orig_base or comp_base is null in function " << __FUNCTION__
         << " at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_INPUT_POINTER_NULL;
  }

  Frame userBase;  // default to be identity transform
  Vec t(mes_tool.segment(0, 3));
  Quaternion q = Quaternion::FromEigenVec(mes_tool.segment(3, 4));
  Frame userTool(q, t);

  // 8pt point method
  std::vector<Pose> ps(numJnts);
  std::vector<refPose> rps(numJnts);

  // first compute uncalibrated base frame
  SetUsingCalibratedModel(false);
  double sumYaw = 0, sumPitch = 0, sumRoll = 0;
  for (size_t i = 0; i < numJnts; i++) {
    int ret = JntToCart(jnt_base_measures.col(i), &ps[i]);
    if (ret < 0) {
      strs.str("");
      strs << GetName() << ":"
           << "FK error, code  " << ret
           << "can not do base frame calibration in" << __FUNCTION__
           << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return ret;
    }
    refPose default_rps;
    default_rps.setDefaultPose(ps[i]);
    // get refPose under default base and userTool, and canonical model
    default_rps.getPoseUnderNewRef(userBase, userTool, &rps[i]);
  }

  Rotation newr;
  orig_base->resize(7);
  if (numJnts == 3) {  // 3pt method
    Vec newCenter =
        rps[0].getTranslation();  // rps[0] is the origin of the datus reference
                                  // frame of workpiece
    (*orig_base)(0) = newCenter.x();
    (*orig_base)(1) = newCenter.y();
    (*orig_base)(2) = newCenter.z();

    Vec newX =
        (rps[1].getTranslation() - rps[0].getTranslation()).NormalizeVec();
    Vec tmpY = rps[2].getTranslation() - rps[1].getTranslation();
    Vec tmpY1 = tmpY - (tmpY.dot(newX)) * newX;
    Vec newY = tmpY1.NormalizeVec();
    Vec newZ = newX * newY;
    newr.UnitX(newX);
    newr.UnitY(newY);
    newr.UnitZ(newZ);

    // Rotation newr(newX, newY, newZ);
    Quaternion q;
    newr.GetQuaternion(&q);
    (*orig_base)(3) = q.w();
    (*orig_base)(4) = q.x();
    (*orig_base)(5) = q.y();
    (*orig_base)(6) = q.z();
    strs.str("");
    strs << GetName() << ":"
         << "3pt base frame computation: uncalibrated frame, trans="
         << newCenter.ToString() << ", rot=" << newr.ToString() << std::endl;
    LOG_INFO(strs);
  } else if (numJnts == 8) {  // 8 pt method
    // pt 0, 1 determines x axis on the right, 2,3 is parallel to x axis on the
    // left, pt 4, 5 on y axis on the top, and 6, 7 is parallel to y axis on the
    // bottom
    //  top_right is the intersection pt between line 01 and line 45,
    // bot_left is the intersection pt between line 23, and line 67
    Eigen::MatrixXd ptIn(3, numJnts);
    for (size_t i = 0; i < numJnts; i++) {
      Vec p = rps[i].getTranslation();
      ptIn.col(i) = p.ToEigenVec();
    }
    Eigen::Vector3d meanPt = ptIn.rowwise().mean();
    Eigen::MatrixXd tmp = ptIn.colwise() - meanPt;

    // using svd decompsition to find the normal of 8-pt plane
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        tmp.transpose(), Eigen::ComputeFullV | Eigen::ComputeFullU);
    Eigen::MatrixXd tmpV = svd.matrixV();
    Eigen::Vector3d pn = tmpV.col(2);
    Vec vn(pn);

    // now compute x and y
    Eigen::Vector3d newX1 = ptIn.col(1) - ptIn.col(0);
    Vec tx1(newX1);
    Vec vx1 = tx1.NormalizeVec();

    Eigen::Vector3d newX2 = ptIn.col(2) - ptIn.col(3);
    Vec tx2(newX2);
    Vec vx2 = tx2.NormalizeVec();
    Vec vx = (vx1 + vx2).NormalizeVec();

    Eigen::Vector3d newY1 = ptIn.col(5) - ptIn.col(4);
    Vec ty1(newY1);
    Vec vy1 = ty1.NormalizeVec();

    Eigen::Vector3d newY2 = ptIn.col(6) - ptIn.col(7);
    Vec ty2(newY2);
    Vec vy2 = ty2.NormalizeVec();
    Vec vy = (vy1 + vy2).NormalizeVec();

    Vec vz = vx * vy;
    // determine the actual normal of the 8pt plane
    if (vz.dot(vn) > 0) {
      vz = vn;
    } else {
      vz = -vn;
    }

    // obtain the actual vx and vy
    vx = (vx - vx.dot(vz) * vz).NormalizeVec();
    vy = vz * vx;
    newr.UnitX(vx);
    newr.UnitY(vy);
    newr.UnitZ(vz);
    // Rotation newr(vx, vy, vz);
    Quaternion q;
    newr.GetQuaternion(&q);
    (*orig_base)(3) = q.w();
    (*orig_base)(4) = q.x();
    (*orig_base)(5) = q.y();
    (*orig_base)(6) = q.z();

    // now obtain the origin of the workobj coordinate frame
    Eigen::Vector3d ex = vx.ToEigenVec();
    Eigen::Vector3d ey = vy.ToEigenVec();
    // now we can compute projected 8 poitns
    Eigen::VectorXd xproj = ex.transpose() * tmp;
    Eigen::VectorXd yproj = ey.transpose() * tmp;

    // compute the coordinates of 4 corner
    double ytop = (yproj(1) + yproj(0)) / 2.0;
    double ybot = (yproj(2) + yproj(3)) / 2.0;
    double xtop = (xproj(4) + xproj(5)) / 2.0;
    double xbot = (xproj(6) + xproj(7)) / 2.0;
    // center of the figure
    double xcenter = (xtop + xbot) / 2.0;
    double ycenter = (ytop + ybot) / 2.0;
    strs.str("");
    strs << GetName() << ":"
         << "8 pt method in uncalibrated model, x-length= " << fabs(xtop - xbot)
         << ", y-length= " << fabs(ytop - ybot) << std::endl;
    LOG_INFO(strs);

    Vec newCenter(meanPt);
    newCenter = newCenter + xcenter * vx + ycenter * vy;
    (*orig_base)(0) = newCenter.x();
    (*orig_base)(1) = newCenter.y();
    (*orig_base)(2) = newCenter.z();
  } else {
    strs.str("");
    strs << GetName() << ":"
         << "num of input joints in  " << __FUNCTION__ << " at line "
         << __LINE__ << " is out of scope" << std::endl;
    LOG_ERROR(strs);
    return -1;
  }

  // now start to compute com_base
  if (!isDHCalibrated_) {
    strs.str("");
    strs << GetName() << ":"
         << "robot is not calibrated, "
         << "can not do error compensation in calibrated base, but use the "
            "original base"
         << __FUNCTION__ << ", line " << __LINE__ << std::endl;
    LOG_INFO(strs);
    *comp_base = *orig_base;
    return 0;
  }

  // first compute calibrated base frame
  SetUsingCalibratedModel(true);
  for (size_t i = 0; i < numJnts; i++) {
    int ret = JntToCart(jnt_base_measures.col(i), &ps[i]);
    if (ret < 0) {
      strs.str("");
      strs << GetName() << ":"
           << "FK error, code  " << ret << "can not do error compensation in"
           << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return ret;
    }
    refPose default_rps;
    default_rps.setDefaultPose(ps[i]);
    // get refPose under default base and userTool, and calibrated model
    default_rps.getPoseUnderNewRef(userBase, userTool, &rps[i]);
  }

  comp_base->resize(7);
  if (numJnts == 3) {
    Vec newCenter =
        rps[0].getTranslation();  // rps[0] is the origin of the datus reference
                                  // frame of workpiece
    Vec newX =
        (rps[1].getTranslation() - rps[0].getTranslation()).NormalizeVec();
    Vec tmpY = rps[2].getTranslation() - rps[1].getTranslation();
    Vec tmpY1 = tmpY - (tmpY.dot(newX)) * newX;
    Vec newY = tmpY1.NormalizeVec();
    Vec newZ = newX * newY;
    newr.UnitX(newX);
    newr.UnitY(newY);
    newr.UnitZ(newZ);

    (*comp_base)(0) = newCenter.x();
    (*comp_base)(1) = newCenter.y();
    (*comp_base)(2) = newCenter.z();
    Quaternion q;
    newr.GetQuaternion(&q);
    (*comp_base)(3) = q.w();
    (*comp_base)(4) = q.x();
    (*comp_base)(5) = q.y();
    (*comp_base)(6) = q.z();
  } else if (numJnts == 8) {  // 8 pt method
    // pt 0, 1 determines x axis on the right, 2,3 is parallel to x axis on the
    // left, pt 4, 5 on y axis on the top, and 6, 7 is parallel to y axis on the
    // bottom
    //  top_right is the intersection pt between line 01 and line 45,
    // bot_left is the intersection pt between line 23, and line 67
    Eigen::MatrixXd ptIn(3, numJnts);
    for (size_t i = 0; i < numJnts; i++) {
      Vec p = rps[i].getTranslation();
      ptIn.col(i) = p.ToEigenVec();
    }
    Eigen::Vector3d meanPt = ptIn.rowwise().mean();
    Eigen::MatrixXd tmp = ptIn.colwise() - meanPt;

    // using svd decompsition to find the normal of 8-pt plane
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        tmp.transpose(), Eigen::ComputeFullV | Eigen::ComputeFullU);
    Eigen::MatrixXd tmpV = svd.matrixV();
    Eigen::Vector3d pn = tmpV.col(2);
    Vec vn(pn);

    // now compute x and y
    Eigen::Vector3d newX1 = ptIn.col(1) - ptIn.col(0);
    Vec tx1(newX1);
    Vec vx1 = tx1.NormalizeVec();

    Eigen::Vector3d newX2 = ptIn.col(2) - ptIn.col(3);
    Vec tx2(newX2);
    Vec vx2 = tx2.NormalizeVec();
    Vec vx = (vx1 + vx2).NormalizeVec();

    Eigen::Vector3d newY1 = ptIn.col(5) - ptIn.col(4);
    Vec ty1(newY1);
    Vec vy1 = ty1.NormalizeVec();

    Eigen::Vector3d newY2 = ptIn.col(6) - ptIn.col(7);
    Vec ty2(newY2);
    Vec vy2 = ty2.NormalizeVec();
    Vec vy = (vy1 + vy2).NormalizeVec();

    Vec vz = vx * vy;
    // determine the actual normal of the 8pt plane
    if (vz.dot(vn) > 0) {
      vz = vn;
    } else {
      vz = -vn;
    }

    // obtain the actual vx and vy
    vx = (vx - vx.dot(vz) * vz).NormalizeVec();
    vy = vz * vx;

    newr.UnitX(vx);
    newr.UnitY(vy);
    newr.UnitZ(vz);

    Quaternion q;
    newr.GetQuaternion(&q);
    (*comp_base)(3) = q.w();
    (*comp_base)(4) = q.x();
    (*comp_base)(5) = q.y();
    (*comp_base)(6) = q.z();

    // now obtain the origin of the workobj coordinate frame
    Eigen::Vector3d ex = vx.ToEigenVec();
    Eigen::Vector3d ey = vy.ToEigenVec();
    // now we can compute projected 8 poitns
    Eigen::VectorXd xproj = ex.transpose() * tmp;
    Eigen::VectorXd yproj = ey.transpose() * tmp;

    // compute the coordinates of 4 corner
    double ytop = (yproj(1) + yproj(0)) / 2.0;
    double ybot = (yproj(2) + yproj(3)) / 2.0;
    double xtop = (xproj(4) + xproj(5)) / 2.0;
    double xbot = (xproj(6) + xproj(7)) / 2.0;
    strs.str("");
    strs << GetName() << ":"
         << "8 pt method in calibrated model, x-length= " << fabs(xtop - xbot)
         << ", y-length= " << fabs(ytop - ybot) << std::endl;
    LOG_INFO(strs);

    // center of the figure
    double xcenter = (xtop + xbot) / 2.0;
    double ycenter = (ytop + ybot) / 2.0;

    Vec newCenter(meanPt);
    newCenter = newCenter + xcenter * vx + ycenter * vy;
    (*comp_base)(0) = newCenter.x();
    (*comp_base)(1) = newCenter.y();
    (*comp_base)(2) = newCenter.z();
  } else {
    strs.str("");
    strs << GetName() << ":"
         << "num of input joints in  " << __FUNCTION__ << " at line "
         << __LINE__ << " is out of scope" << std::endl;
    LOG_ERROR(strs);
    return -1;
  }
  return 0;
}

int SerialArmCalib::CpsCartPose(const refPose &p,
                                const Eigen::VectorXd &canonicalBase,
                                refPose *cp) {
  std::ostringstream strs;
  if (!cp || canonicalBase.rows() != 7) {
    strs.str("");
    strs << GetName() << ":"
         << "input cp is null in function " << __FUNCTION__ << " at line "
         << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_INPUT_POINTER_NULL;
  }
  Pose ps;
  p.getDefaultPose(&ps);  // get default pose w.r.t. default base and tool
                          // this is what robot kinematics does
  // in the following calculation, we all use uncalibrated, or canonical
  // model
  SetUsingCalibratedModel(false);
  // step 1: using canonical IK to compute ideal joint vector
  Eigen::VectorXd init_jnt;
  int ret = CartToJnt(ps, &init_jnt);
  if (ret < 0) {
    strs.str("");
    strs << GetName() << ":"
         << "IK error, code  " << ret << "can not do error compensation in"
         << __FUNCTION__ << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return ret;
  }
  // step 2, optimize jnt from init_jnt such that |f(cal_para, jnt) - ps|
  // is minimal
  Eigen::VectorXd opt_jnt;
  ret = OptimizeJntAfterCalib(init_jnt, p, &opt_jnt);
  if (ret < 0) {
    strs.str("");
    strs << GetName() << ":"
         << "OptimizeJntAfterCalib error, code  " << ret
         << "can not do error compensation in" << __FUNCTION__ << ", line "
         << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return ret;
  }
  // step 3, using canonical FK to find the compensated trajectory
  ret = JntToCart(opt_jnt, &ps);
  if (ret < 0) {
    strs.str("");
    strs << GetName() << ":"
         << "canonical FK error, code  " << ret
         << "can not do error compensation in" << __FUNCTION__ << ", line "
         << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return ret;
  }

  Frame cpTool;
  p.getTool(&cpTool);

  Vec t(canonicalBase.segment(0, 3));
  Quaternion q = Quaternion::FromEigenVec(canonicalBase.segment(3, 4));
  Frame cpBase(q, t);

  refPose default_rps;
  default_rps.setDefaultPose(ps);

  default_rps.getPoseUnderNewRef(cpBase, cpTool, cp);
  return 0;
}

int SerialArmCalib::CpsJnt(const refPose &p, Eigen::VectorXd *cq) {
  std::ostringstream strs;
  if (!cq) {
    strs.str("");
    strs << GetName() << ":"
         << "input cq is null in function " << __FUNCTION__ << " at line "
         << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_INPUT_POINTER_NULL;
  }
  if (!isDHCalibrated_) {
    strs.str("");
    strs << GetName() << ":"
         << "robot is not calibrated, "
         << "can not do error compensation in" << __FUNCTION__ << ", line "
         << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_COMP_WITHOUT_CALIB;
  }
  Pose ps;
  p.getDefaultPose(&ps);  // get default pose w.r.t. default base and tool
                          // this is what robot kinematics does
  // in the following calculation, we all use uncalibrated, or canonical
  // model
  SetUsingCalibratedModel(false);
  // step 1: using canonical IK to compute ideal joint vector
  Eigen::VectorXd init_jnt;
  int ret = CartToJnt(ps, &init_jnt);
  if (ret < 0) {
    strs.str("");
    strs << GetName() << ":"
         << "IK error, code  " << ret << "can not do error compensation in"
         << __FUNCTION__ << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return ret;
  }
  // step 2, optimize jnt from init_jnt such that |f(cal_para, *cq) - ps|
  // is minimal
  ret = OptimizeJntAfterCalib(init_jnt, p, cq);
  if (ret < 0) {
    strs.str("");
    strs << GetName() << ":"
         << "OptimizeJntAfterCalib error, code  " << ret
         << "can not do error compensation in" << __FUNCTION__ << ", line "
         << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return ret;
  }
  return 0;
}

int SerialArmCalib::CpsRobPath(
    const Eigen::VectorXd &calibBase, const Eigen::VectorXd &origBase,
    const Eigen::VectorXd &tool, const EigenDRef<Eigen::MatrixXd> &d_traj,
    EigenDRef<Eigen::MatrixXd> *md_traj, EigenDRef<Eigen::MatrixXd> *d_j_traj,
    EigenDRef<Eigen::MatrixXd> *md_j_traj, EigenDRef<Eigen::MatrixXd> *a_traj) {
  std::ostringstream strs;
  if (!md_traj || !d_j_traj || !md_j_traj || !a_traj) {
    strs.str("");
    strs << GetName() << ":"
         << "Input data pointer is null"
         << " so can not do calibration, in function " << __FUNCTION__
         << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_INPUT_POINTER_NULL;
  }
  if (!isDHCalibrated_) {
    strs.str("");
    strs << GetName() << ":"
         << "robot is not calibrated, "
         << "can not do error compensation in" << __FUNCTION__ << ", line "
         << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_COMP_WITHOUT_CALIB;
  }
  // generate base and tool frame from the input data
  if (calibBase.rows() != 7 || origBase.rows() != 7) {
    strs.str("");
    strs << GetName() << ":"
         << "Input Base data is not translation + quaternion"
         << " so can not do calibration, in function " << __FUNCTION__
         << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_INPUT_PARA_WRONG_DIM;
  }
  strs.str("");
  strs << GetName() << ":"
       << "In traj. compensation: calib base= " << calibBase
       << ", uncalibrated base= " << origBase << ", desired tool=" << tool
       << std::endl;
  LOG_INFO(strs);

  Vec bv(calibBase.segment(0, 3));
  Quaternion bq = Quaternion::FromEigenVec(calibBase.segment(3, 4));
  Frame cpBase(bq, bv);

  Vec tv(tool.segment(0, 3));
  Quaternion tq = Quaternion::FromEigenVec(tool.segment(3, 4));
  Frame cpTool(tq, tv);

  Vec bv_old(origBase.segment(0, 3));
  Quaternion bq_old = Quaternion::FromEigenVec(origBase.segment(3, 4));
  Frame oldBase(bq_old, bv_old);

  size_t numPts = d_traj.cols();
  size_t numRows = d_traj.rows();
  // resize output variables
  md_traj->resize(numRows, numPts);
  d_j_traj->resize(DoF_, numPts);
  md_j_traj->resize(DoF_, numPts);
  a_traj->resize(numRows, numPts);

  for (size_t i = 0; i < numPts; i++) {
    Eigen::VectorXd cur_pt = d_traj.col(i);
    Vec v(cur_pt.segment(0, 3));
    Quaternion quat = Quaternion::FromEigenVec(cur_pt.segment(3, 4));
    Frame goalFrame(quat, v);
    size_t inFlag = cur_pt(7);
    // for scara, there is only 1 branch flag: elbow (up or down), and for
    // six-axis robot, there are 3 flags
    std::vector<int> branchFlags;
    SingleInt2RobnuxBranch(inFlag, branchFlags);

    // convert joint turn data into ikJointTurns
    size_t tFlag = cur_pt(8);
    std::vector<int> ikJointTurns;
    SingleInt2RobnuxTurn(tFlag, DoF_, ikJointTurns);
    // for the moment we will assume all joint turns are 0
    refPose rps_init, rps_init1;
    Pose ps;
    rps_init.setFrame(goalFrame);
    rps_init.setBranchFlags(branchFlags);
    rps_init.setJointTurns(ikJointTurns);
    rps_init.setBase(cpBase);
    rps_init.setTool(cpTool);
    rps_init.getDefaultPose(&ps);  // get default pose w.r.t. default base and
                                   // tool this is what robot kinematics does
    // step 1: using canonical IK to compute ideal joint vector
    SetUsingCalibratedModel(false);
    Eigen::VectorXd init_jnt, init_jnt1;
    strs.str("");
    strs << GetName() << ", pt " << i << ", IK: default cart is "
         << ps.ToString(true) << std::endl;
    LOG_INFO(strs);
    int ret = CartToJnt(ps, &init_jnt);
    if (ret < 0) {
      strs.str("");
      strs << GetName() << " IK error, code  " << ret << ", cart is "
           << ps.ToString(false) << "can not do error compensation in"
           << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return ret;
    }

    strs.str("");
    strs << GetName() << ":"
         << " init_jnt=" << init_jnt << std::endl;
    LOG_INFO(strs);

    // for modules, their control often lies in joint space
    rps_init1.setFrame(goalFrame);
    rps_init1.setBranchFlags(branchFlags);
    rps_init1.setJointTurns(ikJointTurns);
    rps_init1.setBase(oldBase);
    rps_init1.setTool(cpTool);
    rps_init1.getDefaultPose(
        &ps);  // get default pose w.r.t. default base and tool

    strs.str("");
    strs << GetName() << ", pt " << i << ", IK: uncalib cart is "
         << ps.ToString(true) << std::endl;
    LOG_INFO(strs);
    ret = CartToJnt(ps, &init_jnt1);
    if (ret < 0) {
      strs.str("");
      strs << GetName() << " IK error, code  " << ret << ", uncalib cart is "
           << ps.ToString(false) << "can not do error compensation in"
           << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return ret;
    }

    strs.str("");
    strs << GetName() << ":"
         << " uncalib init_jnt=" << init_jnt1 << std::endl;
    LOG_INFO(strs);
    d_j_traj->col(i) = init_jnt1;

    // step 2, optimize jnt from init_jnt such that |f(calibrate_para, opt_jnt)
    // - ps| is minimal
    Eigen::VectorXd opt_jnt;
    // here rps_init is the desired traj. w.r.t calib base and  new tool
    ret = OptimizeJntAfterCalib(init_jnt, rps_init, &opt_jnt);
    if (ret < 0) {
      strs.str("");
      strs << GetName() << ":"
           << "OptimizeJntAfterCalib error, code  " << ret
           << ", for point index " << i
           << ", can not do error compension, using the original pose instead"
           << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      md_traj->col(i) = d_traj.col(i);  // set same as original one
      a_traj->col(i) = d_traj.col(i);   // set same as original one
      // return ret;
    } else {
      md_j_traj->col(i) = opt_jnt;
      strs.str("");
      strs << GetName() << ":"
           << " opt_jnt=" << opt_jnt << std::endl;
      LOG_INFO(strs);
      // step 3, using canonical FK to find the compensated trajectory
      ret = JntToCart(opt_jnt, &ps);
      if (ret < 0) {
        strs.str("");
        strs << GetName() << ":"
             << "canonical FK error, code  " << ret
             << "can not do error compensation in" << __FUNCTION__ << ", line "
             << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return ret;
      }

      refPose default_rps, rps;
      default_rps.setDefaultPose(ps);
      default_rps.getPoseUnderNewRef(
          oldBase, cpTool,
          &rps);  // get  relative pose w.r.t. oldBase, and new Tool

      md_traj->col(i) =
          rps.ToEigenVecPose();  // in robot app. program, we use oldBase, and
                                 // canonical kinematics

      // step 4, using actual FK to find the actual trajectory
      SetUsingCalibratedModel(true);
      ret = JntToCart(opt_jnt, &ps);
      if (ret < 0) {
        strs.str("");

        strs << GetName() << ":"
             << "canonical FK error, code  " << ret
             << "can not do error compensation in" << __FUNCTION__ << ", line "
             << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return ret;
      }

      default_rps.setDefaultPose(ps);
      default_rps.getPoseUnderNewRef(Frame(), cpTool, &rps);
      strs.str("");
      strs << GetName() << ":"
           << "under default base and new tool, comp rps after comp is "
           << rps.ToString(false) << std::endl;
      LOG_INFO(strs);
      default_rps.getPoseUnderNewRef(
          cpBase, cpTool, &rps);  // get relative to  new Base, which is used
                                  // for comparing with desired pose
      a_traj->col(i) = rps.ToEigenVecPose();
      strs.str("");
      strs << GetName() << ":"
           << "under new base and tool, desired rps is "
           << rps_init.ToString(false)
           << "under new Base and tool, comp rps is " << rps.ToString(false)
           << std::endl;
      LOG_INFO(strs);
    }
  }
  return 0;
}

int SerialArmCalib::HomotopyAlg(const Eigen::VectorXd &init_jnt0,
                                const Eigen::VectorXd &toolOffset,
                                Eigen::VectorXd *init_jnt) {
  std::ostringstream strs;
  if (!init_jnt) {
    strs.str("");
    strs << GetName() << ":"
         << "Input final_plane or final_tool_offset pointer is null"
         << " so can not do calibration, in function " << __FUNCTION__
         << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_INPUT_POINTER_NULL;
  }
  if (!isDHCalibrated_) {
    strs.str("");
    strs << GetName() << ":"
         << "robot is not calibrated, "
         << "can not do error compensation in" << __FUNCTION__ << ", line "
         << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_COMP_WITHOUT_CALIB;
  }
  Eigen::VectorXd jnt0 = init_jnt0;
  // compute the diff of
  Eigen::VectorXd diff_alpha = alpha_c_ - alpha_;
  Eigen::VectorXd diff_a = a_c_ - a_;
  Eigen::VectorXd diff_theta = theta_c_ - theta_;
  Eigen::VectorXd diff_d = d_c_ - d_;
  Eigen::VectorXd diff_beta = beta_c_ - beta_;
  double norm_dalpha = diff_alpha.norm();
  double norm_da = diff_a.norm();
  double norm_dtheta = diff_theta.norm();
  double norm_dd = diff_d.norm();
  double norm_dbeta = diff_beta.norm();

  int step1 =
      std::floor(std::max(std::max(norm_dalpha, norm_dtheta), norm_dbeta) /
                 DH_ANGULAR_EPSILON);
  int step2 = std::floor(std::max(norm_da, norm_dd) / DH_LINEAR_EPSILON);
  int step = std::max(step1, step2);
  Eigen::VectorXd step_diff_alpha = diff_alpha / step;
  Eigen::VectorXd step_diff_a = diff_a / step;
  Eigen::VectorXd step_diff_beta = diff_beta / step;
  Eigen::VectorXd step_diff_theta = diff_theta / step;
  Eigen::VectorXd step_diff_d = diff_d / step;

  // we need a diff para vector to put all above variation together
  Eigen::VectorXd var_para(5 * DoF_);
  for (size_t i = 0; i < DoF_; i++) {
    var_para(5 * i + 0) = step_diff_alpha(i);
    var_para(5 * i + 1) = step_diff_a(i);
    var_para(5 * i + 2) = step_diff_theta(i);
    var_para(5 * i + 3) = step_diff_d(i);
    var_para(5 * i + 4) = step_diff_beta(i);
  }
  Eigen::VectorXd kine_para(5 * DoF_), tmp_para;  // clearing kine_para
  // A djnt + B dpara = 0
  Eigen::MatrixXd A, B;
  for (int j = 0; j < step; j++) {
    // fill in the value of p_alpha, p_a, p_theta, p_d
    kine_para.segment(0, DoF_) = alpha_ + step_diff_alpha * j;
    kine_para.segment(DoF_, DoF_) = a_ + step_diff_a * j;
    kine_para.segment(2 * DoF_, DoF_) = theta_ + step_diff_theta * j;
    kine_para.segment(3 * DoF_, DoF_) = d_ + step_diff_d * j;
    kine_para.segment(4 * DoF_, DoF_) = beta_ + step_diff_beta * j;
    UpdateDH(kine_para, jnt0, &tmp_para);
    Eigen::MatrixXd Jp_t, Jp_r;
    Pose p;
    // compute the expected values from known canonical kinematic parameters
    // i.e., not-calibrated parameter set
    int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r, true);
    if (ret < 0) {
      return ret;
    }

    Rotation r_n = p.getRotation();
    Vec t_n = p.getTranslation();
    Vec tcp_offset(toolOffset.segment(0, 3));
    Vec t_e = r_n * tcp_offset + t_n;

    Eigen::Matrix3d t_e_hat;
    t_e.ToHat(&t_e_hat);
    // compute the translational Jacobian w.r.t. origin of default T.
    Eigen::MatrixXd Jt_tmp = Jp_t - t_e_hat * Jp_r;

    Eigen::MatrixXd Js_t, Js_r, Js_t1, Js_r1;
    // pick joint angle related sub jacobians, because we want to
    // find joint angles correspond to calibrated parameters
    PickSubJacobian(Jt_tmp, Jp_r, &Js_t, &Js_r, true);
    size_t rowTrans = Js_t.rows();
    size_t rowRot = Js_r.rows();
    size_t tolCols = Jt_tmp.cols();
    A.resize(rowTrans + rowRot, rowTrans + rowRot);
    B.resize(rowTrans + rowRot, tolCols);
    if (rowTrans > 0) {
      A.block(0, 0, rowTrans, rowTrans + rowRot) = Js_t;
    }
    if (rowRot > 0) {
      A.block(rowTrans, 0, rowRot, rowTrans + rowRot) = Js_r;
    }

    // pick translation and rotational jacobian for parameter part.
    PickSubJacobianForPara(Jt_tmp, Jp_r, &Js_t1, &Js_r1, true);
    rowTrans = Js_t1.rows();
    rowRot = Js_r1.rows();
    if (rowTrans > 0) {
      B.block(0, 0, rowTrans, tolCols) = Js_t1;
    }
    if (rowRot > 0) {
      B.block(rowTrans, 0, rowRot, tolCols) = Js_r1;
    }

    double detA = A.determinant();
    if (fabs(detA) < CALIB_SINGULAR_CONST) {
      strs.str("");
      strs << GetName() << ":"
           << "calibration regression matrix A is singular "
           << " even with depend columns removed "
           << " with detA= " << detA << " , in function " << __FUNCTION__
           << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_CALIB_JAC_SINGULAR;
    }
    Eigen::VectorXd delta_t = -A.inverse() * B * var_para;
    jnt0 += delta_t;
  }
  *init_jnt = jnt0;
  return 0;
}
int SerialArmCalib::OptimizeJntAfterCalib(const Eigen::VectorXd &init_jnt0,
                                          const refPose &ps,
                                          Eigen::VectorXd *opt_jnt) {
  std::ostringstream strs;
  if (!opt_jnt) {
    strs.str("");
    strs << GetName() << ":"
         << "Input final_plane or final_tool_offset pointer is null"
         << " so can not do calibration, in function " << __FUNCTION__
         << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_INPUT_POINTER_NULL;
  }
  if (!isDHCalibrated_) {
    strs.str("");
    strs << GetName() << ":"
         << "robot is not calibrated, "
         << "can not do error compensation in" << __FUNCTION__ << ", line "
         << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_COMP_WITHOUT_CALIB;
  }
  // retrieve tool info, and tool offset vector
  Frame userTool;
  ps.getTool(&userTool);
  Vec userToolOffset = userTool.getTranslation();
  Eigen::VectorXd eigToolOffset = userTool.ToEigenVecQuat();

  // convert ps to  cps (under default base frame, but under the same userTool)
  refPose cps;
  // base_off_ replaced by default base, because in traj. compensation, the
  // desired traj is always w.r.t. robot base (or default frame)
  ps.getPoseUnderNewRef(defaultBaseOff_, userTool, &cps);
  strs.str("");
  strs << GetName() << ":"
       << "before comp, under default base and  new tool, desired rps is "
       << cps.ToString(false) << std::endl;
  LOG_INFO(strs);
  // using homotopty method to find a rough solution to
  // f(calib_para, mid_jnt) ~= f(old_para, init_jnt)
  Eigen::VectorXd init_jnt(DoF_);
  if (HomotopyAlg(init_jnt0, eigToolOffset, &init_jnt) < 0) {
    strs.str("");
    strs << GetName() << ":"
         << "homotopy algorithm fails in function " << __FUNCTION__ << ", line "
         << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_COMP_HOMOTOPY_FAIL;
  }

  // first make a copy of init_jnt
  Eigen::VectorXd jnt_tmp = init_jnt, jnt_iter, jnt_best_iter;
  // desired rotation
  Rotation d_r = cps.getRotation();
  // desired yaw, pitch, and roll
  double yaw_d, pitch_d, roll_d;
  if (!d_r.GetEulerZYX(&yaw_d, &pitch_d, &roll_d)) {
    return -61;
  }
  // desired translation
  Vec d_t = cps.getTranslation();
  // step 1, Define the regression matrix   A dJoint = b
  // where \partial f / \partial Joint d\Joint = desired_pose - f(\joint), where
  // desired_pose  and f(Joint) are using trans+euler repre.
  Eigen::MatrixXd A;
  Eigen::VectorXd b;

  double estimation_err = std::numeric_limits<double>::max();
  unsigned int numSteps = CALIB_LINE_SEARCH_STEPS;
  double step_size = 1.0 / numSteps;
  int cur_iter = 0;
  Eigen::VectorXd kine_para, tmp_para;  // fill in calibrated DH set
  kine_para.segment(0, DoF_) = alpha_c_;
  kine_para.segment(DoF_, DoF_) = a_c_;
  kine_para.segment(2 * DoF_, DoF_) = theta_c_;
  kine_para.segment(3 * DoF_, DoF_) = d_c_;
  kine_para.segment(4 * DoF_, DoF_) = beta_c_;
  while (estimation_err > MAX_CALIB_MATCHING_ERR && cur_iter < MAX_CALIB_ITER) {
    cur_iter++;
    // recall tmp_para is the calibrated DH set
    UpdateDH(kine_para, jnt_tmp, &tmp_para);
    Eigen::MatrixXd Jp_t, Jp_r;
    Pose p;

    int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r, true);
    if (ret < 0) {
      return ret;
    }
    // we get R_0n, the rotation matrix between 0 and flage
    Rotation r_0n = p.getRotation();
    Rotation r_final = r_0n * userTool.getRotation();
    double yaw, pitch, roll;
    if (!r_final.GetEulerZYX(&yaw, &pitch, &roll)) {
      strs.str("");
      strs << GetName() << ":"
           << "GetEulerZYX fails in function " << __FUNCTION__ << ", at line "
           << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROT2EULER_SINGULAR;
    }

    Vec t_0n = p.getTranslation();
    Vec t_e = r_0n * userToolOffset + t_0n;
    Eigen::Matrix3d t_e_hat;
    t_e.ToHat(&t_e_hat);
    // compute the translational Jacobian w.r.t. origin of default T.
    Eigen::MatrixXd Jt_tmp = Jp_t - t_e_hat * Jp_r;
    Eigen::Vector3d errT = (d_t - t_e).ToEigenVec();
    Eigen::Vector3d errR(roll_d - roll, pitch_d - pitch, yaw_d - yaw);

    Eigen::Vector3d Eubd(roll_d, pitch_d, yaw_d);
    Eigen::Vector3d Eub(roll, pitch, yaw);
    // spatial angular variation = EulerDiff * [delta roll, delta pitch, delta
    // yaw]'
    Eigen::Matrix3d EulerDiff;
    Eigen::Vector3d col1(0, 0, 1);
    EulerDiff.col(2) = col1;
    Eigen::Vector3d col2(-sin(Eub(2)), cos(Eub(2)), 0);
    EulerDiff.col(1) = col2;
    Eigen::Vector3d col3(cos(Eub(2)) * cos(Eub(1)), sin(Eub(2)) * cos(Eub(1)),
                         -sin(Eub(1)));
    EulerDiff.col(0) = col3;

    double detEulerDiff = EulerDiff.determinant();
    if (fabs(detEulerDiff) < CALIB_SINGULAR_CONST) {
      strs.str("");
      strs << GetName() << ":"
           << "Euler spatial jacobian is singular"
           << " with detA= " << detEulerDiff << ", Eub=" << Eub
           << ", Eubd=" << Eubd << ", EulerDiff=" << EulerDiff
           << " , in function " << __FUNCTION__ << ", line " << __LINE__
           << std::endl;
      LOG_ERROR(strs);
      return -ERR_CALIB_JAC_SINGULAR;
    }
    Eigen::MatrixXd Jr_tmp =
        EulerDiff.inverse() * Jp_r;  //  Jr_tmp * d para = errR
    Eigen::MatrixXd Js_t, Js_r;
    // get joint vector related jacobian
    PickSubJacobian(Jt_tmp, Jr_tmp, &Js_t, &Js_r, true);

    size_t rowTrans = Js_t.rows();
    size_t rowRot = Js_r.rows();

    A.resize(rowTrans + rowRot, rowTrans + rowRot);
    b.resize(rowTrans + rowRot);
    A.block(0, 0, rowTrans, rowTrans + rowRot) = Js_t;
    A.block(rowTrans, 0, rowRot, rowTrans + rowRot) = Js_r;

    double tmp_err = PickCartErr(
        errT, errR, &b,
        true);  // given full error vector, pick those mathing with robot type
    Eigen::VectorXd delta_t(DoF_);

    double detA = A.determinant();
    if (fabs(detA) < CALIB_SINGULAR_CONST) {
      strs.str("");
      strs << GetName() << ":"
           << "calibration regression matrix A is singular, A= " << A
           << " even with depend columns removed "
           << " with detA= " << detA << " , in function " << __FUNCTION__
           << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_CALIB_JAC_SINGULAR;
    }
    delta_t = A.inverse() * b;
    jnt_best_iter = jnt_tmp;
    // dynamic step size adjustment
    for (size_t i = 0; i < numSteps; i++) {
      jnt_iter = jnt_tmp + (i + 1) * step_size * delta_t;
      UpdateDH(kine_para, jnt_iter, &tmp_para);
      Eigen::MatrixXd Jp_t1, Jp_r1;
      Pose p1;
      // compute the expected values from known canonical kinematic parameters
      // i.e., not-calibrated parameter set
      ret = CalcJacobian(tmp_para, &p1, &Jp_t1, &Jp_r1, true);
      if (ret < 0) {
        return ret;
      }

      // we get R_04, the rotation matrix between 0 and flange
      r_0n = p1.getRotation();
      r_final = r_0n * userTool.getRotation();
      if (!r_final.GetEulerZYX(&yaw, &pitch, &roll)) {
        strs.str("");
        strs << GetName() << ":"
             << "GetEulerZYX fails in function " << __FUNCTION__ << ", at line "
             << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return -ERR_ROT2EULER_SINGULAR;
      }
      t_e = r_0n * userToolOffset + p1.getTranslation();
      errT = (d_t - t_e).ToEigenVec();
      errR(0) = roll_d - roll;
      errR(1) = pitch_d - pitch;
      errR(2) = yaw_d - yaw;
      double err = PickCartErr(
          errT, errR, &b,
          true);  // given full error vector, pick those mathing with robot type
      if (err < tmp_err) {
        jnt_best_iter = jnt_iter;
        tmp_err = err;
      }
    }
    if (tmp_err >= estimation_err - MAX_CALIB_MATCHING_ERR) {
      if (numSteps < MAX_CALIB_STEPS) {
        step_size /= 2.0;
        numSteps *= 2;
      } else {
        break;
      }
    } else {
      estimation_err = tmp_err;
      jnt_tmp = jnt_best_iter;
    }
  }
  if (estimation_err > MAX_CALIB_MATCHING_ERR) {  // not convergent
    strs.str("");
    strs << GetName() << ":"
         << "parameter not convergent within maximal iter limit,"
         << "estimation_err at iter " << cur_iter << " is " << estimation_err
         << std::endl;
    LOG_ERROR(strs);
    // std::cout << "The desire pose is " << ps.ToString(false) << std::endl;
    return -ERR_COMP_ALG_DIVERGE;
  } else {
    strs.str("");
    strs << GetName() << ":"
         << "parameter convergent within maximal iter limit,"
         << "estimation_err at iter " << cur_iter << " is " << estimation_err
         << std::endl;
    LOG_INFO(strs);
  }
  *opt_jnt = jnt_tmp;
  return 0;
}

}  // namespace kinematics_lib
