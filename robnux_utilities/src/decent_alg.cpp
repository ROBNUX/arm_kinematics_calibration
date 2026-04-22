#include "robnux_utilities/decent_alg.hpp"
#include "robnux_kdl_common/common_constants.hpp"

using namespace ROBNUXLogging;
namespace kinematics_lib {

DecentAlg::DecentAlg()
    : eta_(0.1),
      alg_opt_(0),  // adam algorithm
      sam_region_scale_(1.0),
      beta1_(0.9),
      beta2_(0.999),  // initial step size
      t_(1),
      first_time_cond_opt_(true),
      initialized_(false) {}

DecentAlg::DecentAlg(const double eta, const double sam_region_scale,
                     const double beta1, const double beta2, const int alg_opt)
    : eta_(eta),
      alg_opt_(alg_opt),
      sam_region_scale_(sam_region_scale),
      beta1_(beta1),
      beta2_(beta2),
      t_(1),
      first_time_cond_opt_(true),
      initialized_(true) {}

bool DecentAlg::computeInvCond(const Eigen::MatrixXd& A, double& InvCond,
                               int& rank) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A);
  Eigen::VectorXd sgvalue = svd.singularValues();
  size_t numSing = sgvalue.size();
  rank = numSing;
  if (sgvalue(0) < JACOBIAN_MIN_SING) {
    InvCond = 0;
    rank = 0;  // set rank as 0, and invcond as 0, i.e. singular
    return true;
  }
  for (int i = numSing - 1; i >= 0; i--) {
    if (sgvalue(i) / sgvalue(0) < JACOBIAN_MIN_SING_RATIO) {
      rank--;
    } else {
      break;
    }
  }
  InvCond = sgvalue(numSing - 1) / sgvalue(0);
  return true;
}

void DecentAlg::RemoveColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove) {
  size_t numRows = matrix.rows();
  size_t numCols = matrix.cols() - 1;

  if (colToRemove <= numCols) {
    matrix.block(0, colToRemove, numRows, numCols - colToRemove) =
        matrix.rightCols(numCols - colToRemove);
    matrix.conservativeResize(numRows, numCols);
  }
}

bool DecentAlg::ReduceJacobian(const Eigen::MatrixXd& A1, Eigen::MatrixXd& A2,
                         std::vector<int>& A2_indices,
                         std::vector<int>& d_indices) {
  std::ostringstream strs;

  int init_rank;        // initial rank
  double init_iv_cond;  // initial inverse cond

  if (!computeInvCond(A1, init_iv_cond, init_rank)) {
    strs.str("");
    strs << "computeInvCond fails in function " << __FUNCTION__ << ", at line "
         << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return false;
  }

  strs.str("");
  strs << "Initial rank=" << init_rank
       << ", initial inverse condition number = " << init_iv_cond << std::endl;
  LOG_INFO(strs);

  // initial rank should >= 1
  if (init_rank < 1) {
    strs.str("");
    strs << "Init rank " << init_rank << " is less than 1 !" << std::endl;
    LOG_ERROR(strs);
    return false;
  }

  // now start to reduce columns and then find the one with best condition
  // number
  size_t numCols = A1.cols();

  A2_indices.resize(numCols);
  d_indices.clear();
  for (int i = 0; i < numCols; i++) {
    A2_indices.at(i) = i;
  }

  Eigen::MatrixXd B = A1;
  Eigen::MatrixXd bestB = B;
  while (numCols > init_rank) {
    // remove  1 col as a time
    double max_iv_cond = 0;
    int max_iv_cond_index = -1;

    for (size_t i = 0; i < numCols; i++) {
      Eigen::MatrixXd B1 = B;
      Eigen::VectorXd vCol = B1.col(i);
      RemoveColumn(B1, i);

      int rank;        // rank
      double iv_cond;  // inverse cond
      if (!computeInvCond(B1, iv_cond, rank)) {
        strs.str("");
        strs << "computeInvCond fails with matrix B1=" << B1 << ", in function "
             << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return false;
      }
      if (rank == init_rank && iv_cond >= max_iv_cond) {
        max_iv_cond_index = i;
        max_iv_cond = iv_cond;
        bestB = B1;
      }
      if (vCol.norm() < ZERO_COL_EPS) {  // if this col is already close to 0,
                                         // delete immediately
        max_iv_cond_index = i;
        max_iv_cond = iv_cond;
        bestB = B1;
        break;
      }
    }
    if (max_iv_cond_index < 0) {
      strs.str("");
      strs << "remove 1 col at a time didn't find the reduced matrix "
           << "max_iv_cond_index=" << max_iv_cond_index << ", in function "
           << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return false;
    }
    d_indices.push_back(A2_indices[max_iv_cond_index]);
    A2_indices.erase(A2_indices.begin() + max_iv_cond_index);

    B = bestB;
    numCols--;
    strs.str("");
    strs << "reduced matrix B=" << B << ", by removing column "
         << max_iv_cond_index << ", with best iv_cond=" << max_iv_cond
         << ", in function " << __FUNCTION__ << ", at line " << __LINE__
         << std::endl;
    LOG_INFO(strs);

    if (max_iv_cond > MIN_JAC_INV_COND_ENGOUGH) {
      strs.str("");
      strs << " max_iv_cond has been good enough,"
           << " so exit from the while loop  "
           << ", in function " << __FUNCTION__ << ", at line " << __LINE__
           << std::endl;
      LOG_INFO(strs);
      break;
    }
  }
  A2 = B;
  strs.str("");
  strs << "Final reduced matrix  A2=" << B << ", in function " << __FUNCTION__
       << ", at line " << __LINE__ << std::endl;
  strs << "Indep. column indices=";
  for (size_t i = 0; i < numCols; i++) {
    strs << A2_indices[i] << " ";
  }
  strs << std::endl;
  std::sort(d_indices.begin(), d_indices.end(), std::greater<size_t>());
  strs << "Dep. column indices=";
  for (size_t i = 0; i < d_indices.size(); i++) {
    strs << d_indices[i] << " ";
  }
  strs << std::endl;
  LOG_INFO(strs);
  return true;
}

bool DecentAlg::OptGradientVec(const Eigen::MatrixXd& A1,
                               const Eigen::VectorXd& b,
                               Eigen::VectorXd& para) {
  std::ostringstream strs;
  if (!initialized_) {
    strs.str("");
    strs << "parameter is not initialized, can not do optimization "
         << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return false;
  }

  size_t numParam = A1.cols();
  Eigen::VectorXd delta_p_new(numParam);
  delta_p_new.setZero();  // set to 0;

  size_t numParam1;
  Eigen::MatrixXd A2;
  if (first_time_cond_opt_) {
    if (!ReduceJacobian(A1, A2, ind_indices_, d_indices_)) {
      strs.str("");
      strs << "reduce Jacobian based upon maximizing inv condition fails "
           << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return false;
    }
    first_time_cond_opt_ = false;
    numParam1 = ind_indices_.size();
    adam_s_ = Eigen::VectorXd::Zero(numParam1);
  } else {
    A2 = A1;
    for (size_t i = 0; i < d_indices_.size(); i++) {
      // luckily, the vector of depend column indices in d_jacobian_cols_
      // are arranged from largest toward smallest, so we can continuously
      // call RemoveColumn
      RemoveColumn(A2, d_indices_[i]);
    }
  }

  if (numParam1 != A2.cols()) {
    strs.str("");
    strs << "reduce Jacobian based upon maximizing inv condition"
         << " leads to unmatch dimensions between A2 and ind_indices "
         << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return false;
  }

  Eigen::VectorXd delta_p_old(numParam1);
  delta_p_old.setZero();
  // scaling matrix
  Eigen::MatrixXd scaleD = Eigen::MatrixXd::Identity(numParam1, numParam1);

  // scaling cols of A
  Eigen::MatrixXd A = A2;
  for (size_t i = 0; i < numParam1; i++) {
    Eigen::VectorXd colV = A2.col(i);
    double scale = colV.norm();
    A.col(i) = colV / scale;
    scaleD(i, i) = 1 / scale;
  }
  Eigen::MatrixXd ATA = A.transpose() * A;
  Eigen::VectorXd grad = ATA.inverse() * A.transpose() * b;

  // square of gradient, used in adam-type algorithm
  Eigen::VectorXd grad_sq(numParam1);
  for (size_t i = 0; i < numParam1; i++) {
    grad_sq(i) = grad(i) * grad(i);
  }
  if (alg_opt_ == 0) {  // sam - type regular gradient decent
    if (t_ % 2 == 1) {  // return  scaled grad
      delta_p_old = -SAM_RHO * sam_region_scale_ * grad / grad.norm();
      strs.str("");
      strs << "SAM_RHO=" << SAM_RHO * sam_region_scale_
           << "grad norm=" << grad.norm() << ", sam init grad offset at step "
           << t_ << " = " << delta_p_old << std::endl;
      LOG_INFO(strs);
      //*para = delta_p_old;
      delta_p_old = scaleD * delta_p_old;
      for (size_t i = 0; i < numParam1; i++) {
        delta_p_new(ind_indices_[i]) = delta_p_old(i);
      }
      para = delta_p_new;
      t_++;
      return false;  // for recompute the sam grad
    } else {         // return the actual (worse case of batch grad)
      delta_p_old = eta_ * grad;
    }

  } else if (alg_opt_ == 1) {  // sam/adam RMSprop
    if (t_ % 2 == 1) {         // return  epsilon(w):= scaled grad
      delta_p_old = -SAM_RHO * sam_region_scale_ * grad / grad.norm();
      strs.str("");
      strs << "SAM_RHO=" << SAM_RHO * sam_region_scale_
           << "grad norm=" << grad.norm() << ", sam init grad offset at step "
           << t_ << " = " << delta_p_old << std::endl;
      LOG_INFO(strs);
      //*para = delta_p_old;
      delta_p_old = scaleD * delta_p_old;
      for (size_t i = 0; i < numParam1; i++) {
        delta_p_new(ind_indices_[i]) = delta_p_old(i);
      }
      para = delta_p_new;
      t_++;
      return false;  // for recompute the sam grad
    } else {         // return the actual (worse case of batch grad)
      // Moving average of the squared gradients
      adam_s_ = beta2_ * adam_s_ + (1 - beta2_) * grad_sq;
      double norm_sq = std::max(sqrt(adam_s_.norm()), EPSILON_ADAM_ALG);
      delta_p_old = eta_ * grad / norm_sq;
    }
  }
  delta_p_old = scaleD * delta_p_old;
  for (size_t i = 0; i < numParam1; i++) {
    delta_p_new(ind_indices_[i]) = delta_p_old(i);
  }
  para = delta_p_new;
  strs.str("");
  strs << "grad offset at step " << t_ << " = " << para << std::endl;
  t_++;
  LOG_INFO(strs);
  return true;
}

void DecentAlg::setParam(const double eta, const double sam_region_scale,
                         const double beta1, const double beta2,
                         const int alg_opt) {
  eta_ = eta;
  sam_region_scale_ = sam_region_scale;
  beta1_ = beta1;
  beta2_ = beta2;
  alg_opt_ = alg_opt;
  t_ = 1;
  initialized_ = true;
  first_time_cond_opt_ = true;
  ind_indices_.clear();
  d_indices_.clear();
}

}  // namespace kinematics_lib
