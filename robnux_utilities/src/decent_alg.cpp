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
      initialized_(false),
      lm_lambda_(LM_LAMBDA_INIT),
      lm_prev_cost_(0.0),
      lm_first_call_(true) {}

DecentAlg::DecentAlg(const double eta, const double sam_region_scale,
                     const double beta1, const double beta2, const int alg_opt)
    : eta_(eta),
      alg_opt_(alg_opt),
      sam_region_scale_(sam_region_scale),
      beta1_(beta1),
      beta2_(beta2),
      t_(1),
      first_time_cond_opt_(true),
      initialized_(true),
      lm_lambda_(LM_LAMBDA_INIT),
      lm_prev_cost_(0.0),
      lm_first_call_(true) {}

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

  if (alg_opt_ == 2) {
    return OptGradientVecLM(A1, b, para);
  }

  size_t numParam = A1.cols();
  Eigen::VectorXd delta_p_new(numParam);
  delta_p_new.setZero();  // set to 0;

  // Tried dropping ReduceJacobian for alg_opt 0/1 too (mirroring alg_opt 2
  // / LM's approach), relying only on JACOBIAN_RIDGE_EPS below for
  // numerical safety -- measurably worse: Sam's convergence slowed enough
  // to blow through a 300s test budget, and Sam-Adam picked up a real
  // accuracy regression. LM's lambda_ is *adaptive* and doubles as both a
  // numerical-safety net and a step-size limiter reacting to conditioning;
  // Sam/Sam-Adam's step size is controlled by separate, non-adaptive
  // mechanisms (SAM_RHO/eta_/moment estimates) that don't react to a
  // poorly-conditioned full Jacobian the way LM's damping does, so a small
  // fixed ridge alone isn't a substitute here. Keep the cached-once
  // ReduceJacobian reduction for 0/1 (see its own known d[2]-type
  // limitation in memory/project_calib_bug_hunt.md); only alg_opt 2 skips
  // it.
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
    adam_v_ = Eigen::VectorXd::Zero(numParam1);
  } else {
    A2 = A1;
    for (size_t i = 0; i < d_indices_.size(); i++) {
      // luckily, the vector of depend column indices in d_jacobian_cols_
      // are arranged from largest toward smallest, so we can continuously
      // call RemoveColumn
      RemoveColumn(A2, d_indices_[i]);
    }
    // ind_indices_ is cached from the first call's ReduceJacobian and stays
    // valid (column reduction is only computed once); numParam1 must be
    // re-derived from it here too, otherwise it's left uninitialized on
    // every call after the first, and the size check below fails randomly.
    numParam1 = ind_indices_.size();
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

  // scaling cols of A (columns with ~zero norm -- i.e. a DH perturbation
  // direction this Jacobian is structurally blind to -- are left unscaled;
  // JACOBIAN_RIDGE_EPS below keeps ATA invertible regardless)
  Eigen::MatrixXd A = A2;
  for (size_t i = 0; i < numParam1; i++) {
    Eigen::VectorXd colV = A2.col(i);
    double scale = colV.norm();
    if (scale < ZERO_COL_EPS) {
      scale = 1.0;
    }
    A.col(i) = colV / scale;
    scaleD(i, i) = 1 / scale;
  }
  Eigen::MatrixXd ATA = A.transpose() * A +
      JACOBIAN_RIDGE_EPS * Eigen::MatrixXd::Identity(numParam1, numParam1);
  Eigen::VectorXd grad = ATA.ldlt().solve(A.transpose() * b);

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
      // True per-parameter Adam: first moment (momentum, beta1_) and second
      // moment (RMSprop, beta2_), each applied elementwise -- NOT a single
      // global scalar norm, which would apply the same adaptive scale to
      // every DH parameter regardless of its own gradient history and
      // defeat the entire point of a per-parameter adaptive method.
      // Bias-corrected (standard Adam) so beta2_=0.999's ~1000-step warm-up
      // time doesn't dominate a calibration run that only takes tens of
      // actual steps.
      adam_v_ = beta1_ * adam_v_ + (1.0 - beta1_) * grad;
      adam_s_ = beta2_ * adam_s_ + (1.0 - beta2_) * grad_sq;
      double adam_t = double(t_ / 2);  // count of actual steps so far, incl. this one
      Eigen::VectorXd v_hat = adam_v_ / (1.0 - std::pow(beta1_, adam_t));
      Eigen::VectorXd s_hat = adam_s_ / (1.0 - std::pow(beta2_, adam_t));
      delta_p_old.resize(numParam1);
      for (size_t i = 0; i < numParam1; i++) {
        double denom = std::max(std::sqrt(s_hat(i)), EPSILON_ADAM_ALG);
        delta_p_old(i) = eta_ * v_hat(i) / denom;
      }
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

bool DecentAlg::OptGradientVecLM(const Eigen::MatrixXd& A1,
                                 const Eigen::VectorXd& b,
                                 Eigen::VectorXd& para) {
  std::ostringstream strs;
  size_t numParam = A1.cols();

  // Exclude columns with negligible effect (near-zero absolute norm, or
  // near-zero *relative* to the largest column) from the solve entirely,
  // rather than normalizing them in place. A column that is nominally zero
  // but not exactly so (floating-point roundoff from the FK/Jacobian chain
  // -- e.g. a SCARA's theta[3]/a[3], which have zero true effect on tip
  // *position* since they're past the last position-relevant joint) would
  // otherwise get divided by its own tiny norm, amplifying pure noise up
  // to unit scale and contaminating the regression as if it were real
  // signal -- confirmed concretely: exactly this caused a calibrated
  // theta[3] to diverge to ~1e11 rad once lm_lambda_ shrank enough on
  // later, well-converged iterations (position error stayed fine only
  // because theta[3] doesn't affect position at all -- a lucky accident
  // of this parameterization, not something to rely on). Reuses
  // JACOBIAN_MIN_SING_RATIO (the same relative-negligibility threshold
  // ReduceJacobian/computeInvCond already use) for consistency. Excluded
  // columns always get an exact zero step.
  std::vector<double> col_norms(numParam);
  double max_norm = 0.0;
  for (size_t i = 0; i < numParam; i++) {
    col_norms[i] = A1.col(i).norm();
    max_norm = std::max(max_norm, col_norms[i]);
  }
  std::vector<size_t> active;
  for (size_t i = 0; i < numParam; i++) {
    if (col_norms[i] > ZERO_COL_EPS &&
        col_norms[i] > JACOBIAN_MIN_SING_RATIO * max_norm) {
      active.push_back(i);
    }
  }
  if (active.empty()) {
    strs.str("");
    strs << "LM: every column is structurally uninformative (position-"
         << "invisible), cannot compute a step, in " << __FUNCTION__
         << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    para = Eigen::VectorXd::Zero(numParam);
    t_++;
    return true;
  }

  size_t numActive = active.size();
  Eigen::MatrixXd A(A1.rows(), numActive);
  Eigen::VectorXd scale_active(numActive);
  for (size_t i = 0; i < numActive; i++) {
    scale_active(i) = col_norms[active[i]];
    A.col(i) = A1.col(active[i]) / scale_active(i);
  }

  Eigen::MatrixXd ATA = A.transpose() * A;
  Eigen::VectorXd Atb = A.transpose() * b;

  // The per-column norm filter above only catches columns that are
  // individually near-zero -- it does NOT catch a *group* of columns that
  // are each individually normal-sized but nearly collinear with each
  // other (e.g. a SCARA's d[0..3]: since alpha=0 for every joint, each
  // d[i] is a pure additive Z-translation, so their Jacobian columns are
  // all nearly parallel). For such a group, ATA has a near-zero
  // *eigenvalue* (not a near-zero diagonal entry), and as lm_lambda_
  // shrinks because *other*, well-identified directions are converging
  // nicely, any tiny genuine signal leaking into that near-null direction
  // gets amplified without bound -- confirmed concretely: this produced
  // calibrated d values of hundreds of meters for a real SCARA dataset
  // while still reporting a deceptively good held-out position error
  // (the huge, physically-nonsensical components happened to nearly
  // cancel in their *combined* effect on this particular test set, not
  // because the fit is actually trustworthy).
  //
  // Fix: derive a lambda *floor* from A's own singular values so that
  // (ATA + lambda*I)'s worst-case inverse condition number never drops
  // below MIN_JAC_INV_COND_ENGOUGH (the same acceptability threshold
  // ReduceJacobian/computeInvCond already use elsewhere), independent of
  // how far the globally-adaptive lm_lambda_ has shrunk. Solving
  // (sigma_min^2+lambda)/(sigma_max^2+lambda) >= r for lambda gives
  // lambda >= (r*sigma_max^2 - sigma_min^2) / (1-r).
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A);
  Eigen::VectorXd sv = svd.singularValues();
  double sigma_max = sv(0);
  double sigma_min = sv(sv.size() - 1);
  double lambda_floor = 0.0;
  double denom = 1.0 - MIN_JAC_INV_COND_ENGOUGH;
  if (denom > 0.0) {
    double numerator = MIN_JAC_INV_COND_ENGOUGH * sigma_max * sigma_max -
                       sigma_min * sigma_min;
    lambda_floor = std::max(0.0, numerator / denom);
  }

  // Adapt lambda from the residual-cost trend across successive calls: the
  // caller always rebuilds A/b from the post-step parameters before calling
  // again, so a comparison against the previous call's cost is exactly the
  // classic LM accept/reject signal, with no change needed to the calling
  // convention. A shrinking cost means the local (damped) linear model is
  // trustworthy -> shrink lambda toward a fuller Gauss-Newton step; a
  // growing cost means the last step overshot -> grow lambda toward a more
  // conservative, gradient-descent-like step. (The *caller's* own
  // previous_err/estimation_err outer-loop check remains the final
  // safety net that discards a bad step's DH values entirely.)
  double cur_cost = b.squaredNorm();
  if (!lm_first_call_) {
    if (cur_cost < lm_prev_cost_) {
      lm_lambda_ = std::max(lm_lambda_ * LM_LAMBDA_DECREASE, LM_LAMBDA_MIN);
    } else {
      lm_lambda_ = std::min(lm_lambda_ * LM_LAMBDA_INCREASE, LM_LAMBDA_MAX);
    }
  }
  lm_prev_cost_ = cur_cost;
  lm_first_call_ = false;

  // effective_lambda (not lm_lambda_ itself) is what actually gets applied
  // -- the floor is a per-call, conditioning-driven correction and must
  // not be allowed to persist/ratchet lm_lambda_'s own adaptive trajectory
  double effective_lambda = std::max(lm_lambda_, lambda_floor);
  Eigen::MatrixXd damped =
      ATA + effective_lambda * Eigen::MatrixXd::Identity(numActive, numActive);
  // damped is SPD for lambda > 0 -- LDLT is both faster and more numerically
  // robust here than an explicit inverse, particularly for small lambda
  // where ATA alone may still be near-singular.
  Eigen::VectorXd delta_active = damped.ldlt().solve(Atb);

  para = Eigen::VectorXd::Zero(numParam);
  for (size_t i = 0; i < numActive; i++) {
    para(active[i]) = delta_active(i) / scale_active(i);
  }

  strs.str("");
  strs << "LM step " << t_ << ": lambda=" << lm_lambda_
       << ", lambda_floor=" << lambda_floor
       << ", effective_lambda=" << effective_lambda << ", cost=" << cur_cost
       << ", active_cols=" << numActive << "/" << numParam
       << ", delta=" << para.transpose() << std::endl;
  LOG_INFO(strs);
  t_++;
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
  lm_lambda_ = LM_LAMBDA_INIT;
  lm_prev_cost_ = 0.0;
  lm_first_call_ = true;
}

}  // namespace kinematics_lib
