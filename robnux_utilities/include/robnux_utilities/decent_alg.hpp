/* 
 * File:   decent_alg.hpp, gradient-decent optimization algorithm, 
 * we try to optimize
 *  min| A delta x - b |
 * Author: leon
 *
 * Created on February 17, 2022, 4:14 PM
 */

#ifndef ROBNUX_UTILITIES_DECENT_ALG_HPP_
#define ROBNUX_UTILITIES_DECENT_ALG_HPP_
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>
#include "simple_motion_logger/Logger.h"
#include "robnux_kdl_common/common_exportdecl.h"
namespace kinematics_lib{
// if singular value is too small
static constexpr double JACOBIAN_MIN_SING = 1e-10;
// if singular value / max sing is too small, it will
// be considered not independent
static constexpr double JACOBIAN_MIN_SING_RATIO = 1e-6;
// if inverse cond > 0.01, we think this jocobian is acceptable
static constexpr double MIN_JAC_INV_COND_ENGOUGH = 1e-2;
// if a Jacobian col has too small norm, it will be removed from the matrix
static constexpr double ZERO_COL_EPS = 1e-18;
// decent algorithm step size
static constexpr double CALIB_DECENT_STEPSIZE = 0.00025;

// Fixed (non-adaptive) ridge/Tikhonov regularization applied to every
// alg_opt's ATA before inversion (alg_opt 0/1 -- alg_opt 2/LM uses its own
// separate, adaptive lm_lambda_ instead, sized to also control step
// magnitude, not just numerical safety). This exists purely to keep ATA
// invertible when a raw DH-perturbation column is structurally
// non-identifying (e.g. a joint whose alpha/theta perturbation has zero
// effect on tip position) -- Sam/Sam-Adam's own step size is controlled by
// SAM_RHO/eta_, not by this constant, so it is kept small relative to the
// column-normalized ATA's natural ~1 diagonal scale.
static constexpr double JACOBIAN_RIDGE_EPS = 1e-6;

// Levenberg-Marquardt initial/min/max damping factor (alg_opt=2). The
// regression's A columns are unit-normalized before ATA is formed, so
// diag(ATA) ~= 1 for well-conditioned columns -- these bounds are chosen
// relative to that normalized scale, not to raw DH-parameter units.
static constexpr double LM_LAMBDA_INIT = 1.0e-3;
static constexpr double LM_LAMBDA_MIN = 1.0e-7;
static constexpr double LM_LAMBDA_MAX = 1.0e6;
static constexpr double LM_LAMBDA_DECREASE = 0.5;
static constexpr double LM_LAMBDA_INCREASE = 3.0;

class COMMON_API DecentAlg {
 public:
     DecentAlg();

     /*
      * @eta step size of iteration (used by alg_opt 0 and 1; alg_opt 2
      *  ignores eta_ and instead sizes its step via the adaptive LM
      *  damping factor)
      * @sam_region_scale  sam-type algorithm
      * @beta1, beta2 step size for adam-type algorithm
      */
     DecentAlg(const double eta,
               const double sam_region_scale = 1.0,
               const double beta1 = 0.9,
               const double beta2 = 0.999,
               const int alg_opt = 0);


     void setParam(const double eta,
                   const double sam_region_scale = 1.0,
                   const double beta1 = 0.9,
                   const double beta2 = 0.999,
                   const int alg_opt = 0);

     /*
      * @brief find the optimal gradient vector
      * @param A, b in  optimization problem   min|A * para - b|
      * @para optimal gradient vector
      * alg_opt: 0 = Sam-type gradient descent, 1 = Sam-Adam-type (adaptive
      *   per-parameter step via bias-corrected first/second moment
      *   estimates), 2 = Levenberg-Marquardt damped Gauss-Newton (no SAM
      *   probing; each call directly returns a full adaptively-damped step,
      *   and always returns true)
      */
     bool OptGradientVec(const Eigen::MatrixXd &A, const Eigen::VectorXd &b,
                      Eigen::VectorXd& para);

     /* @brief reduction of Jacobian by condition number
        * @A input Jacobian matrix
        * @B output reduced Jacobian matrix
        * @B_indices are indendent indices, while d_indices are dependent indices
        * NOTE: used by OptGradientVec for alg_opt 0/1 (Sam/Sam-Adam) only.
        * alg_opt 2 (LM) deliberately skips this and regularizes via its own
        * adaptive lm_lambda_ instead -- tried the same for 0/1 too (a fixed
        * JACOBIAN_RIDGE_EPS in place of this reduction), but measured it as
        * strictly worse (much slower convergence for Sam, a real accuracy
        * regression for Sam-Adam), since their step size is controlled by
        * separate, non-adaptive mechanisms that don't react to a
        * poorly-conditioned full Jacobian the way LM's damping does. This
        * greedy, cached-once reduction does have a known limitation (can
        * permanently block a genuinely identifiable parameter from
        * converging -- see memory/project_calib_bug_hunt.md's d[2] finding)
        * but is still the better tradeoff for 0/1 as currently implemented.
      */
     bool ReduceJacobian(const Eigen::MatrixXd& A,
                         Eigen::MatrixXd& B,
                         std::vector<int>& B_indices,
                         std::vector<int>& d_indices);

     /* @brief compute Inverse condition number as well as rank of a matrix
        * @A input matrix
        * @InvCond output inverse condition number
        * @rank output rank of the matrix
      */
     bool computeInvCond(const Eigen::MatrixXd& A, double& InvCond, int& rank);

     /* @brief remove Jacobian matrix column
        * @matrix input matrix
        * @colToRemove column to remove
      */
     void RemoveColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove);

 private:
  /* @brief Levenberg-Marquardt damped Gauss-Newton step (alg_opt_==2).
     Deliberately bypasses ReduceJacobian: LM's damping term (ATA + lambda*I)
     regularizes rank-deficient/ill-conditioned columns directly, so there is
     no need to greedily and permanently drop columns from a single
     early-iteration snapshot the way the SAM path's cached ReduceJacobian
     result does (that caching is why some genuinely-identifiable DH
     parameters, e.g. a SCARA's d[2], can permanently fail to converge under
     alg_opt 0/1 -- see memory/project_calib_bug_hunt.md).
     @param A1, b in optimization problem min|A1 * para - b|
     @para output step
     @return always true (no probing step -- every call returns a usable,
       directly-appliable full step)
   */
  bool OptGradientVecLM(const Eigen::MatrixXd& A1, const Eigen::VectorXd& b,
                        Eigen::VectorXd& para);

  double eta_;  // decay step size

  // option used in the algorithm (choose which algorithm:
  int alg_opt_;

  // scaling coef of sam region
  double sam_region_scale_;
  //  iteration id
  //int iter_;

  // back search constants in nonlinear optimization algorithm
  double beta1_; // beta1 in adam alg.
  double beta2_;  // beta2 in adam alg.

  // cycle indices
  size_t t_;


  // adam first and second momenta
  Eigen::VectorXd adam_v_, adam_s_;

  // flag on first time computing the rank of the matrix and do col reduction
  bool first_time_cond_opt_;

  // initialized
  bool initialized_;

  // indices of independent and dependent column indices
  std::vector<int> ind_indices_, d_indices_;

  // Levenberg-Marquardt state (alg_opt_==2 only)
  double lm_lambda_;
  double lm_prev_cost_;
  bool lm_first_call_;
};
}

#endif  /* ROBNUX_UTILITIES_DECENT_ALG_HPP_ */

