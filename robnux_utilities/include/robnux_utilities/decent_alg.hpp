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

class COMMON_API DecentAlg {
 public:
     DecentAlg();
     
     /* 
      * @eta step size of iteration
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
      */
     bool OptGradientVec(const Eigen::MatrixXd &A, const Eigen::VectorXd &b,
                      Eigen::VectorXd& para);

     /* @brief reduction of Jacobian by condition number
        * @A input Jacobian matrix
        * @B output reduced Jacobian matrix
        * @B_indices are indendent indices, while d_indices are dependent indices
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
};
}

#endif  /* ROBNUX_UTILITIES_DECENT_ALG_HPP_ */

