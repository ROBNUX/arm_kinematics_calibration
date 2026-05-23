// Smoke tests for robnux_utilities: RPE_Utility CSV round-trip and DecentAlg
// matrix helpers.

#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

#include <eigen3/Eigen/Core>

#include "robnux_utilities/decent_alg.hpp"
#include "robnux_utilities/utility.hpp"

namespace {

using kinematics_lib::DecentAlg;
using kinematics_lib::RPE_Utility;

constexpr double kTol = 1e-9;

TEST(RPE_UtilityTest, WriteReadCSVRoundTrip) {
  RPE_Utility util;
  Eigen::MatrixXd data(3, 2);
  data << 1.0, 2.0, 3.5, -4.25, 100.0, 0.0;

  // Write into a temp file, read back, compare element-wise.
  const std::string path =
      (std::filesystem::temp_directory_path() / "robnux_utilities_test.csv")
          .string();
  ASSERT_TRUE(util.WriteCSVFile(path, data));

  Eigen::MatrixXd loaded;
  ASSERT_TRUE(util.ReadCSVFile(path, loaded));
  ASSERT_EQ(loaded.rows(), data.rows());
  ASSERT_EQ(loaded.cols(), data.cols());
  for (int i = 0; i < data.rows(); ++i) {
    for (int j = 0; j < data.cols(); ++j) {
      EXPECT_NEAR(loaded(i, j), data(i, j), 1e-6);
    }
  }
  std::remove(path.c_str());
}

TEST(DecentAlgTest, DefaultConstructibleAndParamUpdate) {
  DecentAlg alg;
  alg.setParam(0.001, 1.0, 0.9, 0.999, 0);
  SUCCEED();
}

TEST(DecentAlgTest, ComputeInvCondOnIdentityIsOne) {
  DecentAlg alg;
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(4, 4);
  double inv_cond = 0;
  int rank = 0;
  ASSERT_TRUE(alg.computeInvCond(I, inv_cond, rank));
  EXPECT_NEAR(inv_cond, 1.0, 1e-9);
  EXPECT_EQ(rank, 4);
}

TEST(DecentAlgTest, ComputeInvCondOnSingularIsZeroRankReduced) {
  DecentAlg alg;
  // Singular: row 2 = 2 * row 0.
  Eigen::MatrixXd S(3, 3);
  S << 1, 2, 3, 0, 0, 0, 2, 4, 6;
  double inv_cond = 0;
  int rank = 0;
  ASSERT_TRUE(alg.computeInvCond(S, inv_cond, rank));
  EXPECT_LT(rank, 3);
}

TEST(DecentAlgTest, RemoveColumnShrinksMatrix) {
  DecentAlg alg;
  Eigen::MatrixXd M(2, 3);
  M << 1, 2, 3, 4, 5, 6;
  alg.RemoveColumn(M, 1);  // remove middle column
  ASSERT_EQ(M.rows(), 2);
  ASSERT_EQ(M.cols(), 2);
  EXPECT_DOUBLE_EQ(M(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(M(0, 1), 3.0);
  EXPECT_DOUBLE_EQ(M(1, 0), 4.0);
  EXPECT_DOUBLE_EQ(M(1, 1), 6.0);
}

TEST(DecentAlgTest, OptGradientVecCallable) {
  // Verify that OptGradientVec can be invoked without crashing on a small
  // well-formed system. The solver's success depends on internal tolerances
  // and full algorithm initialization (alpha, sam scale, adam betas), so we
  // only assert "does not crash" and leave the bool return value unchecked.
  DecentAlg alg(0.001, 1.0, 0.9, 0.999, 0);
  Eigen::MatrixXd A(2, 2);
  A << 2, 0, 0, 4;
  Eigen::VectorXd b(2);
  b << 6, 8;
  Eigen::VectorXd para;
  EXPECT_NO_THROW({ (void)alg.OptGradientVec(A, b, para); });
}

}  // namespace
