#ifndef ROBNUX_UTILITIES_RPE_UTILITY_API_HPP_
#define ROBNUX_UTILITIES_RPE_UTILITY_API_HPP_
#include <eigen3/Eigen/Core>
#include "robnux_kdl_common/common_exportdecl.h"
#include <string>

namespace kinematics_lib {

class COMMON_API RPE_Utility {
 public:
  RPE_Utility();

  bool ReadCSVFile(const std::string fileName, Eigen::MatrixXd& data);
  bool WriteCSVFile(const std::string fileName, const Eigen::MatrixXd& data);
};

}  // namespace kinematics_lib
#endif  /* ROBNUX_UTILITIES_RPE_UTILITY_API_HPP_ */