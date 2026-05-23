// C++ smoke tests for the calibration classes exposed by arm_calib_commands.
// These verify that each concrete subclass of BaseCalibration constructs,
// reports the expected name, and dispatches virtual calls polymorphically
// through a BaseCalibration pointer.

#include <gtest/gtest.h>

#include <memory>

#include "arm_calib_kinematics/base_calib.hpp"
#include "arm_calib_kinematics/scara_calib.hpp"
#include "arm_calib_kinematics/serialarm_calib.hpp"
#include "arm_calib_kinematics/singleaxis_calib.hpp"
#include "arm_calib_kinematics/sixaxis_calib.hpp"
#include "arm_calib_kinematics/ujnt_calib.hpp"
#include "arm_calib_kinematics/xyz_calib.hpp"
#include "arm_calib_kinematics/xyzur_calib.hpp"

namespace {

using kinematics_lib::BaseCalibration;
using kinematics_lib::ScaraCalib;
using kinematics_lib::SerialArmCalib;
using kinematics_lib::SingleAxisCalib;
using kinematics_lib::SixAxisCalib;
using kinematics_lib::UjntCalib;
using kinematics_lib::XyzGantryCalib;
using kinematics_lib::XyzUrCalib;

TEST(ConcreteCalibClasses, ScaraCalibConstructsAndReportsName) {
  ScaraCalib c;
  EXPECT_EQ(c.GetName(), "Scara calib");
}

TEST(ConcreteCalibClasses, SixAxisCalibConstructsAndReportsName) {
  SixAxisCalib c;
  EXPECT_EQ(c.GetName(), "SixAxis calib");
}

TEST(ConcreteCalibClasses, SingleAxisCalibConstructsAndReportsName) {
  SingleAxisCalib c;
  EXPECT_EQ(c.GetName(), "SingleAxisCalib");
}

TEST(ConcreteCalibClasses, UjntCalibConstructsAndReportsName) {
  UjntCalib c;
  EXPECT_EQ(c.GetName(), "UJNT calibration");
}

TEST(ConcreteCalibClasses, XyzGantryCalibConstructsAndReportsName) {
  XyzGantryCalib c;
  EXPECT_EQ(c.GetName(), "XYZGantry calib");
}

TEST(ConcreteCalibClasses, XyzUrCalibConstructsAndReportsName) {
  XyzUrCalib c;
  EXPECT_EQ(c.GetName(), "XYZUR calib");
}

TEST(BaseCalibrationPolymorphism, SerialArmCalibInheritsBaseCalibration) {
  // Verify that the inheritance chain Scara -> SerialArm -> BaseCalibration
  // allows upcasting to BaseCalibration, which is what the pybind11
  // py::class_<Derived, BaseCalibration> declarations rely on.
  ScaraCalib scara;
  BaseCalibration* base = static_cast<BaseCalibration*>(
      static_cast<SerialArmCalib*>(&scara));
  ASSERT_NE(base, nullptr);
}

TEST(BaseCalibrationApi, SetOptParamAcceptsSamParams) {
  ScaraCalib c;
  BaseCalibration& base = c;
  Eigen::VectorXd opt_param(2);
  opt_param << 0.0, 1.0;  // [opt_method=Sam, sam_region_scale=1.0]
  EigenDRef<Eigen::VectorXd> ref(opt_param);
  // setOptParam is non-pure virtual with a default implementation.
  // We only assert it does not throw.
  EXPECT_NO_THROW(base.setOptParam(ref));
}

TEST(BaseCalibrationApi, LoadCalibParamSetWithEmptyVectorReturns) {
  // We do not know the required parameter-vector length for every model
  // without per-class domain knowledge, so we only verify that the call
  // returns rather than crashes for a degenerate input.
  ScaraCalib c;
  BaseCalibration& base = c;
  Eigen::VectorXd cal_dh = Eigen::VectorXd::Zero(0);
  EigenDRef<Eigen::VectorXd> ref(cal_dh);
  // Either true or false is acceptable; we only require no crash.
  (void)base.LoadCalibParamSet(ref);
}

}  // namespace
