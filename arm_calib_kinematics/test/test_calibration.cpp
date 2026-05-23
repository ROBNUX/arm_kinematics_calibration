// Smoke tests for arm_calib_kinematics: construction of every concrete
// calibration class, GetName, polymorphic upcasts through BaseCalibration,
// setOptParam, and parameter-set round-trip.

#include <gtest/gtest.h>

#include <memory>

#include <eigen3/Eigen/Core>

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

// ----------------------------------------------------------------------------
// Constructors + GetName
// ----------------------------------------------------------------------------

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

TEST(ConcreteCalibClasses, SerialArmCalibConstructs) {
  SerialArmCalib c;
  // SerialArmCalib's GetName returns "serial arm calibration".
  EXPECT_EQ(c.GetName(), "serial arm calibration");
}

// ----------------------------------------------------------------------------
// Polymorphism via BaseCalibration
// ----------------------------------------------------------------------------

TEST(BaseCalibrationPolymorphism, ScaraCalibUpcastsToBaseCalibration) {
  // Verify that the inheritance chain Scara -> SerialArm -> BaseCalibration
  // allows upcasting to BaseCalibration. This is the upcast the pybind11
  // bindings rely on.
  ScaraCalib scara;
  BaseCalibration* base =
      static_cast<BaseCalibration*>(static_cast<SerialArmCalib*>(&scara));
  ASSERT_NE(base, nullptr);
}

TEST(BaseCalibrationPolymorphism, AllSerialArmCalibsUpcast) {
  std::unique_ptr<SerialArmCalib> samples[] = {
      std::unique_ptr<SerialArmCalib>(new ScaraCalib()),
      std::unique_ptr<SerialArmCalib>(new SixAxisCalib()),
      std::unique_ptr<SerialArmCalib>(new SingleAxisCalib()),
      std::unique_ptr<SerialArmCalib>(new UjntCalib()),
      std::unique_ptr<SerialArmCalib>(new XyzGantryCalib()),
  };
  for (auto& p : samples) {
    BaseCalibration* base = static_cast<BaseCalibration*>(p.get());
    ASSERT_NE(base, nullptr);
  }
}

TEST(BaseCalibrationPolymorphism, XyzUrCalibDirectlyUpcasts) {
  XyzUrCalib calib;
  BaseCalibration* base = &calib;
  ASSERT_NE(base, nullptr);
}

// ----------------------------------------------------------------------------
// setOptParam
// ----------------------------------------------------------------------------

TEST(BaseCalibrationApi, SetOptParamWithSamConfig) {
  ScaraCalib c;
  BaseCalibration& base = c;
  Eigen::VectorXd opt_param(2);
  opt_param << 0.0, 1.0;  // opt_method = Sam, sam_region_scale = 1.0
  EigenDRef<Eigen::VectorXd> ref(opt_param);
  EXPECT_NO_THROW(base.setOptParam(ref));
}

TEST(BaseCalibrationApi, SetOptParamWithSamAdamConfig) {
  SixAxisCalib c;
  BaseCalibration& base = c;
  Eigen::VectorXd opt_param(2);
  opt_param << 1.0, 2.5;  // opt_method = Sam-Adam, region scale = 2.5
  EigenDRef<Eigen::VectorXd> ref(opt_param);
  EXPECT_NO_THROW(base.setOptParam(ref));
}

// ----------------------------------------------------------------------------
// LoadCalibParamSet — only that the call returns rather than crashes for a
// degenerate input; the correct parameter-vector length depends on the model.
// ----------------------------------------------------------------------------

TEST(BaseCalibrationApi, LoadCalibParamSetWithEmptyVectorReturns) {
  ScaraCalib c;
  BaseCalibration& base = c;
  Eigen::VectorXd cal_dh = Eigen::VectorXd::Zero(0);
  EigenDRef<Eigen::VectorXd> ref(cal_dh);
  (void)base.LoadCalibParamSet(ref);
}

TEST(BaseCalibrationApi, ResetCalibrationDoesNotThrow) {
  // ResetCalibration is declared on SerialArmCalib (overrides
  // BaseKinematicMap::ResetCalibration). It should be safe to call on a
  // freshly-constructed instance.
  ScaraCalib c;
  EXPECT_NO_THROW(c.ResetCalibration());
}

}  // namespace
