#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "arm_calib_kinematics/base_calib.hpp"
#include "arm_calib_kinematics/scara_calib.hpp"
#include "arm_calib_kinematics/serialarm_calib.hpp"
#include "arm_calib_kinematics/singleaxis_calib.hpp"
#include "arm_calib_kinematics/sixaxis_calib.hpp"
#include "arm_calib_kinematics/ujnt_calib.hpp"
#include "arm_calib_kinematics/xyz_calib.hpp"
#include "arm_calib_kinematics/xyzur_calib.hpp"
#include "robnux_kdl_common/pose.hpp"

namespace py = pybind11;
using kinematics_lib::BaseCalibration;
using kinematics_lib::refPose;
using kinematics_lib::ScaraCalib;
using kinematics_lib::SerialArmCalib;
using kinematics_lib::SingleAxisCalib;
using kinematics_lib::SixAxisCalib;
using kinematics_lib::UjntCalib;
using kinematics_lib::XyzGantryCalib;
using kinematics_lib::XyzUrCalib;

namespace {

// Wrap methods that write outputs through Eigen reference parameters into
// tuple-returning Python functions. The caller specifies the desired output
// sizes; the C++ method fills locally-allocated buffers that are returned to
// Python by value.

py::tuple py_CalibTCPDistMethod(BaseCalibration& self,
                                const Eigen::VectorXd& base_offset,
                                const EigenDRef<Eigen::MatrixXd>& qa_array,
                                const EigenDRef<Eigen::VectorXd>& measureMents,
                                const EigenDRef<Eigen::Vector3d>& mes_normal,
                                Eigen::Index tool_offset_size) {
  Eigen::VectorXd tool_offset = Eigen::VectorXd::Zero(tool_offset_size);
  EigenDRef<Eigen::VectorXd> tool_offset_ref(tool_offset);
  int ret = self.CalibTCPDistMethod(base_offset, qa_array, measureMents,
                                    mes_normal, tool_offset_ref);
  return py::make_tuple(ret, tool_offset);
}

py::tuple py_CalibBaseFrame(BaseCalibration& self,
                            const EigenDRef<Eigen::MatrixXd>& jnt_measures,
                            const Eigen::VectorXd& mes_tool,
                            Eigen::Index base_size) {
  Eigen::VectorXd orig_base = Eigen::VectorXd::Zero(base_size);
  Eigen::VectorXd comp_base = Eigen::VectorXd::Zero(base_size);
  EigenDRef<Eigen::VectorXd> orig_ref(orig_base);
  EigenDRef<Eigen::VectorXd> comp_ref(comp_base);
  int ret = self.CalibBaseFrame(jnt_measures, mes_tool, orig_ref, comp_ref);
  return py::make_tuple(ret, orig_base, comp_base);
}

py::tuple py_GetCalibParamSet(BaseCalibration& self, Eigen::Index param_size) {
  Eigen::VectorXd cal_DH = Eigen::VectorXd::Zero(param_size);
  EigenDRef<Eigen::VectorXd> cal_DH_ref(cal_DH);
  bool ok = self.GetCalibParamSet(cal_DH_ref);
  return py::make_tuple(ok, cal_DH);
}

py::tuple py_PickSubJacobianForPara(BaseCalibration& self,
                                    const Eigen::MatrixXd& Jt_p,
                                    const Eigen::MatrixXd& Jp_r,
                                    bool reduction) {
  Eigen::MatrixXd Js_t1;
  Eigen::MatrixXd Js_r1;
  bool ok = self.PickSubJacobianForPara(Jt_p, Jp_r, Js_t1, Js_r1, reduction);
  return py::make_tuple(ok, Js_t1, Js_r1);
}

void py_setOptParam(BaseCalibration& self, const Eigen::VectorXd& opt_param) {
  // Round-trip through a local VectorXd so pybind11+eigen reliably binds the
  // numpy buffer (the EigenDRef<...> form with EigenDStride does not always
  // bind cleanly via &BaseCalibration::setOptParam).
  Eigen::VectorXd local = opt_param;
  EigenDRef<Eigen::VectorXd> ref(local);
  self.setOptParam(ref);
}

bool py_LoadCalibParamSet(BaseCalibration& self,
                          const Eigen::VectorXd& cal_DH) {
  Eigen::VectorXd local = cal_DH;
  EigenDRef<Eigen::VectorXd> ref(local);
  return self.LoadCalibParamSet(ref);
}

py::tuple py_CpsCartPose(BaseCalibration& self, const refPose& p,
                         const Eigen::VectorXd& canonicalBase) {
  refPose cp;
  int ret = self.CpsCartPose(p, canonicalBase, cp);
  return py::make_tuple(ret, cp);
}

py::tuple py_CpsJnt(BaseCalibration& self, const refPose& p, Eigen::Index dof) {
  Eigen::VectorXd cq = Eigen::VectorXd::Zero(dof);
  int ret = self.CpsJnt(p, cq);
  return py::make_tuple(ret, cq);
}

py::tuple py_CpsRobPath(BaseCalibration& self, const Eigen::VectorXd& calibBase,
                        const Eigen::VectorXd& origBase,
                        const Eigen::VectorXd& tool,
                        const EigenDRef<Eigen::MatrixXd>& d_traj,
                        Eigen::Index md_traj_rows, Eigen::Index d_j_traj_rows,
                        Eigen::Index md_j_traj_rows, Eigen::Index a_traj_rows) {
  const Eigen::Index cols = d_traj.cols();
  Eigen::MatrixXd md_traj = Eigen::MatrixXd::Zero(md_traj_rows, cols);
  Eigen::MatrixXd d_j_traj = Eigen::MatrixXd::Zero(d_j_traj_rows, cols);
  Eigen::MatrixXd md_j_traj = Eigen::MatrixXd::Zero(md_j_traj_rows, cols);
  Eigen::MatrixXd a_traj = Eigen::MatrixXd::Zero(a_traj_rows, cols);
  EigenDRef<Eigen::MatrixXd> md_ref(md_traj);
  EigenDRef<Eigen::MatrixXd> dj_ref(d_j_traj);
  EigenDRef<Eigen::MatrixXd> mdj_ref(md_j_traj);
  EigenDRef<Eigen::MatrixXd> a_ref(a_traj);
  int ret = self.CpsRobPath(calibBase, origBase, tool, d_traj, md_ref, dj_ref,
                            mdj_ref, a_ref);
  return py::make_tuple(ret, md_traj, d_j_traj, md_j_traj, a_traj);
}

}  // namespace

PYBIND11_MODULE(arm_calib_commands, m) {
  m.doc() =
      "Python bindings for arm_calib_kinematics::BaseCalibration and its "
      "concrete subclasses (Scara, SixAxis, SingleAxis, UJNT, XYZ gantry, "
      "XYZ+UR). Reuses geometric and motion data types (Vec, Quaternion, "
      "Frame, Pose, refPose, LocData, ProfileData, ...) from "
      "rob_motion_commands.";

  // Pull in rob_motion_commands so that its registered C++ types
  // (refPose, Pose, Frame, Vec, LocData, ...) are available to the
  // calibration bindings declared below. Without this import, methods that
  // take/return refPose would fail with a type-conversion error at call time.
  py::module_::import("rob_motion_commands");

  // BaseCalibration: abstract interface. Bindings here are inherited by every
  // concrete subclass below, so each calibration method is defined once.
  py::class_<BaseCalibration>(m, "BaseCalibration")
      .def("LaserDistanceCalib", &BaseCalibration::LaserDistanceCalib,
           py::arg("base_offset"), py::arg("tool_offset"),
           py::arg("laser2CartMap"), py::arg("cart_measure"),
           py::arg("qa_array"), py::arg("laser_measure"))
      .def("VerifyLaserDistanceCalib",
           &BaseCalibration::VerifyLaserDistanceCalib, py::arg("base_offset"),
           py::arg("tool_offset"), py::arg("laser2CartMat"),
           py::arg("cart_measure"), py::arg("qa_array"),
           py::arg("laser_measure"))
      .def("DirectMesCalib", &BaseCalibration::DirectMesCalib,
           py::arg("base_offset"), py::arg("tool_offset"),
           py::arg("cart_measure"), py::arg("measureMents"),
           py::arg("qa_array"))
      .def("VerifyDirectMesCalib", &BaseCalibration::VerifyDirectMesCalib,
           py::arg("base_offset"), py::arg("tool_offset"),
           py::arg("cart_measure"), py::arg("measureMents"),
           py::arg("qa_array"))
      .def("CalibTCPDistMethod", &py_CalibTCPDistMethod, py::arg("base_offset"),
           py::arg("qa_array"), py::arg("measureMents"), py::arg("mes_normal"),
           py::arg("tool_offset_size") = 6,
           "TCP calibration using a single mechanical distance sensor. "
           "Returns (status, tool_offset).")
      .def("CalibBaseFrame", &py_CalibBaseFrame, py::arg("jnt_measures"),
           py::arg("mes_tool"), py::arg("base_size") = 6,
           "8-point base-frame calibration. Returns "
           "(status, orig_base, comp_base).")
      .def("CpsCartPose", &py_CpsCartPose, py::arg("p"),
           py::arg("canonicalBase"),
           "Single-pose Cartesian-error compensation. Returns "
           "(status, compensated_refPose).")
      .def("CpsJnt", &py_CpsJnt, py::arg("p"), py::arg("dof"),
           "Single-point joint-angle compensation. Returns "
           "(status, compensated_joint_angles).")
      .def("CpsRobPath", &py_CpsRobPath, py::arg("calibBase"),
           py::arg("origBase"), py::arg("tool"), py::arg("d_traj"),
           py::arg("md_traj_rows"), py::arg("d_j_traj_rows"),
           py::arg("md_j_traj_rows"), py::arg("a_traj_rows"),
           "Compensate a Cartesian path using the calibrated model. Returns "
           "(status, md_traj, d_j_traj, md_j_traj, a_traj).")
      .def("GetCalibParamSet", &py_GetCalibParamSet, py::arg("param_size"),
           "Read out the calibrated parameter vector. Returns "
           "(ok, cal_DH).")
      .def("LoadCalibParamSet", &py_LoadCalibParamSet, py::arg("cal_DH"))
      .def("PickSubJacobianForPara", &py_PickSubJacobianForPara,
           py::arg("Jt_p"), py::arg("Jp_r"), py::arg("reduction") = false,
           "Pick the translational and rotational sub-Jacobians for the "
           "calibration parameters. Returns (ok, Js_t1, Js_r1).")
      .def("setOptParam", &py_setOptParam, py::arg("opt_param"),
           "Configure gradient-type optimization parameters. opt_param[0] = "
           "opt_method (0: Sam, 1: Sam-Adam); opt_param[1] = "
           "sam_region_scale.");

  py::class_<SerialArmCalib, BaseCalibration>(m, "SerialArmCalib")
      .def(py::init<>())
      .def(py::init<size_t>(), py::arg("DoF"))
      .def(py::init<const Eigen::VectorXd&>(), py::arg("kine_para"))
      .def("GetName", &SerialArmCalib::GetName)
      .def("ResetCalibration", &SerialArmCalib::ResetCalibration);

  py::class_<ScaraCalib, SerialArmCalib>(m, "ScaraCalib")
      .def(py::init<>())
      .def(py::init<const Eigen::VectorXd&>(), py::arg("kine_para"))
      .def("GetName", &ScaraCalib::GetName);

  py::class_<SixAxisCalib, SerialArmCalib>(m, "SixAxisCalib")
      .def(py::init<>())
      .def(py::init<const Eigen::VectorXd&>(), py::arg("kine_para"))
      .def("GetName", &SixAxisCalib::GetName);

  py::class_<SingleAxisCalib, SerialArmCalib>(m, "SingleAxisCalib")
      .def(py::init<>())
      .def(py::init<const Eigen::VectorXd&>(), py::arg("kine_para"))
      .def("GetName", &SingleAxisCalib::GetName);

  py::class_<UjntCalib, SerialArmCalib>(m, "UjntCalib")
      .def(py::init<>())
      .def(py::init<const Eigen::VectorXd&>(), py::arg("kine_para"))
      .def("GetName", &UjntCalib::GetName);

  py::class_<XyzGantryCalib, SerialArmCalib>(m, "XyzGantryCalib")
      .def(py::init<>())
      .def(py::init<const Eigen::VectorXd&>(), py::arg("kine_para"))
      .def("GetName", &XyzGantryCalib::GetName);

  py::class_<XyzUrCalib, BaseCalibration>(m, "XyzUrCalib")
      .def(py::init<>())
      .def(py::init<const Eigen::VectorXd&, const Eigen::VectorXd&>(),
           py::arg("dh_UR"), py::arg("dh_XYZ"))
      .def("GetName", &XyzUrCalib::GetName);
}
