#!/usr/bin/env python3
"""Example 01 — calibration class construction and basic parameter API.

Covers:
  - Constructing every concrete calibration class (default and with kine_para).
  - GetName  — verify each class reports its expected name.
  - setOptParam — configure the gradient optimizer (Sam / Sam-Adam).
  - GetCalibParamSet / LoadCalibParamSet — parameter-set round-trip.
  - ResetCalibration — restore a calibrated model to its nominal state.

Run from a sourced workspace:

    python3 script/01_calib_classes.py
"""

import math

import numpy as np

import arm_calib_commands as c


# ---------------------------------------------------------------------------
# Craig DH parameter layout for serialArm-based classes:
#   kine_para = [alpha[0..n-1], a[0..n-1], theta[0..n-1], d[0..n-1],
#                tx, ty, tz, qw, qx, qy, qz]   ← 4*DoF + 7 elements total
# The trailing 7 elements are the identity base offset; the constructor uses
# them only to infer DoF = (size - 7) / 4.
# ---------------------------------------------------------------------------

_BASE = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]   # identity frame [tx,ty,tz,qw,qx,qy,qz]


def scara_kine_para() -> np.ndarray:
    """4-DOF SCARA: two 300 mm links, 300 mm vertical travel."""
    alpha = [0.0,        0.0,  math.pi, 0.0]
    a     = [0.30,       0.30, 0.0,     0.0]
    theta = [0.0,        0.0,  0.0,     0.0]
    d     = [0.30,       0.0,  0.20,    0.05]
    return np.array(alpha + a + theta + d + _BASE, dtype=np.float64)


def sixaxis_kine_para() -> np.ndarray:
    """6-DOF serial arm — simplified UR-style DH parameters."""
    alpha = [0.0, -math.pi/2,  0.0, -math.pi/2,  math.pi/2, -math.pi/2]
    a     = [0.0,  0.075,      0.365, 0.09,       0.0,        0.0]
    theta = [0.0,  0.0,        0.0,   0.0,        0.0,        0.0]
    d     = [0.295, 0.0,       0.0,   0.340,      0.0,        0.095]
    return np.array(alpha + a + theta + d + _BASE, dtype=np.float64)


def singleaxis_kine_para() -> np.ndarray:
    """1-DOF prismatic axis."""
    return np.array([0.0, 0.0, 0.0, 0.0] + _BASE, dtype=np.float64)


def ujnt_kine_para() -> np.ndarray:
    """2-DOF universal joint (R-R): 150 mm arm, 90° twist."""
    alpha = [0.0,         math.pi / 2]
    a     = [0.15,        0.0]
    theta = [0.0,         0.0]
    d     = [0.30,        0.0]
    return np.array(alpha + a + theta + d + _BASE, dtype=np.float64)


def xyz_kine_para() -> np.ndarray:
    """3-DOF XYZ gantry: axis-aligned prismatic joints."""
    alpha = [0.0,  math.pi / 2, -math.pi / 2]
    a     = [0.0,  0.0,          0.0]
    theta = [0.0,  math.pi / 2, -math.pi / 2]
    d     = [0.0,  0.0,          0.0]
    return np.array(alpha + a + theta + d + _BASE, dtype=np.float64)


def main() -> int:
    print("=" * 60)
    print("Calibration classes — arm_calib_commands")
    print("=" * 60)

    # ----------------------------------------------------------------
    # 1. Construct every concrete class (default + with kine_para)
    # ----------------------------------------------------------------
    print("\n--- Construction and GetName ---")
    classes_default = [
        ("ScaraCalib",      c.ScaraCalib()),
        ("SixAxisCalib",    c.SixAxisCalib()),
        ("SingleAxisCalib", c.SingleAxisCalib()),
        ("UjntCalib",       c.UjntCalib()),
        ("XyzGantryCalib",  c.XyzGantryCalib()),
        ("XyzUrCalib",      c.XyzUrCalib()),
        ("SerialArmCalib",  c.SerialArmCalib(4)),   # 4-DOF generic
    ]
    for cls_name, obj in classes_default:
        print(f"  {cls_name}() -> GetName: '{obj.GetName()}'")

    classes_with_para = [
        ("ScaraCalib(para)",      c.ScaraCalib(scara_kine_para())),
        ("SixAxisCalib(para)",    c.SixAxisCalib(sixaxis_kine_para())),
        ("SingleAxisCalib(para)", c.SingleAxisCalib(singleaxis_kine_para())),
        ("UjntCalib(para)",       c.UjntCalib(ujnt_kine_para())),
        ("XyzGantryCalib(para)",  c.XyzGantryCalib(xyz_kine_para())),
        ("XyzUrCalib(UR,XYZ)",    c.XyzUrCalib(sixaxis_kine_para(), xyz_kine_para())),
    ]
    for cls_name, obj in classes_with_para:
        print(f"  {cls_name} -> GetName: '{obj.GetName()}'")

    # ----------------------------------------------------------------
    # 2. setOptParam — two optimizer configurations
    # ----------------------------------------------------------------
    print("\n--- setOptParam ---")
    scara = c.ScaraCalib(scara_kine_para())

    # Sam-type optimizer (method=0): opt_param = [method, region_scale]
    sam_param = np.array([0.0, 1.0], dtype=np.float64)
    scara.setOptParam(sam_param)
    print(f"  Sam config applied:       method={sam_param[0]:.0f}  "
          f"region_scale={sam_param[1]:.2f}")

    # Sam-Adam optimizer (method=1): region_scale can be larger
    six = c.SixAxisCalib(sixaxis_kine_para())
    adam_param = np.array([1.0, 2.5], dtype=np.float64)
    six.setOptParam(adam_param)
    print(f"  Sam-Adam config applied:  method={adam_param[0]:.0f}  "
          f"region_scale={adam_param[1]:.2f}")

    # ----------------------------------------------------------------
    # 3. GetCalibParamSet / LoadCalibParamSet round-trip
    # ----------------------------------------------------------------
    print("\n--- GetCalibParamSet / LoadCalibParamSet round-trip ---")

    for cls_name, obj, dof in [
        ("ScaraCalib",      c.ScaraCalib(scara_kine_para()),      4),
        ("SixAxisCalib",    c.SixAxisCalib(sixaxis_kine_para()),  6),
        ("SingleAxisCalib", c.SingleAxisCalib(singleaxis_kine_para()), 1),
        ("UjntCalib",       c.UjntCalib(ujnt_kine_para()),        2),
        ("XyzGantryCalib",  c.XyzGantryCalib(xyz_kine_para()),    3),
    ]:
        param_size = 4 * dof + 7  # DH params (alpha,a,theta,d) + 7-element base offset
        ok, cal_dh = obj.GetCalibParamSet(param_size)
        print(f"  {cls_name}: GetCalibParamSet(size={param_size}) ok={ok}  "
              f"len={len(cal_dh)}")
        if ok and len(cal_dh) > 0:
            loaded = obj.LoadCalibParamSet(cal_dh)
            print(f"    LoadCalibParamSet(same vec) returned: {loaded}")

    # ----------------------------------------------------------------
    # 4. ResetCalibration — restore nominal DH after (hypothetical) calibration
    # ----------------------------------------------------------------
    print("\n--- ResetCalibration ---")
    scara2 = c.ScaraCalib(scara_kine_para())
    scara2.ResetCalibration()
    ok, cal_dh = scara2.GetCalibParamSet(4 * 4 + 7)
    print(f"  ScaraCalib after ResetCalibration: GetCalibParamSet ok={ok}  "
          f"len={len(cal_dh)}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
