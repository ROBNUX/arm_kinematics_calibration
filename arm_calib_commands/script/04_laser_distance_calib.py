#!/usr/bin/env python3
"""Example 04 — laser-distance DH calibration.

LaserDistanceCalib uses three mutually perpendicular laser displacement
sensors to identify DH parameters from a set of robot poses.  Each sensor
measures how much its target plane has moved relative to a reference pose.

    err_after = calib.LaserDistanceCalib(
        base_offset,    # 7-element [tx, ty, tz, qw, qx, qy, qz]
        tool_offset,    # 7-element [tx, ty, tz, qw, qx, qy, qz]
        laser2CartMap,  # 3 × 3 matrix: laser_disp -> Cartesian_disp
        cart_measure,   # 8 × N Cartesian measurement matrix
        qa_array,       # DOF × N joint-angle matrix
        laser_measure,  # 3 × N laser-reading matrix
    )

VerifyLaserDistanceCalib returns a 2-element vector:
    result[0]  average point error with uncalibrated model
    result[1]  delta improvement (positive = calibration helped)

Data layout
-----------
cart_measure  (8 × N): [x, y, z, roll, pitch, yaw, branch, turn] per column
laser_measure (3 × N): [laser_x_disp, laser_y_disp, laser_z_disp] per column
laser2CartMap (3 × 3): maps laser displacements to Cartesian displacements
    For three axis-aligned lasers this is the identity matrix.

Run from a sourced workspace:

    python3 script/04_laser_distance_calib.py
"""

import math

import numpy as np

import arm_calib_commands as c

rng = np.random.default_rng(7)

# ---------------------------------------------------------------------------
# SCARA DH parameters (4 DOF)
# ---------------------------------------------------------------------------
DOF = 4
ALPHA = np.array([0.0, 0.0, math.pi, 0.0])
A     = np.array([0.30, 0.30, 0.0, 0.0])
THETA = np.array([0.0, 0.0, 0.0, 0.0])
D     = np.array([0.30, 0.0, 0.20, 0.05])
BASE  = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
KINE_PARA = np.concatenate([ALPHA, A, THETA, D, BASE])

IDENTITY_FRAME = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], dtype=np.float64)

N_POSES = 25


def make_laser_data(dof: int, n: int) -> tuple:
    """Generate synthetic laser calibration data.

    In a real setup:
      1. Move the robot to N poses; record joint angles and controller FK.
      2. Read the three laser sensors at each pose.
      3. Build qa_array, cart_measure, and laser_measure.

    Here we approximate SCARA FK and add small systematic DH errors as the
    "true" physical displacements seen by the lasers.
    """
    qa = rng.uniform(-math.pi / 3, math.pi / 3, (dof, n)).astype(np.float64)

    x = A[0] * np.cos(qa[0]) + A[1] * np.cos(qa[0] + qa[1])
    y = A[0] * np.sin(qa[0]) + A[1] * np.sin(qa[0] + qa[1])
    z = D[0] - qa[2] - D[2]
    yaw = qa[0] + qa[1] + qa[3]

    cart_measure = np.zeros((8, n), dtype=np.float64)
    cart_measure[0] = x
    cart_measure[1] = y
    cart_measure[2] = z
    cart_measure[5] = yaw

    # Laser readings: each laser measures the shift in one Cartesian axis.
    # Systematic DH error creates a small offset across all poses.
    dh_error = np.array([0.0015, -0.0008, 0.0025])
    laser_measure = np.zeros((3, n), dtype=np.float64)
    laser_measure[0] = x + dh_error[0] + rng.normal(0, 5e-5, n)  # X laser
    laser_measure[1] = y + dh_error[1] + rng.normal(0, 5e-5, n)  # Y laser
    laser_measure[2] = z + dh_error[2] + rng.normal(0, 5e-5, n)  # Z laser

    return qa, cart_measure, laser_measure


def main() -> int:
    print("=" * 60)
    print("Laser-distance DH calibration — ScaraCalib")
    print("=" * 60)

    scara = c.ScaraCalib(KINE_PARA)

    qa_array, cart_measure, laser_measure = make_laser_data(DOF, N_POSES)

    # For axis-aligned lasers the mapping matrix is the identity.
    laser2CartMap = np.eye(3, dtype=np.float64)

    print(f"\nData: {N_POSES} poses  DOF={DOF}")
    print(f"  cart_measure  shape: {cart_measure.shape}")
    print(f"  laser_measure shape: {laser_measure.shape}")
    print(f"  laser2CartMap shape: {laser2CartMap.shape}")

    # ----------------------------------------------------------------
    # LaserDistanceCalib
    # ----------------------------------------------------------------
    print("\n--- LaserDistanceCalib ---")
    err_after = scara.LaserDistanceCalib(
        IDENTITY_FRAME,    # base_offset
        IDENTITY_FRAME,    # tool_offset
        laser2CartMap,
        cart_measure,
        qa_array,
        laser_measure,
    )
    print(f"  Average point error after calibration: {err_after:.6f} m")

    # ----------------------------------------------------------------
    # VerifyLaserDistanceCalib
    # ----------------------------------------------------------------
    print("\n--- VerifyLaserDistanceCalib ---")
    result = scara.VerifyLaserDistanceCalib(
        IDENTITY_FRAME,
        IDENTITY_FRAME,
        laser2CartMap,
        cart_measure,
        qa_array,
        laser_measure,
    )
    print(f"  Result vector: {result}")
    if len(result) >= 2:
        print(f"  Avg error (uncalibrated): {result[0]:.6f} m")
        print(f"  Delta improvement (>0 is better): {result[1]:.6f} m")

    # ----------------------------------------------------------------
    # Read back calibrated DH parameters
    # ----------------------------------------------------------------
    print("\n--- Calibrated DH parameters ---")
    ok, cal_dh = scara.GetCalibParamSet(4 * DOF)
    if ok:
        print(f"  alpha: {cal_dh[:DOF].round(6)}")
        print(f"  a:     {cal_dh[DOF:2*DOF].round(6)}")
        print(f"  theta: {cal_dh[2*DOF:3*DOF].round(6)}")
        print(f"  d:     {cal_dh[3*DOF:].round(6)}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
