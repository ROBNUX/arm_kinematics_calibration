#!/usr/bin/env python3
"""Example 02 — DirectMesCalib and VerifyDirectMesCalib.

DirectMesCalib identifies robot DH parameters from pairs of:
  - cart_measure  : Cartesian poses logged by the robot controller (using its
                    built-in, possibly erroneous kinematic model).
  - measureMents  : Corresponding tool positions measured by an external
                    device such as a CMM or tracking laser.

The mismatch between the two data sets is what the calibration algorithm
minimises to identify corrected DH parameters.

VerifyDirectMesCalib reports the average point-position error before and
after calibration; output[1] > 0 means calibration improved accuracy.

Data format
-----------
cart_measure : 8 × N  float64 matrix
    Each column is one measurement:
    [x, y, z, roll, pitch, yaw, branch_flag, turn]
measureMents : 3 × N  float64 matrix
    Each column is the externally measured tool position: [x, y, z]
qa_array     : DOF × N  float64 matrix
    Each column is the joint-angle vector for that measurement pose.
base_offset  : 7-element float64 vector  [tx, ty, tz, qw, qx, qy, qz]
tool_offset  : 7-element float64 vector  [tx, ty, tz, qw, qx, qy, qz]

Run from a sourced workspace:

    python3 script/02_direct_measurement_calib.py
"""

import math

import numpy as np

import arm_calib_commands as c

rng = np.random.default_rng(42)

# ---------------------------------------------------------------------------
# SCARA DH parameters (4 DOF)
#   kine_para = [alpha x4, a x4, theta x4, d x4, tx, ty, tz, qw, qx, qy, qz]
#   The trailing 7-element identity base offset lets the constructor infer
#   DoF = (size - 7) / 4.
# ---------------------------------------------------------------------------
DOF = 4
ALPHA = np.array([0.0, 0.0, math.pi, 0.0])
A     = np.array([0.30, 0.30, 0.0, 0.0])
THETA = np.array([0.0, 0.0, 0.0, 0.0])
D     = np.array([0.30, 0.0, 0.20, 0.05])
BASE  = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
KINE_PARA = np.concatenate([ALPHA, A, THETA, D, BASE])

# Identity base and tool offsets: [tx, ty, tz, qw, qx, qy, qz]
IDENTITY_FRAME = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], dtype=np.float64)

N_POSES = 30  # number of calibration measurement poses


def make_synthetic_data(dof: int, n: int) -> tuple:
    """Generate synthetic calibration data for a SCARA-like robot.

    In a real experiment you would:
      1. Command the robot to N poses and record the joint angles (qa_array).
      2. Log the controller's FK output as cart_measure.
      3. Measure the actual TCP position with a CMM → measureMents.

    Here we fake this: qa_array is random joint configurations, cart_measure
    is the nominal FK result (approximated), and measureMents adds a small
    systematic offset to simulate DH parameter errors.
    """
    qa_array = rng.uniform(-math.pi / 3, math.pi / 3, (dof, n))

    # --- approximate Cartesian positions for a 2-link SCARA (first 2 joints) ---
    # joint 0 and 1 drive x/y; joint 2 drives z; joint 3 is rotation
    x = A[0] * np.cos(qa_array[0]) + A[1] * np.cos(qa_array[0] + qa_array[1])
    y = A[0] * np.sin(qa_array[0]) + A[1] * np.sin(qa_array[0] + qa_array[1])
    z = D[0] - qa_array[2] - D[2]
    yaw = qa_array[0] + qa_array[1] + qa_array[3]

    cart_measure = np.zeros((8, n), dtype=np.float64)
    cart_measure[0] = x
    cart_measure[1] = y
    cart_measure[2] = z
    cart_measure[3] = 0.0       # roll  (SCARA tool is always vertical)
    cart_measure[4] = 0.0       # pitch
    cart_measure[5] = yaw       # yaw
    cart_measure[6] = 0.0       # branch flag
    cart_measure[7] = 0.0       # turn

    # Simulate DH errors: small systematic offset in each direction
    dh_error = np.array([0.002, -0.001, 0.003])
    measurements = np.vstack([x + dh_error[0],
                               y + dh_error[1],
                               z + dh_error[2]])  # 3 × N

    return qa_array, cart_measure, measurements


def main() -> int:
    print("=" * 60)
    print("Direct measurement calibration — ScaraCalib")
    print("=" * 60)

    scara = c.ScaraCalib(KINE_PARA)

    qa_array, cart_measure, measurements = make_synthetic_data(DOF, N_POSES)

    print(f"\nData: {N_POSES} measurement poses  DOF={DOF}")
    print(f"  qa_array     shape: {qa_array.shape}   dtype: {qa_array.dtype}")
    print(f"  cart_measure shape: {cart_measure.shape}   dtype: {cart_measure.dtype}")
    print(f"  measurements shape: {measurements.shape}   dtype: {measurements.dtype}")

    # ----------------------------------------------------------------
    # DirectMesCalib: returns average point error after calibration
    # ----------------------------------------------------------------
    print("\n--- DirectMesCalib ---")
    err_after = scara.DirectMesCalib(
        IDENTITY_FRAME,   # base_offset
        IDENTITY_FRAME,   # tool_offset
        cart_measure,
        measurements,
        qa_array,
    )
    print(f"  Average point error after calibration: {err_after:.6f} m")

    # ----------------------------------------------------------------
    # VerifyDirectMesCalib: returns [err_before, delta_improvement]
    # ----------------------------------------------------------------
    print("\n--- VerifyDirectMesCalib ---")
    result = scara.VerifyDirectMesCalib(
        IDENTITY_FRAME,
        IDENTITY_FRAME,
        cart_measure,
        measurements,
        qa_array,
    )
    print(f"  Verification result vector: {result}")
    if len(result) >= 2:
        print(f"  Average error before calibration: {result[0]:.6f} m")
        print(f"  Delta (>0 means improvement):     {result[1]:.6f} m")

    # ----------------------------------------------------------------
    # Read back the calibrated DH parameters
    # ----------------------------------------------------------------
    print("\n--- Calibrated DH parameter set ---")
    param_size = 4 * DOF + 7
    ok, cal_dh = scara.GetCalibParamSet(param_size)
    if ok:
        print(f"  alpha (calibrated): {cal_dh[:DOF].round(6)}")
        print(f"  a     (calibrated): {cal_dh[DOF:2*DOF].round(6)}")
        print(f"  theta (calibrated): {cal_dh[2*DOF:3*DOF].round(6)}")
        print(f"  d     (calibrated): {cal_dh[3*DOF:].round(6)}")
    else:
        print("  GetCalibParamSet returned ok=False")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
