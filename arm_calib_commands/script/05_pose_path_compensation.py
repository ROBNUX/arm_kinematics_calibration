#!/usr/bin/env python3
"""Example 05 — Cartesian-pose and path compensation after calibration.

Once DH parameters have been calibrated (via DirectMesCalib, LaserDistanceCalib,
or by loading a previously saved parameter set), the compensation functions
shift robot commands to account for model errors:

CpsCartPose
-----------
Single-pose compensation: given a desired Cartesian pose (refPose) and the
canonical base frame, returns the corrected pose that the robot should
actually target so that the TCP lands on the desired position.

    (ret, compensated_refPose) = calib.CpsCartPose(p, canonicalBase)

CpsJnt
------
Single-point compensation: returns corrected joint angles for a desired pose.

    (ret, corrected_jnt) = calib.CpsJnt(p, dof)

CpsRobPath
----------
Batch path compensation: applies the calibrated model to an entire Cartesian
trajectory and returns the corrected trajectory plus auxiliary outputs.

    (ret, md_traj, d_j_traj, md_j_traj, a_traj) = calib.CpsRobPath(
        calibBase,      # 7-element calibrated base frame
        origBase,       # 7-element uncalibrated (canonical) base frame
        tool,           # 7-element tool frame
        d_traj,         # 7 × N desired Cartesian path  [tx,ty,tz,qw,qx,qy,qz]
        md_traj_rows,   # rows for compensated Cartesian output (7)
        d_j_traj_rows,  # rows for nominal joint trajectory (DOF)
        md_j_traj_rows, # rows for compensated joint trajectory (DOF)
        a_traj_rows,    # rows for actual Cartesian trajectory (7)
    )

Run from a sourced workspace:

    python3 script/05_pose_path_compensation.py
"""

import math

import numpy as np

import arm_calib_commands as c
import rob_motion_commands as m

rng = np.random.default_rng(3)

# ---------------------------------------------------------------------------
# SCARA DH parameters (4 DOF)
# ---------------------------------------------------------------------------
DOF = 4
ALPHA = np.array([0.0, 0.0, math.pi, 0.0])
A     = np.array([0.30, 0.30, 0.0, 0.0])
THETA = np.array([0.0, 0.0, 0.0, 0.0])
D     = np.array([0.30, 0.0, 0.20, 0.05])
KINE_PARA = np.concatenate([ALPHA, A, THETA, D])

# Identity frame as 7-element vector [tx, ty, tz, qw, qx, qy, qz]
IDENTITY_FRAME = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], dtype=np.float64)


def prepare_calib(scara: c.ScaraCalib) -> None:
    """Run a minimal DirectMesCalib so the model is 'calibrated'."""
    N = 20
    qa = rng.uniform(-math.pi / 3, math.pi / 3, (DOF, N)).astype(np.float64)
    x = A[0] * np.cos(qa[0]) + A[1] * np.cos(qa[0] + qa[1])
    y = A[0] * np.sin(qa[0]) + A[1] * np.sin(qa[0] + qa[1])
    z = D[0] - qa[2] - D[2]
    yaw = qa[0] + qa[1] + qa[3]

    cart = np.zeros((8, N), dtype=np.float64)
    cart[0], cart[1], cart[2], cart[5] = x, y, z, yaw

    meas = np.vstack([x + 0.002, y - 0.001, z + 0.003])
    scara.DirectMesCalib(IDENTITY_FRAME, IDENTITY_FRAME, cart, meas, qa)


def make_refpose(x: float, y: float, z: float) -> m.refPose:
    """Construct a refPose at (x,y,z) with identity rotation."""
    ee = m.Frame(m.Vec(x, y, z))
    base = m.Frame.Identity()
    tool = m.Frame.Identity()
    return m.refPose(ee, base, tool, [0], [0])


def main() -> int:
    print("=" * 60)
    print("Pose and path compensation — ScaraCalib")
    print("=" * 60)

    scara = c.ScaraCalib(KINE_PARA)
    prepare_calib(scara)
    print("  Calibration step completed (synthetic data).")

    # ----------------------------------------------------------------
    # CpsCartPose — single-pose Cartesian compensation
    # ----------------------------------------------------------------
    print("\n--- CpsCartPose ---")
    test_poses = [
        make_refpose(0.45,  0.15, -0.50),
        make_refpose(-0.30, 0.40, -0.60),
        make_refpose(0.20, -0.20, -0.45),
    ]
    for idx, rp in enumerate(test_poses):
        ok_orig, orig_frame = rp.getBase()
        t = orig_frame.getTranslation() if ok_orig else None
        ret, cps_pose = scara.CpsCartPose(rp, IDENTITY_FRAME)
        print(f"  Pose {idx}: ret={ret}  compensated refPose returned: "
              f"{type(cps_pose).__name__}")

    # ----------------------------------------------------------------
    # CpsJnt — single-point joint-space compensation
    # ----------------------------------------------------------------
    print("\n--- CpsJnt ---")
    for idx, rp in enumerate(test_poses):
        ret, cq = scara.CpsJnt(rp, DOF)
        status = "ok" if ret == 0 else f"code={ret}"
        print(f"  Pose {idx}: {status}  corrected joints={cq.round(4)}")

    # ----------------------------------------------------------------
    # CpsRobPath — full path compensation
    # ----------------------------------------------------------------
    print("\n--- CpsRobPath ---")
    N_TRAJ = 15   # number of trajectory waypoints

    # Desired Cartesian path: [tx, ty, tz, qw, qx, qy, qz] per column
    t_vals = np.linspace(0, 1, N_TRAJ)
    d_traj = np.zeros((7, N_TRAJ), dtype=np.float64)
    d_traj[0] = 0.4 * np.cos(2 * math.pi * t_vals)      # x
    d_traj[1] = 0.4 * np.sin(2 * math.pi * t_vals)      # y
    d_traj[2] = -0.55                                     # z = constant
    d_traj[3] = 1.0                                       # qw = 1 (identity rotation)

    ret, md_traj, d_j_traj, md_j_traj, a_traj = scara.CpsRobPath(
        IDENTITY_FRAME,   # calibBase
        IDENTITY_FRAME,   # origBase
        IDENTITY_FRAME,   # tool
        d_traj,
        md_traj_rows=7,   # compensated Cartesian: same 7 rows
        d_j_traj_rows=DOF,
        md_j_traj_rows=DOF,
        a_traj_rows=7,
    )
    print(f"  CpsRobPath return code: {ret}")
    print(f"  d_traj    shape: {d_traj.shape}")
    print(f"  md_traj   shape: {md_traj.shape}  "
          f"(compensated Cartesian path)")
    print(f"  d_j_traj  shape: {d_j_traj.shape}  "
          f"(nominal joint trajectory)")
    print(f"  md_j_traj shape: {md_j_traj.shape}  "
          f"(modified joint trajectory)")
    print(f"  a_traj    shape: {a_traj.shape}  "
          f"(actual Cartesian from modified joints)")

    if ret == 0 and md_traj.shape[1] > 0:
        cart_err = np.linalg.norm(md_traj[:3, :] - d_traj[:3, :], axis=0)
        print(f"  Max XYZ compensation delta: {cart_err.max():.6f} m")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
