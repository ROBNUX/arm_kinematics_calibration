#!/usr/bin/env python3
"""Example 03 — TCP calibration and base-frame calibration.

CalibTCPDistMethod
------------------
Identifies the TCP offset using a mechanical distance sensor that measures
the perpendicular distance from the tool tip to a flat plane. The robot is
commanded to N poses; at each pose the sensor reading and the joint angles
are recorded.

    (ret, tool_offset) = calib.CalibTCPDistMethod(
        base_offset,      # 7-element [tx, ty, tz, qw, qx, qy, qz]
        qa_array,         # DOF × N joint-angle matrix
        measureMents,     # N-element vector of distance readings (m)
        mes_normal,       # 3-element plane normal unit vector
        tool_offset_size, # output size (default 6: tx,ty,tz,yaw,pitch,roll)
    )

CalibBaseFrame
--------------
Determines the robot base frame position and orientation using an 8-point
probe / laser measurement method. At each of the 8 poses the probe touches
a fixed feature on a workpiece.

    (ret, orig_base, comp_base) = calib.CalibBaseFrame(
        jnt_measures,  # DOF × 8 matrix
        mes_tool,      # 7-element tool frame of the probe
        base_size,     # output size (default 6)
    )

Run from a sourced workspace:

    python3 script/03_tcp_and_base_calib.py
"""

import math

import numpy as np

import arm_calib_commands as c

rng = np.random.default_rng(0)

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


def main() -> int:
    print("=" * 60)
    print("TCP calibration and base-frame calibration")
    print("=" * 60)

    scara = c.ScaraCalib(KINE_PARA)

    # ================================================================
    # CalibTCPDistMethod
    # ================================================================
    print("\n--- CalibTCPDistMethod ---")

    N_TCP = 12   # number of probe poses for TCP identification

    # Joint angles: vary joint 0 (rotation around base) so that the same
    # TCP sees the plane from different approach angles.
    qa_tcp = np.zeros((DOF, N_TCP), dtype=np.float64)
    qa_tcp[0] = np.linspace(-math.pi / 4, math.pi / 4, N_TCP)
    qa_tcp[2] = 0.05  # small prismatic offset

    # Synthetic distance readings: in a real experiment the probe gives the
    # perpendicular distance from the tip to the plane.  Here we simulate a
    # constant distance (the plane is always at the same height) with small
    # noise.
    true_tcp_z = 0.12   # true TCP length in the z-direction of the tool frame
    base_dist  = 0.050  # nominal distance from base to the plane
    measurements_tcp = np.full(N_TCP, base_dist, dtype=np.float64)
    measurements_tcp += rng.normal(0, 1e-4, N_TCP)  # add sensor noise

    # Plane normal: the plane is horizontal, so normal points in +Z
    mes_normal = np.array([0.0, 0.0, 1.0], dtype=np.float64)

    ret_tcp, tool_offset = scara.CalibTCPDistMethod(
        IDENTITY_FRAME,
        qa_tcp,
        measurements_tcp,
        mes_normal,
        tool_offset_size=6,   # output: [tx, ty, tz, yaw, pitch, roll]
    )
    print(f"  Return code: {ret_tcp}  (0 = success)")
    if ret_tcp == 0:
        print(f"  Identified TCP offset (tx,ty,tz,yaw,pitch,roll):")
        print(f"    {tool_offset.round(6)}")
    else:
        print(f"  TCP calibration did not converge (code={ret_tcp})")

    # ================================================================
    # CalibBaseFrame
    # ================================================================
    print("\n--- CalibBaseFrame ---")

    N_BASE = 8   # 8-point method

    # At each of the 8 poses the probe (attached at the tool flange) touches
    # a fixed feature on the workpiece.  The feature positions are known in
    # the workpiece frame; the joint angles at each touch are recorded.
    qa_base = rng.uniform(-math.pi / 4, math.pi / 4, (DOF, N_BASE)).astype(np.float64)

    # Tool frame of the probe: identity (probe tip is at the tool origin)
    mes_tool = IDENTITY_FRAME.copy()

    ret_base, orig_base, comp_base = scara.CalibBaseFrame(
        qa_base,
        mes_tool,
        base_size=6,   # output: [tx, ty, tz, yaw, pitch, roll]
    )
    print(f"  Return code: {ret_base}  (0 = success)")
    if ret_base == 0:
        print(f"  Uncalibrated base frame (orig_base): {orig_base.round(6)}")
        print(f"  Calibrated  base frame (comp_base): {comp_base.round(6)}")
    else:
        print(f"  Base-frame calibration did not converge (code={ret_base})")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
