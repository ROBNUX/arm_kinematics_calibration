#!/usr/bin/env python3
"""Example 06 — PickSubJacobianForPara: extract parameter sub-Jacobians.

During DH calibration the full Jacobian includes both joint-angle columns and
parameter columns.  PickSubJacobianForPara extracts only the parameter portion
(the columns corresponding to calibratable DH offsets) and optionally reduces
it to the linearly independent columns (reduction=True).

    (ok, Js_t, Js_r) = calib.PickSubJacobianForPara(
        Jt_p,       # m × n  full translational Jacobian
        Jp_r,       # k × n  full rotational   Jacobian
        reduction,  # bool — reduce to observable parameters
    )

The shape of the output depends on the concrete calibration class:
  - ScaraCalib:     Js_t is 2×p, Js_r is 1×p  (planar + yaw only)
  - SixAxisCalib:   Js_t is 3×p, Js_r is 3×p
  - SingleAxisCalib / UjntCalib: reduced dimensions

Input layout for the full Jacobian
-----------------------------------
For a DOF-joint robot with P calibratable parameters the full Jacobian has
n = DOF + P columns, where the first P columns correspond to DH parameters
and the remaining DOF columns correspond to joint angles.  The number of
rows is 3 for the translational part and 3 for the rotational part.

Run from a sourced workspace:

    python3 script/06_jacobian_picking.py
"""

import math

import numpy as np

import arm_calib_commands as c

rng = np.random.default_rng(99)


# ---------------------------------------------------------------------------
# Craig DH helpers
# ---------------------------------------------------------------------------

_BASE = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]   # identity base offset (7 elements)


def scara_kine_para() -> np.ndarray:
    alpha = [0.0, 0.0, math.pi, 0.0]
    a     = [0.30, 0.30, 0.0, 0.0]
    theta = [0.0, 0.0, 0.0, 0.0]
    d     = [0.30, 0.0, 0.20, 0.05]
    return np.array(alpha + a + theta + d + _BASE, dtype=np.float64)


def sixaxis_kine_para() -> np.ndarray:
    alpha = [0.0, -math.pi/2,  0.0, -math.pi/2,  math.pi/2, -math.pi/2]
    a     = [0.0,  0.075,      0.365, 0.09,       0.0,        0.0]
    theta = [0.0,  0.0,        0.0,   0.0,        0.0,        0.0]
    d     = [0.295, 0.0,       0.0,   0.340,      0.0,        0.095]
    return np.array(alpha + a + theta + d + _BASE, dtype=np.float64)


def demo_pick(name: str, calib_obj, dof: int, n_param_cols: int) -> None:
    """Show PickSubJacobianForPara with and without reduction."""
    total_cols = n_param_cols + dof   # parameters + joint columns

    # Random full Jacobian (3 translational rows, 3 rotational rows)
    Jt_full = rng.standard_normal((3, total_cols))
    Jr_full = rng.standard_normal((3, total_cols))

    print(f"\n  {name} (DOF={dof}, param_cols={n_param_cols}, "
          f"total_cols={total_cols})")

    # Without reduction (keep all observable parameter columns)
    ok_full, Js_t, Js_r = calib_obj.PickSubJacobianForPara(
        Jt_full, Jr_full, False
    )
    print(f"    reduction=False: ok={ok_full}  "
          f"Js_t shape={Js_t.shape}  Js_r shape={Js_r.shape}")

    # With reduction (drop linearly dependent columns)
    ok_red, Js_t_r, Js_r_r = calib_obj.PickSubJacobianForPara(
        Jt_full, Jr_full, True
    )
    print(f"    reduction=True:  ok={ok_red}  "
          f"Js_t shape={Js_t_r.shape}  Js_r shape={Js_r_r.shape}")

    if ok_full and Js_t.size > 0:
        print(f"    Frobenius norm of Js_t (full): "
              f"{np.linalg.norm(Js_t):.4f}")


def main() -> int:
    print("=" * 60)
    print("Sub-Jacobian picking for calibration parameters")
    print("=" * 60)

    # For a DOF-n serial arm: P = 4*DOF calibratable DH parameters
    # (alpha, a, theta, d for each joint).

    cases = [
        ("ScaraCalib",      c.ScaraCalib(scara_kine_para()),      4,  4 * 4),
        ("SixAxisCalib",    c.SixAxisCalib(sixaxis_kine_para()),  6,  4 * 6),
        ("SingleAxisCalib", c.SingleAxisCalib(
             np.array([0.0, 0.0, 0.0, 0.0] + _BASE)),            1,  4 * 1),
        ("UjntCalib",       c.UjntCalib(
             np.array([0.0, math.pi/2, 0.15, 0.0,
                       0.0, 0.0,       0.30, 0.0] + _BASE)),      2,  4 * 2),
    ]

    for name, obj, dof, n_params in cases:
        demo_pick(name, obj, dof, n_params)

    # ----------------------------------------------------------------
    # Illustration: use the picked sub-Jacobian to build a normal-equation
    # system for a calibration step (pseudocode / conceptual demo).
    # ----------------------------------------------------------------
    print("\n--- Conceptual: least-squares calibration step ---")
    scara = c.ScaraCalib(scara_kine_para())
    dof   = 4
    n_params = 4 * dof

    # Simulated measurements and Jacobian for one calibration iteration
    n_poses = 20
    Jt_full  = rng.standard_normal((3 * n_poses, n_params + dof))
    Jr_full  = rng.standard_normal((3 * n_poses, n_params + dof))
    err_t    = rng.normal(0, 1e-3, 3 * n_poses)  # translational residuals
    err_r    = rng.normal(0, 1e-4, 3 * n_poses)  # rotational residuals

    ok, Js_t, Js_r = scara.PickSubJacobianForPara(Jt_full, Jr_full, True)
    if ok and Js_t.size > 0 and Js_r.size > 0:
        # Stack translational and rotational sub-Jacobians
        J_stacked  = np.vstack([Js_t, Js_r])
        err_stacked = np.concatenate([err_t[:Js_t.shape[0]],
                                      err_r[:Js_r.shape[0]]])
        # Normal equations: min ||J*Δp - e||²  →  Δp = pinv(J)*e
        delta_params, _, _, _ = np.linalg.lstsq(J_stacked, err_stacked,
                                                 rcond=None)
        print(f"  Js_t shape: {Js_t.shape}  Js_r shape: {Js_r.shape}")
        print(f"  Solved delta_params ({len(delta_params)} values): "
              f"{delta_params.round(6)}")
    else:
        print(f"  PickSubJacobianForPara returned ok={ok}; "
              f"skipping least-squares step.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
