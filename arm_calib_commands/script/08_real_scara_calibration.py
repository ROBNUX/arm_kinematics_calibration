#!/usr/bin/env python3
"""
DirectMesCalib calibration against real measured data from two Mitsubishi
SCARA robots (controller-reported joint/Cartesian log + independent laser
tracker ground truth), reporting position error before vs. after
calibration.

Nominal geometry for both robots comes from the canonical DH parameter
files in arm_kinematics_trajectory/robot_model/ (6CRH7020.txt,
6020_scara.txt -- one value per line, [alpha(4) | a(4) | theta(4) | d(4)],
loaded via load_canonical_dh() below), cross-checked directly against the
measured data before running any calibration (see
memory/project_real_scara_calib.md for the full derivation):

  alpha = [0, 0, 0, 0]   (Scara::CartToJnt never references alpha_ at all --
                          a pure-planar, all-parallel-Z chain)
  a     = [0, a1, a2, 0] (a1/a2 = the two link lengths; NOT a[0]/a[1] --
                          confirmed against scara.cpp's own a_[1]/a_[2]
                          usage. 6CRH7020: 425mm/275mm. 6020_scara:
                          325mm/275mm)
  theta = [0, 0, 0, 0]
  d     = [0, 0, 0, 0]   (matches the canonical files exactly; a separate,
                          fixed Z-datum offset specific to how each
                          dataset's cart.csv reports Z -- found by
                          comparing cart.csv's Z column against jnt.csv's
                          J3 column, constant to within 0.06 micron across
                          each full dataset, i.e. a genuine mechanical/
                          reference-frame constant, not noise -- is passed
                          separately as base_offset's Z-translation
                          (verified numerically equivalent to folding it
                          into d[0] instead; base_offset is the more
                          correct place for it since it keeps the DH model
                          itself matching the canonical file exactly)

Which measurement directory is which robot is *derived*, not assumed: see
identify_robot() below, which fits each dataset's controller-reported
X/Y (unaffected by any Z-offset question) against both candidate a1/a2
pairs and picks whichever gives near-zero residual -- the wrong pairing
is off by exactly |a1_wrong - a1_right| across every sample, an
unambiguous signal.

Data format (both directories): three CSVs sharing one row per sample --
  jnt*.csv:   [J1(deg), J2(deg), J3(mm), J4(deg), ...unused padding axes]
  cart*.csv:  [X(mm), Y(mm), Z(mm), A(deg), B(deg), C(deg), ...]
              -- the *controller's own* (uncalibrated) FK-computed pose
  laser*.csv: [X(mm), Y(mm), Z(mm), ...]
              -- independently measured ground truth (laser tracker)

Run (ROS2 + /opt/robnux overlay sourced):
    python3 08_real_scara_calibration.py
"""
import glob
import os

import numpy as np

import arm_calib_commands as c
import rob_motion_commands as m

IDENTITY = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
PF = m.Profile(0.3, 1.0, 10.0, 0.3, 1.0, 10.0)
DOF = 4
ROBOT_MODEL_DIR = "/home/leon/robnux/arm_kinematics_trajectory/robot_model"

CANDIDATES = {
    "6CRH7020": ("6CRH7020.txt", 0.425, 0.275),
    "6020_scara": ("6020_scara.txt", 0.325, 0.275),
}


def load_canonical_a(filename: str) -> tuple:
    """Parses a robot_model/*.txt canonical DH file (one value per line,
    [alpha(4) | a(4) | theta(4) | d(4)], possibly with trailing padding) and
    returns (a1, a2) -- confirms the file's own a[1]/a[2] rather than
    hardcoding link lengths by hand."""
    path = os.path.join(ROBOT_MODEL_DIR, filename)
    vals = [float(line) for line in open(path) if line.strip()]
    a = vals[4:8]
    return a[1], a[2]


def find_one(directory: str, pattern: str) -> str:
    matches = sorted(glob.glob(os.path.join(directory, pattern)))
    if not matches:
        raise FileNotFoundError(f"no file matching {pattern!r} in {directory}")
    return matches[0]


def find_triple(directory: str, stem: str = None):
    """Locates the jnt/cart/laser CSV triple for one measurement run.

    stem=None picks whichever jnt*/cart*/laser* files exist with no other
    numbered variant conflicting (works for single-file-set directories like
    sj_july_lasertracker's jnt.csv/cart.csv/laser.csv). Directories with
    multiple grid variants (e.g. mitsubishi_7020_measure's
    {jnt,cart,laser}_grid.csv vs {jnt,cart,laser}_grid_2.csv) must pass an
    explicit stem ("_grid" or "_grid_2") to disambiguate -- globbing
    "*jnt*.csv" alone would nondeterministically match either.
    """
    if stem is None:
        return (find_one(directory, "*jnt*.csv"),
                find_one(directory, "*cart*.csv"),
                find_one(directory, "*laser*.csv"))
    return (os.path.join(directory, f"jnt{stem}.csv"),
            os.path.join(directory, f"cart{stem}.csv"),
            os.path.join(directory, f"laser{stem}.csv"))


def load_csv_floats(path: str, n_cols: int) -> np.ndarray:
    """Skips the header row; keeps only the first n_cols columns of each
    data row (real export files sometimes declare more header columns than
    they actually populate, or than we need)."""
    rows = []
    with open(path) as f:
        next(f)  # header
        for line in f:
            line = line.strip()
            if not line:
                continue
            vals = [float(x) for x in line.split(",")[:n_cols]]
            rows.append(vals)
    return np.array(rows)


def load_robot_data(directory: str, stem: str = None):
    jnt_path, cart_path, laser_path = find_triple(directory, stem)

    jnt_raw = load_csv_floats(jnt_path, DOF)      # [J1,J2,J3,J4] deg/deg/mm/deg
    cart_raw = load_csv_floats(cart_path, 3)      # [X,Y,Z] mm (controller FK)
    laser_raw = load_csv_floats(laser_path, 3)    # [X,Y,Z] mm (ground truth)

    n = min(len(jnt_raw), len(cart_raw), len(laser_raw))
    jnt_raw, cart_raw, laser_raw = jnt_raw[:n], cart_raw[:n], laser_raw[:n]

    qa_array = np.zeros((DOF, n))
    qa_array[0] = np.radians(jnt_raw[:, 0])
    qa_array[1] = np.radians(jnt_raw[:, 1])
    qa_array[2] = jnt_raw[:, 2] / 1000.0   # prismatic joint: mm -> m
    qa_array[3] = np.radians(jnt_raw[:, 3])

    cart_measure_mm = cart_raw          # keep raw mm for model-identification checks
    measurements = (laser_raw / 1000.0).T   # 3 x n, meters

    return qa_array, cart_measure_mm, measurements


def fk_xy_mm(a1: float, a2: float, qa_array: np.ndarray) -> np.ndarray:
    para = np.concatenate([[0, 0, 0, 0], [0, a1, a2, 0], [0, 0, 0, 0],
                           [0, 0, 0, 0], IDENTITY])
    rob = m.Robot("scara", para, IDENTITY, IDENTITY, PF)
    xy = np.zeros((2, qa_array.shape[1]))
    for i in range(qa_array.shape[1]):
        ok, loc = rob.ForwardKin(qa_array[:, i])
        xy[:, i] = [loc.x * 1000.0, loc.y * 1000.0]
    rob.Shutdown()
    return xy


def identify_robot(directory: str, qa_array: np.ndarray,
                   cart_measure_mm: np.ndarray) -> str:
    print(f"  Identifying robot model for {directory}:")
    best_name, best_err = None, float("inf")
    for cand_name, (filename, a1, a2) in CANDIDATES.items():
        xy = fk_xy_mm(a1, a2, qa_array)
        err = np.linalg.norm(xy - cart_measure_mm[:, 0:2].T, axis=0)
        print(f"    vs {cand_name} (a1={a1}, a2={a2} from {filename}): "
              f"XY residual mean={err.mean():.4f}mm max={err.max():.4f}mm")
        if err.mean() < best_err:
            best_err, best_name = err.mean(), cand_name
    print(f"  -> identified as {best_name} "
          f"(residual {best_err:.4f}mm, next-best is ~100mm off)")
    return best_name


def position_errors(cart_a: np.ndarray, cart_b: np.ndarray) -> np.ndarray:
    """cart_a, cart_b: 3xN meter arrays -> per-sample Euclidean error (m)."""
    return np.linalg.norm(cart_a - cart_b, axis=0)


def euler_zyx_to_R(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def fk_positions(calib, base_offset: np.ndarray, qa_array: np.ndarray,
                 tool_xyz: np.ndarray = None) -> np.ndarray:
    """Cartesian *tool-tip* positions (3xN, m) from the calib object's
    *current* (possibly just-calibrated) DH model, via GetCalibParamSet + a
    matching plain Robot instance -- avoids needing a public FK call on the
    calibration object itself. rob.ForwardKin() returns the flange frame
    only (no tool offset applied), so tool_xyz (translation-only, matching
    DirectMesCalib's own tcp_trans usage) is applied manually here in the
    flange's own world orientation -- must match whatever tool_offset was
    passed into DirectMesCalib/VerifyDirectMesCalib, or this validation
    would silently compare the wrong physical point."""
    ok, cal_dh = calib.GetCalibParamSet(4 * DOF + 7)
    if not ok:
        raise RuntimeError("GetCalibParamSet failed")
    rob = m.Robot("scara", cal_dh[: 4 * DOF + 7], base_offset, base_offset, PF)
    out = np.zeros((3, qa_array.shape[1]))
    for i in range(qa_array.shape[1]):
        ok_fk, loc = rob.ForwardKin(qa_array[:, i])
        flange = np.array([loc.x, loc.y, loc.z])
        if tool_xyz is not None:
            R = euler_zyx_to_R(np.radians(loc.A), np.radians(loc.B), np.radians(loc.C))
            flange = flange + R @ tool_xyz
        out[:, i] = flange
    rob.Shutdown()
    return out


def run_robot(name: str, qa_array: np.ndarray, cart_measure_mm: np.ndarray,
             measurements: np.ndarray, base_offset: np.ndarray, a1: float,
             a2: float, opt_method: int, opt_name: str, test_frac: float = 0.2,
             seed: int = 0, tool_offset: np.ndarray = None):
    n = qa_array.shape[1]
    if tool_offset is None:
        tool_offset = IDENTITY
    tool_xyz = tool_offset[0:3]

    print(f"\n{'=' * 70}")
    print(f"{name} -- {n} measured samples, optimizer={opt_name}")
    print(f"{'=' * 70}")

    nominal_para = np.concatenate([[0, 0, 0, 0], [0, a1, a2, 0], [0, 0, 0, 0],
                                   [0, 0, 0, 0], IDENTITY])

    cart_measure = np.zeros((8, n))
    cart_measure[0:3] = (cart_measure_mm / 1000.0).T

    # -- error of the *uncalibrated* model against ground truth, using the
    # controller's own reported cart pose (no calibration involved at all)
    before_all = position_errors(cart_measure[0:3], measurements)
    print(f"  Uncalibrated (controller-reported) error: "
          f"mean={before_all.mean()*1000:.4f}mm  max={before_all.max()*1000:.4f}mm")

    # -- error of *our own* nominal model (base_offset + tool_offset, no
    # calibration) against ground truth -- distinct from the line above,
    # since cart.csv may have been recorded under a different/unknown tool
    # configuration than the one we're telling DirectMesCalib about here
    if not np.allclose(tool_offset, IDENTITY):
        nominal_calib = c.ScaraCalib(nominal_para)
        nominal_pos_all = fk_positions(nominal_calib, base_offset, qa_array, tool_xyz)
        nominal_before_all = position_errors(nominal_pos_all, measurements)
        print(f"  Uncalibrated (our nominal model + tool_offset={np.round(tool_xyz*1000,1)}mm) "
              f"error: mean={nominal_before_all.mean()*1000:.4f}mm  "
              f"max={nominal_before_all.max()*1000:.4f}mm")

    # -- honest held-out split: calibrate on train_idx only, never let
    # DirectMesCalib see test_idx at all
    rng = np.random.default_rng(seed)
    perm = rng.permutation(n)
    n_test = max(3, int(n * test_frac))
    test_idx = perm[:n_test]
    train_idx = perm[n_test:]

    calib = c.ScaraCalib(nominal_para)
    calib.setOptParam(np.array([float(opt_method), 1.0]))
    err_after_internal = calib.DirectMesCalib(
        base_offset, tool_offset,
        cart_measure[:, train_idx], measurements[:, train_idx],
        qa_array[:, train_idx])
    print(f"  DirectMesCalib (train split, n={len(train_idx)}) "
          f"returned: {err_after_internal:.6f}")

    ok, cal_dh = calib.GetCalibParamSet(4 * DOF + 7)
    if ok:
        a_cal = cal_dh[4:8]
        d_cal = cal_dh[12:16]
        theta_cal = cal_dh[8:12]
        print(f"  Nominal a:      [0, {a1:.6f}, {a2:.6f}, 0]")
        print(f"  Calibrated a:   {np.round(a_cal, 6)}")
        print(f"  Nominal d:      [0, 0, 0, 0]")
        print(f"  Calibrated d:   {np.round(d_cal, 6)}")
        print(f"  Calibrated theta (rad): {np.round(theta_cal, 6)}")

    # true, independent held-out validation: FK the calibrated model at the
    # TEST joint values (never used for fitting) and compare to laser truth
    calibrated_pos_test = fk_positions(calib, base_offset, qa_array[:, test_idx],
                                      tool_xyz)
    after_test = position_errors(calibrated_pos_test, measurements[:, test_idx])
    before_test = (nominal_before_all[test_idx]
                  if not np.allclose(tool_offset, IDENTITY) else before_all[test_idx])
    print(f"  Held-out test set (n={len(test_idx)}, never used for fitting):")
    print(f"    before: mean={before_test.mean()*1000:.4f}mm  "
          f"max={before_test.max()*1000:.4f}mm")
    print(f"    after:  mean={after_test.mean()*1000:.4f}mm  "
          f"max={after_test.max()*1000:.4f}mm")
    improvement_pct = (1 - after_test.mean() / before_test.mean()) * 100
    print(f"    improvement: {improvement_pct:.1f}% reduction in mean error")

    # full-dataset before/after (matches the existing example scripts'
    # convention -- includes points the model was fit on, so more
    # optimistic than the held-out numbers above, but useful for
    # cross-checking against VerifyDirectMesCalib's own reported metric)
    result = calib.VerifyDirectMesCalib(base_offset, tool_offset, cart_measure,
                                        measurements, qa_array)
    print(f"  VerifyDirectMesCalib (full dataset): before={result[0]*1000:.4f}mm  "
          f"improvement={result[1]*1000:.4f}mm")

    return before_test, after_test


def run_robot_laser(name: str, qa_array: np.ndarray, cart_measure_mm: np.ndarray,
                    measurements: np.ndarray, a1: float, a2: float,
                    opt_method: int, opt_name: str, test_frac: float = 0.2,
                    seed: int = 0):
    """LaserDistanceCalib fits *relative* displacements against a reference
    sample rather than absolute coordinates, so any *constant* bias --
    including a wrong/unknown base_offset, and (since this dataset's
    orientation is constant throughout, confirmed in
    memory/project_real_scara_calib.md) even a wrong/unknown tool_offset --
    cancels out of the differencing entirely. base_offset/tool_offset are
    passed as identity here on purpose: this method's whole point is to not
    need to know them."""
    n = qa_array.shape[1]
    nominal_para = np.concatenate([[0, 0, 0, 0], [0, a1, a2, 0], [0, 0, 0, 0],
                                   [0, 0, 0, 0], IDENTITY])
    laser2CartMap = np.eye(3)

    print(f"\n{'=' * 70}")
    print(f"{name} -- LaserDistanceCalib (relative-displacement fit), "
          f"{n} samples, optimizer={opt_name}")
    print(f"{'=' * 70}")

    rng = np.random.default_rng(seed)
    perm = rng.permutation(n)
    n_test = max(3, int(n * test_frac))
    test_idx = perm[:n_test]
    train_idx = perm[n_test:]
    ref_idx = train_idx[0]   # fixed reference sample for both fit and validation

    cart_measure = np.zeros((8, n))
    cart_measure[0:3] = (cart_measure_mm / 1000.0).T

    # "before" error: relative displacement of the controller's own reported
    # position vs. relative displacement of the laser-measured ground
    # truth, both referenced against ref_idx -- robust to a wrong constant
    # base/tool offset baked into cart.csv the same way the calibration
    # itself will be
    d_cart = cart_measure[0:3, test_idx] - cart_measure[0:3, ref_idx:ref_idx + 1]
    d_laser = measurements[:, test_idx] - measurements[:, ref_idx:ref_idx + 1]
    before_test = position_errors(d_cart, d_laser)
    print(f"  Uncalibrated relative-displacement error (held-out, "
          f"vs ref sample {ref_idx}): mean={before_test.mean()*1000:.4f}mm  "
          f"max={before_test.max()*1000:.4f}mm")

    calib = c.ScaraCalib(nominal_para)
    calib.setOptParam(np.array([float(opt_method), 1.0]))
    err_after_internal = calib.LaserDistanceCalib(
        IDENTITY, IDENTITY, laser2CartMap,
        cart_measure[:, train_idx], qa_array[:, train_idx],
        measurements[:, train_idx])
    print(f"  LaserDistanceCalib (train split, n={len(train_idx)}) "
          f"returned: {err_after_internal:.6f}")

    ok, cal_dh = calib.GetCalibParamSet(4 * DOF + 7)
    if ok:
        print(f"  Nominal a:      [0, {a1:.6f}, {a2:.6f}, 0]")
        print(f"  Calibrated a:   {np.round(cal_dh[4:8], 6)}")
        print(f"  Nominal d:      [0, 0, 0, 0]")
        print(f"  Calibrated d:   {np.round(cal_dh[12:16], 6)}")
        print(f"  Calibrated theta (rad): {np.round(cal_dh[8:12], 6)}")

    # held-out validation: FK the calibrated model (base/tool = identity --
    # irrelevant by construction) at ref_idx and each test index, compare
    # relative displacements to the laser truth's relative displacements
    calibrated_pos = fk_positions(calib, IDENTITY, qa_array[:, np.append(test_idx, ref_idx)])
    calibrated_pos_test = calibrated_pos[:, :-1]
    calibrated_pos_ref = calibrated_pos[:, -1:]
    d_cal = calibrated_pos_test - calibrated_pos_ref
    after_test = position_errors(d_cal, d_laser)
    print(f"  Held-out test set (n={len(test_idx)}, never used for fitting):")
    print(f"    before: mean={before_test.mean()*1000:.4f}mm  "
          f"max={before_test.max()*1000:.4f}mm")
    print(f"    after:  mean={after_test.mean()*1000:.4f}mm  "
          f"max={after_test.max()*1000:.4f}mm")
    improvement_pct = (1 - after_test.mean() / before_test.mean()) * 100
    print(f"    improvement: {improvement_pct:.1f}% reduction in mean error")

    result = calib.VerifyLaserDistanceCalib(IDENTITY, IDENTITY, laser2CartMap,
                                            cart_measure, qa_array, measurements)
    print(f"  VerifyLaserDistanceCalib (full dataset): before={result[0]*1000:.4f}mm  "
          f"improvement={result[1]*1000:.4f}mm")

    return before_test, after_test


def prepare_robot(name: str, directory: str, a1: float, a2: float,
                  stem: str = None, known_base_offset: np.ndarray = None,
                  known_tool_offset: np.ndarray = None):
    print(f"\n{'=' * 70}")
    print(f"{name}")
    print(f"{'=' * 70}")
    qa_array, cart_measure_mm, measurements = load_robot_data(directory, stem)
    identify_robot(directory, qa_array, cart_measure_mm)

    if known_base_offset is not None or known_tool_offset is not None:
        # authoritative values (e.g. from a "Tool, base Parameter Setting"
        # file) -- do NOT also derive a Z-datum offset from cart.csv here,
        # since that would double-count the same physical Z shift the known
        # tool/base offset already accounts for
        base_offset = (known_base_offset if known_base_offset is not None
                      else IDENTITY.copy())
        tool_offset = (known_tool_offset if known_tool_offset is not None
                      else IDENTITY.copy())
        print(f"  Using known base_offset={np.round(base_offset[0:3]*1000,2)}mm, "
              f"tool_offset={np.round(tool_offset[0:3]*1000,2)}mm "
              f"(not derived from data)")
        return qa_array, cart_measure_mm, measurements, base_offset, tool_offset

    # fixed Z-datum offset between this dataset's cart.csv Z convention and
    # the DH chain's own Z=0 (see module docstring) -- solved directly
    # rather than assumed, then passed as base_offset (not d[0]) so the DH
    # model itself matches the canonical file exactly
    z_off = float(np.mean(cart_measure_mm[:, 2] - qa_array[2] * 1000.0)) / 1000.0
    z_off_std = float(np.std(cart_measure_mm[:, 2] - qa_array[2] * 1000.0))
    print(f"  Fixed Z-datum offset (base_offset.z): {z_off*1000:.4f}mm "
          f"(std={z_off_std:.6f}mm across all samples)")
    base_offset = np.array([0.0, 0.0, z_off, 1.0, 0.0, 0.0, 0.0])
    return qa_array, cart_measure_mm, measurements, base_offset, IDENTITY.copy()


def main() -> int:
    robots = [
        ("Mitsubishi 7020 SCARA (mitsubishi_7020_measure, grid 1)",
         "/home/leon/calibration/mitsubishi_7020_measure",
         *load_canonical_a("6CRH7020.txt"), "_grid", None, None),
        ("Mitsubishi 7020 SCARA (mitsubishi_7020_measure, grid 2)",
         "/home/leon/calibration/mitsubishi_7020_measure",
         *load_canonical_a("6CRH7020.txt"), "_grid_2", None, None),
        ("Mitsubishi 7020 SCARA (mitsubishi_7020_measure, 3rd_set, "
         "known tool=-60mm)",
         "/home/leon/calibration/mitsubishi_7020_measure/3rd_set",
         *load_canonical_a("6CRH7020.txt"), None,
         IDENTITY.copy(), np.array([0.0, 0.0, -0.060, 1.0, 0.0, 0.0, 0.0])),
        ("Mitsubishi 6020 SCARA (sj_july_lasertracker)",
         "/home/leon/calibration/sj_july_lasertracker",
         *load_canonical_a("6020_scara.txt"), None, None, None),
    ]
    prepared = [(name, a1, a2, prepare_robot(name, directory, a1, a2, stem,
                                            known_base, known_tool))
               for name, directory, a1, a2, stem, known_base, known_tool
               in robots]

    for opt_method, opt_name in [(0, "Sam"), (2, "Levenberg-Marquardt")]:
        for name, a1, a2, (qa_array, cart_measure_mm, measurements,
                          base_offset, tool_offset) in prepared:
            run_robot(name, qa_array, cart_measure_mm, measurements,
                     base_offset, a1, a2, opt_method, opt_name,
                     tool_offset=tool_offset)

    print(f"\n\n{'#' * 78}")
    print("# LaserDistanceCalib (relative-displacement fit -- base/tool "
          "offset irrelevant by construction)")
    print(f"{'#' * 78}")
    for opt_method, opt_name in [(0, "Sam"), (2, "Levenberg-Marquardt")]:
        for name, a1, a2, (qa_array, cart_measure_mm, measurements,
                          base_offset, tool_offset) in prepared:
            run_robot_laser(name, qa_array, cart_measure_mm, measurements,
                            a1, a2, opt_method, opt_name)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
