#!/usr/bin/env python3
"""
Random DirectMesCalib/LaserDistanceCalib ground-truth recovery test across
every calibrator type in arm_calib_kinematics.

Mirrors robnux_arm_sim/scripts/test_fk_ik_all_arms.py's methodology (from the
sibling arm_kinematics_trajectory repo), adapted from FK/IK round-trips to
calibration recovery:

For each CalibSpec in calib_registry.build_all_specs(), run N independent
trials. Each trial:
  1. Plants a random small DH error (mm-scale) on the spec's
     perturb_a_idx/perturb_d_idx entries -> "true" (physically miscalibrated)
     robot.
  2. Builds "true" and "nominal" rob_motion_commands.Robot instances from the
     SAME C++ FK model the calibrator itself uses internally (not a
     hand-rolled Python approximation -- see calib_registry module docstring
     for why that distinction matters).
  3. Samples random joint configs within the spec's safe bounds, generates
     synthetic measurements via the true robot's FK and the nominal
     robot's FK (playing the role of the "controller's own, uncalibrated"
     readings).
  4. Runs DirectMesCalib + VerifyDirectMesCalib, and separately
     LaserDistanceCalib + VerifyLaserDistanceCalib, on a freshly-constructed
     nominal calibrator instance.
  5. Checks: no crash, valid (non-negative) return codes, and that
     calibration does not make accuracy meaningfully WORSE (VerifyXxxCalib's
     improvement delta is not a large negative regression).

     Full sub-mm recovery of every individual DH parameter is NOT required
     to pass -- ReduceJacobian's greedy, cached-once column-rank reduction
     can permanently drop a structurally-hard-to-separate parameter's column
     (e.g. ScaraCalib's d[2], confirmed via a dedicated ground-truth check;
     see memory/project_calib_bug_hunt.md). That is a real, documented
     algorithmic limitation, not a per-run bug, so it must not fail every
     run of this broader regression sweep. This test's job is to catch
     crashes, wrong-sign regressions, and outright non-functional algorithms
     -- the stricter per-parameter recovery check lives in
     ground_truth_calib_test.py's ScaraCalib-specific deep dive.

Each calibrator type is tested in its own subprocess: a native Eigen
assertion abort in one type's code path must not take down the others'
results (same reasoning as test_fk_ik_all_arms.py).

Run (ROS2 + /opt/robnux overlay sourced):
    python3 test_calib_all_types.py [N] [--type NAME] [--seed S]
"""
import argparse
import json
import math
import subprocess
import sys

import numpy as np

import calib_registry as reg

DEFAULT_N = 8
# A "regression" is improvement worse than -max(floor, frac * before). The
# absolute floor (0.5mm) absorbs numerical noise -- meaningful for
# translation-only mechanisms (XyzGantryCalib, SingleAxisCalib) where
# LaserDistanceCalib's sample-0-referenced differencing structurally cannot
# see a constant DH offset at all, so "before" legitimately rounds to ~0 and
# any residual noise is enormous in *relative* terms while being physically
# negligible in absolute terms.
REGRESSION_ABS_FLOOR = 0.0005  # meters
REGRESSION_REL_FRAC = 0.5


def is_regression(before: float, improvement: float) -> bool:
    return improvement < -max(REGRESSION_ABS_FLOOR, REGRESSION_REL_FRAC * before)


def fk_positions(robot, qa_array):
    n = qa_array.shape[1]
    out = np.zeros((3, n))
    ok_all = True
    for i in range(n):
        ok, loc = robot.ForwardKin(qa_array[:, i])
        if not ok:
            ok_all = False
            continue
        out[:, i] = [loc.x, loc.y, loc.z]
    return out, ok_all


def sample_qa(spec: reg.CalibSpec, rng: np.random.Generator, n: int) -> np.ndarray:
    qa = np.zeros((spec.dof, n))
    for i, (lo, hi) in enumerate(spec.joint_bounds):
        qa[i] = rng.uniform(lo, hi, n)
    return qa


def signed_magnitude(rng: np.random.Generator, scale: float,
                     min_frac: float = 0.4) -> float:
    """A perturbation with a guaranteed-observable minimum magnitude.

    Plain rng.uniform(-scale, scale) occasionally draws values close to 0,
    which (for a system with only one truly identifiable DH parameter, e.g.
    SingleAxisCalib) fall below DecentAlg's fixed SAM-step correction
    granularity and register as a false "regression" -- not a real bug, just
    an unfair single-trial draw. Sampling the magnitude away from 0 avoids
    that without loosening the regression-detection tolerance itself.
    """
    mag = rng.uniform(min_frac, 1.0) * scale
    return mag * rng.choice([-1.0, 1.0])


def plant_true_dh(spec: reg.CalibSpec, rng: np.random.Generator):
    a_true = list(spec.a)
    d_true = list(spec.d)
    for idx in spec.perturb_a_idx:
        a_true[idx] += signed_magnitude(rng, spec.perturb_scale)
    for idx in spec.perturb_d_idx:
        d_true[idx] += signed_magnitude(rng, spec.perturb_scale)
    return a_true, d_true


def run_direct_mes_trial(c, m, spec: reg.CalibSpec, rng: np.random.Generator):
    a_true, d_true = plant_true_dh(spec, rng)
    nominal_para = spec.build_kine_para()
    true_para = spec.build_kine_para(a=a_true, d=d_true)

    pf = m.Profile(*spec.profile_args)
    true_robot = m.Robot(spec.robname, true_para, reg.IDENTITY, reg.IDENTITY, pf)
    nominal_robot = m.Robot(spec.robname, nominal_para, reg.IDENTITY, reg.IDENTITY, pf)

    qa_array = sample_qa(spec, rng, spec.n_measures)
    measurements, ok1 = fk_positions(true_robot, qa_array)
    nominal_pos, ok2 = fk_positions(nominal_robot, qa_array)
    true_robot.Shutdown()
    nominal_robot.Shutdown()
    if not (ok1 and ok2):
        return {"skipped": True, "reason": "FK unreachable sample"}

    cart_measure = np.zeros((8, spec.n_measures))
    cart_measure[0:3] = nominal_pos

    calib_cls = getattr(c, spec.name)
    calib = calib_cls(nominal_para)
    err_after = calib.DirectMesCalib(reg.IDENTITY, reg.IDENTITY, cart_measure,
                                     measurements, qa_array)
    result = calib.VerifyDirectMesCalib(reg.IDENTITY, reg.IDENTITY, cart_measure,
                                        measurements, qa_array)
    return {
        "skipped": False,
        "err_after": float(err_after),
        "before": float(result[0]),
        "improvement": float(result[1]),
    }


def run_laser_trial(c, m, spec: reg.CalibSpec, rng: np.random.Generator):
    a_true, d_true = plant_true_dh(spec, rng)
    nominal_para = spec.build_kine_para()
    true_para = spec.build_kine_para(a=a_true, d=d_true)

    pf = m.Profile(*spec.profile_args)
    true_robot = m.Robot(spec.robname, true_para, reg.IDENTITY, reg.IDENTITY, pf)
    nominal_robot = m.Robot(spec.robname, nominal_para, reg.IDENTITY, reg.IDENTITY, pf)

    qa_array = sample_qa(spec, rng, spec.n_measures)
    true_pos, ok1 = fk_positions(true_robot, qa_array)
    nominal_pos, ok2 = fk_positions(nominal_robot, qa_array)
    true_robot.Shutdown()
    nominal_robot.Shutdown()
    if not (ok1 and ok2):
        return {"skipped": True, "reason": "FK unreachable sample"}

    cart_measure = np.zeros((8, spec.n_measures))
    cart_measure[0:3] = nominal_pos
    laser2CartMap = np.eye(3)
    laser_measure = true_pos.copy()

    calib_cls = getattr(c, spec.name)
    calib = calib_cls(nominal_para)
    err_after = calib.LaserDistanceCalib(reg.IDENTITY, reg.IDENTITY, laser2CartMap,
                                         cart_measure, qa_array, laser_measure)
    result = calib.VerifyLaserDistanceCalib(reg.IDENTITY, reg.IDENTITY, laser2CartMap,
                                            cart_measure, qa_array, laser_measure)
    return {
        "skipped": False,
        "err_after": float(err_after),
        "before": float(result[0]),
        "improvement": float(result[1]),
    }


def run_spec_test(spec_name: str, n_trials: int, seed: int) -> dict:
    import arm_calib_commands as c
    import rob_motion_commands as m

    spec = reg.spec_by_name(spec_name)
    rng = np.random.default_rng(seed)

    algos = [("DirectMesCalib", run_direct_mes_trial)]
    if spec.laser_calib_testable:
        algos.append(("LaserDistanceCalib", run_laser_trial))
    trials = {algo: [] for algo, _ in algos}
    errors = []
    for algo, runner in algos:
        for _ in range(n_trials):
            try:
                res = runner(c, m, spec, rng)
            except Exception as e:  # noqa: BLE001 -- want any failure captured
                res = {"skipped": False, "exception": str(e)}
            trials[algo].append(res)

    def summarize(records):
        used = [r for r in records if not r.get("skipped")]
        exc = [r for r in used if "exception" in r]
        neg_ret = [r for r in used if "err_after" in r and r["err_after"] < 0]
        regressions = [r for r in used if "improvement" in r and
                       is_regression(r["before"], r["improvement"])]
        return {
            "n_used": len(used), "n_skipped": len(records) - len(used),
            "n_exceptions": len(exc), "exceptions": [r["exception"] for r in exc],
            "n_negative_return": len(neg_ret),
            "n_regressions": len(regressions),
            "regressions": regressions[:3],
        }

    summary = {algo: summarize(recs) for algo, recs in trials.items()}
    passed = all(
        s["n_exceptions"] == 0 and s["n_negative_return"] == 0 and
        s["n_regressions"] == 0 and s["n_used"] > 0
        for s in summary.values()
    )
    return {"name": spec_name, "passed": passed, "summary": summary, "crashed": False}


def print_result(res: dict, n_requested: int):
    status = "PASS" if res["passed"] else "FAIL"
    if res.get("crashed"):
        print(f"[{status}] {res['name']:16s} CRASHED: {res.get('error', 'unknown')}")
        return
    print(f"[{status}] {res['name']:16s}")
    for algo, s in res["summary"].items():
        print(f"    {algo:20s} used={s['n_used']}/{n_requested} "
              f"skipped={s['n_skipped']} exceptions={s['n_exceptions']} "
              f"negative_return={s['n_negative_return']} "
              f"regressions={s['n_regressions']}")
        for e in s["exceptions"][:2]:
            print(f"        exception: {e}")
        for r in s["regressions"][:2]:
            print(f"        regression: before={r['before']:.6f} "
                  f"improvement={r['improvement']:.6f}")


def run_child(n: int, spec_name: str, seed: int) -> int:
    res = run_spec_test(spec_name, n, seed)
    print("RESULT_JSON:" + json.dumps(res))
    return 0 if res["passed"] else 1


def run_parent(n: int, specs, seed: int):
    results = []
    for spec in specs:
        proc = subprocess.run(
            [sys.executable, __file__, str(n), "--type", spec.name,
             "--seed", str(seed), "--_child"],
            capture_output=True, text=True,
        )
        res = None
        for line in proc.stdout.splitlines():
            if line.startswith("RESULT_JSON:"):
                res = json.loads(line[len("RESULT_JSON:"):])
                break
        if res is None:
            tail = "\n".join(proc.stderr.strip().splitlines()[-8:])
            res = {"name": spec.name, "passed": False, "crashed": True,
                  "error": f"exit code {proc.returncode}; stderr tail:\n{tail}"}
        print_result(res, n)
        results.append(res)
    return results


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("n", nargs="?", type=int, default=DEFAULT_N,
                     help="random trials per calibrator/algorithm (default 8)")
    ap.add_argument("--type", default=None, help="only test this calibrator name")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--_child", action="store_true", help=argparse.SUPPRESS)
    args = ap.parse_args()

    if args._child:
        return run_child(args.n, args.type, args.seed)

    specs = reg.build_all_specs()
    if args.type:
        specs = [s for s in specs if s.name == args.type]
        if not specs:
            print(f"unknown --type {args.type!r}; known: "
                  f"{[s.name for s in reg.build_all_specs()]}")
            return 2

    print("=" * 78)
    print(f"Calibration recovery test — {args.n} trials/type/algorithm, "
          f"seed={args.seed}")
    print("=" * 78)
    results = run_parent(args.n, specs, args.seed)

    print("=" * 78)
    n_fail = sum(1 for r in results if not r["passed"])
    if n_fail:
        print(f"RESULT: {n_fail}/{len(results)} calibrator type(s) FAILED")
    else:
        print(f"RESULT: all {len(results)} calibrator type(s) PASSED")
    return 1 if n_fail else 0


if __name__ == "__main__":
    raise SystemExit(main())
