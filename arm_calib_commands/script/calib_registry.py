#!/usr/bin/env python3
"""
Shared registry of every calibrator type in arm_calib_kinematics, for use by
random_calib_tests.py.

Mirrors robnux_arm_sim/scripts/arm_registry.py's pattern (from the sibling
arm_kinematics_trajectory repo): each CalibSpec bundles a nominal DH geometry
plus everything needed to (1) build "true" and "nominal" rob_motion_commands
Robot instances that share the SAME C++ FK model the calibrator itself uses
internally, and (2) build the matching arm_calib_commands calibrator object.
Testing against the real C++ FK (rather than a hand-rolled Python
approximation) is what actually exercises the calibration algorithms
correctly -- see memory/project_calib_bug_hunt.md for why this matters.

This file has no ROS-execution side effects; importing it just builds numpy
parameter arrays. arm_calib_commands / rob_motion_commands are only imported
by consumers that actually construct robots/calibrators.
"""
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

PI = math.pi
IDENTITY = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])


@dataclass
class CalibSpec:
    name: str            # calibrator class name, e.g. "ScaraCalib"
    robname: str          # rob_motion_commands.Robot plugin name (ground truth FK)
    dof: int
    alpha: List[float]
    a: List[float]
    theta: List[float]
    d: List[float]
    profile_args: Tuple[float, float, float, float, float, float]
    joint_bounds: List[Tuple[float, float]]
    # indices (0-based, within the DoF-length a/d vectors) that carry a
    # physically meaningful, nonzero link length/offset -- these are the DH
    # entries perturbed to build the synthetic "true" (miscalibrated) robot.
    # Perturbing a structurally-zero entry (e.g. a SCARA's a[2]=0, a
    # not-installed link) wouldn't correspond to any real calibration error.
    perturb_a_idx: List[int] = field(default_factory=list)
    perturb_d_idx: List[int] = field(default_factory=list)
    # DecentAlg's SAM-type optimizer (the only one reachable in practice --
    # see project_calib_bug_hunt memory) takes fixed-granularity gradient
    # steps sized around SAM_RHO (1e-3) * sam_region_scale_, not steps scaled
    # to the actual residual. Planting errors well below that granularity
    # (sub-mm) makes "improvement" noisy/sometimes negative even though the
    # algorithm is working as designed -- 5mm matches realistic DH
    # miscalibration magnitudes and sits safely above that floor.
    perturb_scale: float = 0.005  # meters
    n_measures: int = 60
    # LaserDistanceCalib fits *relative* displacements referenced against
    # sample 0 (differencing out anything constant across samples). A pure
    # translational/prismatic mechanism whose only perturbable DH entries
    # are constant offsets along the joint's own travel axis (e.g. a pure
    # XYZ gantry's d[i], or a single linear axis's d[0]) produces a planted
    # error that is *information-theoretically invisible* to that
    # differencing -- there is no observable signal to calibrate against,
    # so testing recovery there isn't meaningful (see
    # memory/project_calib_bug_hunt.md). Revolute-adjacent mechanisms don't
    # have this issue: a perturbed `a[i]` changes the lever arm, so its
    # contribution to tip position varies with the other joints' angles
    # across samples and remains observable under differencing.
    laser_calib_testable: bool = True
    notes: str = ""

    def build_kine_para(self, alpha=None, a=None, theta=None, d=None) -> np.ndarray:
        alpha = self.alpha if alpha is None else alpha
        a = self.a if a is None else a
        theta = self.theta if theta is None else theta
        d = self.d if d is None else d
        return np.concatenate([alpha, a, theta, d, IDENTITY]).astype(np.float64)


def _scara_spec() -> CalibSpec:
    return CalibSpec(
        name="ScaraCalib", robname="scara", dof=4,
        alpha=[0.0, 0.0, math.pi, 0.0],
        a=[0.30, 0.30, 0.0, 0.0],
        theta=[0.0, 0.0, 0.0, 0.0],
        d=[0.30, 0.0, 0.20, 0.05],
        profile_args=(0.05, 0.2, 2.0, 0.1, 0.5, 5.0),
        joint_bounds=[(-1.8, 1.8), (-2.3, 2.3), (-0.15, 0.15), (-3.0, 3.0)],
        perturb_a_idx=[0, 1],
        perturb_d_idx=[2],
    )


def _sixaxis_spec() -> CalibSpec:
    return CalibSpec(
        name="SixAxisCalib", robname="sixaxis_1", dof=6,
        alpha=[0, -PI / 2, 0, -PI / 2, PI / 2, -PI / 2],
        a=[0, 0.075, 0.25, 0.03, 0, 0],
        theta=[0, PI / 2, 0, 0, 0, 0],
        d=[0.33, 0, 0, 0.25, 0, 0.09],
        profile_args=(0.05, 0.2, 2.0, 0.1, 0.5, 5.0),
        joint_bounds=[(-PI * 0.5, PI * 0.5)] * 3 + [(-PI * 0.5, PI * 0.5)] +
                     [(-1.2, -0.3)] + [(-PI * 0.5, PI * 0.5)],
        perturb_a_idx=[1, 2, 3],
        perturb_d_idx=[0, 3],
    )


def _ujnt_spec() -> CalibSpec:
    return CalibSpec(
        name="UjntCalib", robname="ujnt", dof=2,
        alpha=[0.0, PI / 2],
        a=[0.15, 0.0],
        theta=[0.0, 0.0],
        d=[0.30, 0.0],
        profile_args=(0.05, 0.2, 2.0, 0.1, 0.5, 5.0),
        joint_bounds=[(-1.3, 1.3), (-1.3, 1.3)],
        perturb_a_idx=[0],
        perturb_d_idx=[0],
    )


def _xyz_gantry_spec() -> CalibSpec:
    return CalibSpec(
        name="XyzGantryCalib", robname="xyz_gantry", dof=3,
        alpha=[0.0, PI / 2, -PI / 2],
        a=[0.0, 0.0, 0.0],
        theta=[0.0, 0.0, 0.0],
        d=[0.0, 0.0, 0.0],
        profile_args=(0.1, 0.5, 5.0, 0.1, 0.5, 5.0),
        joint_bounds=[(-0.45, -0.02), (-0.4, 0.4), (-0.4, 0.4)],
        perturb_a_idx=[],
        perturb_d_idx=[0, 1, 2],
        laser_calib_testable=False,
        notes="pure XYZ gantry: every perturbable DH entry is a constant "
              "per-axis offset, invisible to LaserDistanceCalib's "
              "sample-0-referenced differencing",
    )


def _single_axis_spec() -> CalibSpec:
    return CalibSpec(
        name="SingleAxisCalib", robname="single_axis", dof=1,
        alpha=[0.0], a=[0.0], theta=[0.0], d=[0.0],
        profile_args=(0.1, 0.5, 5.0, 0.1, 0.5, 5.0),
        joint_bounds=[(-PI * 0.9, PI * 0.9)],
        perturb_a_idx=[], perturb_d_idx=[0],
        laser_calib_testable=False,
        notes="single linear axis: its only DH entry is a constant "
              "offset, invisible to LaserDistanceCalib's "
              "sample-0-referenced differencing",
    )


def build_all_specs() -> List[CalibSpec]:
    return [
        _scara_spec(),
        _sixaxis_spec(),
        _ujnt_spec(),
        _xyz_gantry_spec(),
        _single_axis_spec(),
    ]


def spec_by_name(name: str) -> Optional[CalibSpec]:
    for s in build_all_specs():
        if s.name == name:
            return s
    return None
