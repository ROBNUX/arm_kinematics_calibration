# arm_kinematics_calibration

A ROS 2 workspace for **robot kinematic-model calibration**. Sits on top of
[arm_kinematics_trajectory](../arm_kinematics_trajectory/) (which provides the
underlying FK/IK plugins, geometric types, and Python bindings) and adds
calibration algorithms plus a thin Python binding layer.

## Packages

| Package | Purpose |
|---|---|
| [`robnux_utilities`](robnux_utilities/) | CSV I/O (`RPE_Utility`) and the gradient-descent solver (`DecentAlg`) used by the calibration loop |
| [`arm_calib_kinematics`](arm_calib_kinematics/) | `BaseCalibration` interface and seven concrete robot calibrators — `SerialArmCalib`, `ScaraCalib`, `SixAxisCalib`, `SingleAxisCalib`, `UjntCalib`, `XyzGantryCalib`, `XyzUrCalib` |
| [`arm_calib_commands`](arm_calib_commands/) | pybind11 module exposing the above to Python; re-uses `Frame`/`Pose`/`refPose` from `rob_motion_commands` |

## What it calibrates

Seven robot kinematic families are supported:

- **SCARA**, **single-axis module**, **U-joint**, **XYZ gantry**
- **5-axis XYZ+UR** (XYZ stage + UR-style 2-DoF wrist)
- **6-axis serial arm**
- **General serial arm** (arbitrary DoF)

For each, two measurement modalities are implemented:

1. **Direct 3D-coordinate measurement** — e.g. laser tracker giving
   `(x, y, z)` of the TCP at each pose.
2. **Laser-distance measurement** — three orthogonal laser displacement
   sensors measuring distance to fixed planes.

Two gradient-descent variants are available for the inner optimization
loop: **Sam-type** and **Sam-Adam-type** (Adam momentum on top of the Sam
search-region step). Pick via `BaseCalibration::setOptParam`.

The calibration API also includes:

- TCP calibration via a single mechanical distance probe
  (`CalibTCPDistMethod`)
- Base-frame calibration from 8-point edge probing
  (`CalibBaseFrame`)
- Per-pose Cartesian / joint compensation (`CpsCartPose`, `CpsJnt`)
- Whole-path correction through the calibrated DH model (`CpsRobPath`)
- Round-tripping the calibrated parameter vector
  (`GetCalibParamSet` / `LoadCalibParamSet`)

## Dependencies

ROS 2 (Humble or newer), Eigen3, pybind11, pluginlib, the upstream
[arm_kinematics_trajectory](../arm_kinematics_trajectory/) workspace.
Tests additionally need `ament_cmake_gtest` and `python3-numpy`.

## Build

```bash
cd ~/your_ws/src
# arm_kinematics_trajectory must be present alongside this one
git clone <arm_kinematics_calibration-repo>
cd ..
colcon build
source install/setup.bash
```

## Run the tests

```bash
colcon test --packages-select \
  robnux_utilities arm_calib_kinematics arm_calib_commands
colcon test-result --verbose
```

`arm_calib_kinematics` and `robnux_utilities` ship C++ gtest suites.
`arm_calib_commands` adds a pytest suite exercising the bindings.

## Python quickstart

```python
import numpy as np
from arm_calib_commands import ScaraCalib

calib = ScaraCalib()

# Configure the optimizer: opt_method=0 → Sam-type, sam_region_scale=1.0
calib.setOptParam(np.array([0.0, 1.0]))

# --- Direct 3D-coordinate calibration ------------------------------------
# Each column of cart_measure is (x, y, z, R, P, Y, branch, turn).
# Each column of qa_array is a joint vector taken at the same instant.
# measurements: 3 x N array of measured TCP positions (xyz).
base_off  = np.zeros(7)                       # robot base offset (pose7)
tool_off  = np.zeros(7)                       # tool offset (pose7)
cart_measure = ...                            # 8 x N
qa_array     = ...                            # DoF x N
measurements = ...                            # 3 x N
err = calib.DirectMesCalib(base_off, tool_off, cart_measure,
                           measurements, qa_array)
print(f"mean point error after calibration: {err:.4f} m")

# --- Read out the calibrated DH vector ----------------------------------
# (size depends on the robot family — see the calibration class header)
ok, cal_dh = calib.GetCalibParamSet(param_size=16)
np.savetxt("scara_calibrated.csv", cal_dh)

# --- Per-pose joint compensation ----------------------------------------
# Construct the desired refPose using types from rob_motion_commands.
import rob_motion_commands as motion
ee   = motion.Frame(motion.Vec(0.5, 0.0, 0.1))
base = motion.Frame(motion.Vec(1.0, 0.0, 0.0))
tool = motion.Frame.Identity()
target = motion.refPose(ee, base, tool, [0], [0, 0, 0, 0])
status, cq = calib.CpsJnt(target, dof=4)
```

## Notes on the bindings

`arm_calib_commands` imports `rob_motion_commands` at module load so that
`Frame`, `Pose`, `refPose`, `Vec` and friends are shared C++ types between
the two modules — no duplicate type registration.

Because `SerialArmCalib` virtually inherits `serialArm`, pybind11's
multi-step upcast through that virtual base can produce an incorrect
`BaseCalibration*` for `Scara`/`SixAxis`/`SingleAxis`/`Ujnt`/`XyzGantry`
instances. The bindings work around this by defining
`setOptParam` / `LoadCalibParamSet` / `GetCalibParamSet` on each concrete
subclass via templated wrappers (see
[`module_calib_command.cpp`](arm_calib_commands/src/intp/module_calib_command.cpp)),
so the compiler resolves the cast in one shot instead.
