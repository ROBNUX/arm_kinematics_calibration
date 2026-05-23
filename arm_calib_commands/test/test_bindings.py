"""Python-level smoke tests for the arm_calib_commands pybind11 module.

Verifies that each concrete calibration class is constructible from Python,
that the BaseCalibration abstract class is *not* directly constructible,
that GetName returns the expected string per class, and that types reused
from rob_motion_commands (refPose / Frame / Vec) interoperate correctly
with the CpsCartPose binding.
"""

import numpy as np
import pytest

import arm_calib_commands as c
import rob_motion_commands as motion


@pytest.fixture(scope="module")
def concrete_calibrations():
    return {
        "ScaraCalib": c.ScaraCalib(),
        "SixAxisCalib": c.SixAxisCalib(),
        "SingleAxisCalib": c.SingleAxisCalib(),
        "UjntCalib": c.UjntCalib(),
        "XyzGantryCalib": c.XyzGantryCalib(),
        "XyzUrCalib": c.XyzUrCalib(),
        "SerialArmCalib": c.SerialArmCalib(),
    }


def test_module_loads_and_exposes_classes():
    expected = {
        "BaseCalibration",
        "SerialArmCalib",
        "ScaraCalib",
        "SixAxisCalib",
        "SingleAxisCalib",
        "UjntCalib",
        "XyzGantryCalib",
        "XyzUrCalib",
    }
    missing = expected - set(dir(c))
    assert not missing, f"missing bindings: {sorted(missing)}"


def test_base_calibration_is_abstract():
    with pytest.raises(TypeError):
        c.BaseCalibration()


def test_concrete_calibrations_construct(concrete_calibrations):
    # The fixture itself constructs each; if any class failed, fixture setup
    # would have raised. Sanity-check that each is a BaseCalibration too.
    for name, instance in concrete_calibrations.items():
        assert isinstance(instance, c.BaseCalibration), (
            f"{name} should inherit BaseCalibration"
        )


@pytest.mark.parametrize(
    "class_name,expected_name",
    [
        ("ScaraCalib", "Scara calib"),
        ("SixAxisCalib", "SixAxis calib"),
        ("SingleAxisCalib", "SingleAxisCalib"),
        ("UjntCalib", "UJNT calibration"),
        ("XyzGantryCalib", "XYZGantry calib"),
        ("XyzUrCalib", "XYZUR calib"),
    ],
)
def test_get_name(concrete_calibrations, class_name, expected_name):
    assert concrete_calibrations[class_name].GetName() == expected_name


def test_set_opt_param_does_not_raise(concrete_calibrations):
    # Each concrete subclass binds setOptParam directly (via a templated
    # wrapper) so that pybind11 doesn't have to compose the upcast chain
    # through SerialArmCalib's virtual `serialArm` base.
    opt = np.array([0.0, 1.0], dtype=np.float64)  # [Sam method, region scale]
    for name in ("ScaraCalib", "SixAxisCalib", "SingleAxisCalib",
                 "UjntCalib", "XyzGantryCalib"):
        concrete_calibrations[name].setOptParam(opt)


def test_cross_module_types_available():
    # The arm_calib_commands module imports rob_motion_commands at init so
    # that refPose / Frame / Vec / Pose are usable here. Constructing them
    # should not raise, and they should be the same C++ type that
    # arm_calib_commands binds against.
    ee = motion.Frame(motion.Vec(0.5, 0.0, 0.0))
    base = motion.Frame(motion.Vec(1.0, 0.0, 0.0))
    tool = motion.Frame(motion.Vec(0.0, 1.0, 0.0))
    rp = motion.refPose(ee, base, tool, [0], [0])

    ok, base_out = rp.getBase()
    assert ok
    assert base_out.getTranslation().x() == pytest.approx(1.0)


def test_load_calib_param_set_returns_bool(concrete_calibrations):
    # LoadCalibParamSet is now bound directly on each affected concrete class
    # (see py_LoadCalibParamSet_T<Derived>); pybind11 no longer has to compose
    # the upcast chain through SerialArmCalib's virtual base.
    for name in ("ScaraCalib", "SixAxisCalib", "SingleAxisCalib",
                 "UjntCalib", "XyzGantryCalib"):
        cal_dh = np.zeros(0, dtype=np.float64)
        result = concrete_calibrations[name].LoadCalibParamSet(cal_dh)
        assert isinstance(result, bool)


def test_get_calib_param_set_returns_tuple(concrete_calibrations):
    # GetCalibParamSet is similarly bound directly on each affected concrete
    # class.
    for name in ("ScaraCalib", "SixAxisCalib", "SingleAxisCalib",
                 "UjntCalib", "XyzGantryCalib"):
        ok, vec = concrete_calibrations[name].GetCalibParamSet(0)
        assert isinstance(ok, bool)
        assert isinstance(vec, np.ndarray)
