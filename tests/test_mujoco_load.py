"""Tests for MuJoCo MJCF model loading and joint/actuator/site presence."""

import sys
from pathlib import Path
import pytest

_project_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_project_root))

_XML_PATH = _project_root / "assets" / "wheeled_ur5e.xml"

EXPECTED_JOINTS = [
    "base_x", "base_y", "base_z", "base_yaw",
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]
EXPECTED_ACTUATORS = [
    "act_base_x", "act_base_y", "act_base_z", "act_base_yaw",
    "act_shoulder_pan", "act_shoulder_lift", "act_elbow",
    "act_wrist_1", "act_wrist_2", "act_wrist_3",
]


@pytest.fixture(scope="module")
def mj():
    """Load mujoco and the model once for all tests."""
    import mujoco
    model = mujoco.MjModel.from_xml_path(str(_XML_PATH))
    data = mujoco.MjData(model)
    return mujoco, model, data


def test_xml_exists():
    assert _XML_PATH.exists(), f"MJCF not found: {_XML_PATH}"


def test_model_loads(mj):
    mujoco, model, data = mj
    assert model is not None
    assert data is not None


def test_all_joints_present(mj):
    mujoco, model, data = mj
    for jname in EXPECTED_JOINTS:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
        assert jid >= 0, f"Joint '{jname}' not found in model"


def test_all_actuators_present(mj):
    mujoco, model, data = mj
    for aname in EXPECTED_ACTUATORS:
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, aname)
        assert aid >= 0, f"Actuator '{aname}' not found in model"


def test_ee_site_present(mj):
    mujoco, model, data = mj
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
    assert sid >= 0, "Site 'ee_site' not found in model"


def test_target_body_present(mj):
    mujoco, model, data = mj
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target_body")
    assert bid >= 0, "Body 'target_body' not found in model"


def test_target_is_mocap(mj):
    mujoco, model, data = mj
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target_body")
    mocap_id = model.body_mocapid[bid]
    assert mocap_id >= 0, "target_body is not a mocap body"


def test_nq_equals_ten(mj):
    """Model should have exactly 10 DOFs (one per joint)."""
    mujoco, model, data = mj
    assert model.nq == 10, f"Expected nq=10, got nq={model.nq}"


def test_nu_equals_ten(mj):
    """Model should have exactly 10 actuators."""
    mujoco, model, data = mj
    assert model.nu == 10, f"Expected nu=10, got nu={model.nu}"


def test_ee_site_position_reasonable(mj):
    """At zero configuration, ee_site should be at a plausible location."""
    import mujoco
    import numpy as np
    mujoco_mod, model, data = mj
    mujoco_mod.mj_resetData(model, data)
    mujoco_mod.mj_forward(model, data)
    sid = mujoco_mod.mj_name2id(model, mujoco_mod.mjtObj.mjOBJ_SITE, "ee_site")
    ee = data.site_xpos[sid]
    # EE should be within reasonable workspace
    assert np.linalg.norm(ee) > 0.05, f"EE too close to origin: {ee}"
    assert np.linalg.norm(ee) < 3.0, f"EE too far: {ee}"
