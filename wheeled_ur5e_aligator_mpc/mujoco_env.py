"""
MuJoCo environment wrapper for the wheeled UR5e system.

Handles:
- Loading the MJCF model
- Mapping joint names to qpos addresses and actuator indices
- Reading current state (get_q)
- Writing desired joint positions to actuator ctrl (set_q_des)
- Advancing simulation (step)
- Updating target marker position (set_target_marker)
- Reading end-effector site position (get_ee_pos)
- Optional passive viewer launch

ALIGATOR version: 0.19.0
MuJoCo Python: mujoco >= 3.0
"""

import os
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np


# Ordered joint names matching WheeledUR5eModel.q_names
_JOINT_NAMES = [
    "base_x",
    "base_y",
    "base_z",
    "base_yaw",
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Ordered actuator names matching the position actuators in wheeled_ur5e.xml
_ACTUATOR_NAMES = [
    "act_base_x",
    "act_base_y",
    "act_base_z",
    "act_base_yaw",
    "act_shoulder_pan",
    "act_shoulder_lift",
    "act_elbow",
    "act_wrist_1",
    "act_wrist_2",
    "act_wrist_3",
]

_EE_SITE_NAME = "ee_site"
_TARGET_BODY_NAME = "target_body"


class MujocoWheeledUR5eEnv:
    """
    MuJoCo simulation environment for the wheeled UR5e robot.

    Parameters
    ----------
    xml_path : str
        Path to wheeled_ur5e.xml.
    render : bool
        Whether to launch the passive viewer.
    sim_dt : float
        MuJoCo simulation timestep (should match <option timestep> in MJCF).
    control_dt : float
        Control update interval. step() advances simulation by this much.
    """

    def __init__(
        self,
        xml_path: str,
        render: bool = False,
        sim_dt: float = 0.002,
        control_dt: float = 0.05,
    ) -> None:
        xml_path = str(xml_path)
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        # Override timestep if different from MJCF (usually keep MJCF value)
        self.model.opt.timestep = sim_dt
        self.sim_dt = sim_dt
        self.control_dt = control_dt
        self._steps_per_control = max(1, round(control_dt / sim_dt))

        # Build joint name → qpos address mapping
        self._qpos_addr: list[int] = []
        for jname in _JOINT_NAMES:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            if jid < 0:
                raise ValueError(f"Joint '{jname}' not found in model")
            self._qpos_addr.append(int(self.model.jnt_qposadr[jid]))

        # Build actuator name → ctrl index mapping
        self._act_idx: list[int] = []
        for aname in _ACTUATOR_NAMES:
            aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, aname)
            if aid < 0:
                raise ValueError(f"Actuator '{aname}' not found in model")
            self._act_idx.append(int(aid))

        # End-effector site index
        ee_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, _EE_SITE_NAME)
        if ee_id < 0:
            raise ValueError(f"Site '{_EE_SITE_NAME}' not found in model")
        self._ee_site_id = int(ee_id)

        # Target body index (mocap body)
        tb_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, _TARGET_BODY_NAME)
        if tb_id < 0:
            raise ValueError(f"Body '{_TARGET_BODY_NAME}' not found in model")
        self._target_body_id = int(tb_id)
        # mocap index for target body
        self._target_mocap_id = int(self.model.body_mocapid[tb_id])

        # Viewer
        self._viewer = None
        self._render = render
        if render:
            self._viewer = mujoco.viewer.launch_passive(self.model, self.data)

        # Reset to default state
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

    # ------------------------------------------------------------------
    # State access
    # ------------------------------------------------------------------

    def get_q(self) -> np.ndarray:
        """Read current generalized coordinates in q_names order."""
        q = np.array([self.data.qpos[addr] for addr in self._qpos_addr])
        return q

    def get_ee_pos(self) -> np.ndarray:
        """Read end-effector site world position from MuJoCo."""
        return np.array(self.data.site_xpos[self._ee_site_id])

    # ------------------------------------------------------------------
    # Control
    # ------------------------------------------------------------------

    def set_q_des(self, q_des: np.ndarray) -> None:
        """
        Write desired joint positions to position actuator ctrl array.
        q_des order must match _ACTUATOR_NAMES (same as q_names order).
        """
        for i, aid in enumerate(self._act_idx):
            self.data.ctrl[aid] = q_des[i]

    def set_target_marker(self, p_ref: np.ndarray) -> None:
        """Move the mocap target_body to the given world position."""
        self.data.mocap_pos[self._target_mocap_id] = p_ref

    def step(self, q_des: np.ndarray) -> None:
        """
        Write q_des to actuators then advance simulation by control_dt.
        Renders if viewer is active.
        """
        self.set_q_des(q_des)
        for _ in range(self._steps_per_control):
            mujoco.mj_step(self.model, self.data)
        if self._render and self._viewer is not None and self._viewer.is_running():
            self._viewer.sync()

    def reset(self, q0: np.ndarray | None = None) -> None:
        """Reset simulation to q0 or default zero state."""
        mujoco.mj_resetData(self.model, self.data)
        if q0 is not None:
            for i, addr in enumerate(self._qpos_addr):
                self.data.qpos[addr] = q0[i]
            # Set actuator ctrl to match initial position
            for i, aid in enumerate(self._act_idx):
                self.data.ctrl[aid] = q0[i]
        mujoco.mj_forward(self.model, self.data)

    def render(self) -> None:
        """Sync the viewer (if active)."""
        if self._render and self._viewer is not None and self._viewer.is_running():
            self._viewer.sync()

    def close(self) -> None:
        """Close the viewer if open."""
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None

    def is_viewer_running(self) -> bool:
        """Check whether viewer window is still open."""
        if self._viewer is None:
            return True  # no viewer, always "running"
        return self._viewer.is_running()
