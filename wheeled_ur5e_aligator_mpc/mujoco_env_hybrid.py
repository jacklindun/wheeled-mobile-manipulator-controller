"""
MuJoCo environment for hybrid kino-dynamic mode (Phase 4).

Key differences from mujoco_env.py:
- Uses wheeled_ur5e_hybrid.xml (base position actuators, arm motor actuators)
- State: 16-dim [q_base(4), q_arm(6), v_arm(6)]
- Control: 10-dim [v_base(4), tau_arm(6)]
- Base control: still integrates velocity → position (same as Phase 1-3)
- Arm control: direct torque application
"""

import os
from pathlib import Path

import numpy as np
import mujoco
import mujoco.viewer


class MujocoWheeledUR5eHybridEnv:
    """
    MuJoCo environment for hybrid kino-dynamic wheeled UR5e (Phase 4).

    State: x = [q_base(4), q_arm(6), v_arm(6)] = 16-dim
    Control: u = [v_base(4), tau_arm(6)] = 10-dim
    """

    def __init__(self, render: bool = False):
        """
        Parameters
        ----------
        render : bool
            Enable rendering via mujoco.viewer.
        """
        mjcf_path = Path(__file__).resolve().parents[1] / "assets" / "wheeled_ur5e_hybrid.xml"
        self.model = mujoco.MjModel.from_xml_path(str(mjcf_path))
        self.data = mujoco.MjData(self.model)

        self._render = render
        self._viewer = None

        # Joint names (10 total: 4 base + 6 arm)
        self._joint_names = [
            "base_x", "base_y", "base_z", "base_yaw",
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
        ]

        # Actuator names (10 total: 4 base position + 6 arm motor)
        self._actuator_names = [
            "act_base_x", "act_base_y", "act_base_z", "act_base_yaw",
            "act_shoulder_pan", "act_shoulder_lift", "act_elbow",
            "act_wrist_1", "act_wrist_2", "act_wrist_3",
        ]

        # Build mappings
        self._joint_qpos_adr = {}
        self._joint_qvel_adr = {}
        for jname in self._joint_names:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            self._joint_qpos_adr[jname] = self.model.jnt_qposadr[jid]
            self._joint_qvel_adr[jname] = self.model.jnt_dofadr[jid]

        self._actuator_id = {}
        for aname in self._actuator_names:
            aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, aname)
            self._actuator_id[aname] = aid

        # EE site and target mocap
        self._ee_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
        self._target_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "target_body")

        # Base position targets (for position actuators, integrated from velocity commands)
        self._q_base_target = np.zeros(4)

        # Initialize viewer if rendering
        if render:
            self._viewer = mujoco.viewer.launch_passive(self.model, self.data)

    def get_state(self) -> np.ndarray:
        """
        Get current 16-dim state: [q_base(4), q_arm(6), v_arm(6)].

        Returns
        -------
        x : (16,) state vector
        """
        q_base = np.array([self.data.qpos[self._joint_qpos_adr[jn]] for jn in self._joint_names[:4]])
        q_arm = np.array([self.data.qpos[self._joint_qpos_adr[jn]] for jn in self._joint_names[4:]])
        v_arm = np.array([self.data.qvel[self._joint_qvel_adr[jn]] for jn in self._joint_names[4:]])
        return np.concatenate([q_base, q_arm, v_arm])

    def get_q(self) -> np.ndarray:
        """
        Get current 10-dim configuration: [q_base(4), q_arm(6)].

        Returns
        -------
        q : (10,) configuration vector
        """
        return np.array([self.data.qpos[self._joint_qpos_adr[jn]] for jn in self._joint_names])

    def get_ee_pos(self) -> np.ndarray:
        """
        Get current end-effector position from ee_site.

        Returns
        -------
        p_ee : (3,) position vector
        """
        return self.data.site_xpos[self._ee_site_id].copy()

    def set_control(self, u: np.ndarray) -> None:
        """
        Set control commands: u = [v_base(4), tau_arm(6)].

        For base: integrate v_base → q_base_target, send to position actuators.
        For arm: send tau_arm directly to motor actuators.

        Parameters
        ----------
        u : (10,) control vector [v_base(4), tau_arm(6)]
        """
        v_base = u[:4]
        tau_arm = u[4:10]

        # Integrate base velocity → position target (dt = model.opt.timestep)
        dt = self.model.opt.timestep
        q_base_current = np.array([self.data.qpos[self._joint_qpos_adr[jn]] for jn in self._joint_names[:4]])

        # Body-frame velocity → world-frame (for base_x, base_y)
        base_yaw = q_base_current[3]
        c, s = np.cos(base_yaw), np.sin(base_yaw)
        vx_body, vy_body = v_base[0], v_base[1]
        vx_world = c * vx_body - s * vy_body
        vy_world = s * vx_body + c * vy_body

        # Integrate
        self._q_base_target[0] += dt * vx_world  # base_x
        self._q_base_target[1] += dt * vy_world  # base_y
        self._q_base_target[2] += dt * v_base[2]  # base_z
        self._q_base_target[3] += dt * v_base[3]  # base_yaw
        # Wrap yaw
        self._q_base_target[3] = np.arctan2(np.sin(self._q_base_target[3]), np.cos(self._q_base_target[3]))

        # Send to base actuators (position control)
        for i, aname in enumerate(self._actuator_names[:4]):
            aid = self._actuator_id[aname]
            self.data.ctrl[aid] = self._q_base_target[i]

        # Send to arm actuators (torque control)
        for i, aname in enumerate(self._actuator_names[4:]):
            aid = self._actuator_id[aname]
            self.data.ctrl[aid] = tau_arm[i]

    def set_target_marker(self, p_target: np.ndarray) -> None:
        """
        Set target marker (mocap body) position for visualization.

        Parameters
        ----------
        p_target : (3,) desired EE position
        """
        self.data.mocap_pos[0] = p_target

    def step(self, substeps: int = 1) -> None:
        """
        Step the simulation forward.

        Parameters
        ----------
        substeps : int
            Number of simulation steps per call.
        """
        for _ in range(substeps):
            mujoco.mj_step(self.model, self.data)

        if self._render and self._viewer is not None:
            self._viewer.sync()

    def reset(self, q0: np.ndarray | None = None) -> None:
        """
        Reset simulation to initial configuration.

        Parameters
        ----------
        q0 : (10,) or None
            Initial configuration [q_base(4), q_arm(6)]. If None, use nominal.
        """
        if q0 is None:
            q0 = np.zeros(10)
            q0[2] = 0.2  # base_z = 0.2 m
            q0[4] = np.pi  # shoulder_pan = π (nominal)
            q0[5] = np.pi / 3  # shoulder_lift

        mujoco.mj_resetData(self.model, self.data)

        # Set qpos
        for i, jname in enumerate(self._joint_names):
            self.data.qpos[self._joint_qpos_adr[jname]] = q0[i]

        # Initialize base target to current position
        self._q_base_target[:] = q0[:4]

        # Zero velocities
        self.data.qvel[:] = 0.0

        mujoco.mj_forward(self.model, self.data)

    def render(self) -> None:
        """Launch interactive viewer (blocking)."""
        if self._viewer is None:
            self._viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self._viewer.sync()

    def close(self) -> None:
        """Close viewer if open."""
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None
