"""
Pinocchio-based model backend for wheeled UR5e robot.

This module wraps the MJCF model in pinocchio, providing:
  - Exact FK/Jacobian through pinocchio's analytical algorithms
  - EE orientation (full SE(3) placement) for free
  - Foundation for kino-dynamic dynamics (arm ABA)

The pinocchio parser requires a pre-processed MJCF:
  - MuJoCo's compiler expands <include> directives
  - target_body mocap must be stripped (pinocchio parser stops at first body)
  - Joint order differs: [base_x, base_y, base_yaw, base_z, arm...] vs.
    robot_model.py's [base_x, base_y, base_z, base_yaw, arm...]
"""

import sys
import os
from pathlib import Path
from typing import Tuple
import tempfile

import numpy as np

try:
    import pinocchio as pin
except ImportError:
    print("pinocchio not found. Make sure you're in the 'all' pixi environment.", file=sys.stderr)
    raise

try:
    import mujoco
except ImportError:
    print("mujoco not found. Run 'pixi add --pypi mujoco' in the 'all' env.", file=sys.stderr)
    raise


class PinocchioWheeledUR5eModel:
    """
    Pinocchio-based robot model for the wheeled UR5e.

    Loads the MJCF, compiles/flattens it via mujoco, strips the mocap target_body,
    and builds the pinocchio model. Provides FK, Jacobians, and the reduced arm
    model for ABA.

    Joint order in pinocchio:
      [base_x, base_y, base_yaw, base_z, shoulder_pan, shoulder_lift,
       elbow, wrist_1, wrist_2, wrist_3]

    vs. robot_model.py order:
      [base_x, base_y, base_z, base_yaw, shoulder_pan, shoulder_lift,
       elbow, wrist_1, wrist_2, wrist_3]

    Use q_rm_to_pin / q_pin_to_rm to convert between the two conventions.
    All public methods accept/return state in robot_model.py order so the rest
    of the codebase is unaffected by the internal reordering.
    """

    nq: int = 12  # Updated for wheeled_ur5e_wheels.xml: base(4) + wheels(2) + arm(6)
    nu: int = 12  # Updated: base_controls(4) + wheels(2) + arm(6)

    EE_FRAME: str = "ee_site"

    # Base joint names in pinocchio order (used to build the reduced arm model)
    _BASE_JOINTS = ["base_x", "base_y", "base_yaw", "base_z"]
    _WHEEL_JOINTS = ["left_wheel_joint", "right_wheel_joint"]  # Added for wheels model
    _ARM_JOINTS = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]

    def __init__(self, mjcf_path: str | None = None):
        if mjcf_path is None:
            mjcf_path = str(
                Path(__file__).resolve().parents[1] / "assets" / "wheeled_ur5e_wheels.xml"
            )
        self._mjcf_path = mjcf_path

        # Build the pinocchio-loadable flattened MJCF (cached next to the source).
        pin_mjcf = self._make_pinocchio_mjcf(mjcf_path)

        self.model = pin.buildModelFromMJCF(pin_mjcf)
        self.data = self.model.createData()

        if self.model.nq != self.nq:
            raise RuntimeError(
                f"Expected {self.nq} DOF in pinocchio model, got {self.model.nq}. "
                f"Check MJCF preprocessing at {pin_mjcf}."
            )

        self.ee_frame_id = self.model.getFrameId(self.EE_FRAME)

        # Precompute the q-index reordering between conventions.
        # pinocchio order: bx, by, yaw, bz, arm(6)
        # robot_model order: bx, by, bz, yaw, arm(6)
        # rm[i] = pin[self._rm_from_pin[i]]
        self._pin_from_rm = np.array([0, 1, 3, 2, 4, 5, 6, 7, 8, 9])  # pin_idx for each rm_idx
        self._rm_from_pin = np.array([0, 1, 3, 2, 4, 5, 6, 7, 8, 9])  # rm_idx for each pin_idx
        # (the swap base_z<->base_yaw is its own inverse, so both are identical)

        # Build the reduced 6-DOF arm model (base joints locked at nominal).
        self._build_reduced_arm_model()

    # ------------------------------------------------------------------
    # MJCF preprocessing
    # ------------------------------------------------------------------

    @staticmethod
    def _make_pinocchio_mjcf(mjcf_path: str) -> str:
        """
        Compile the MJCF via mujoco (expands <include>), flatten it, and strip
        the target_body mocap so pinocchio's parser reaches the robot tree.

        Returns the path to the generated pinocchio-loadable XML, cached next to
        the source as <name>_pin.xml.
        """
        import re

        src = Path(mjcf_path)
        out_path = src.with_name(src.stem + "_pin.xml")

        # Compile via mujoco to expand includes and resolve defaults.
        m = mujoco.MjModel.from_xml_path(mjcf_path)
        flat_path = tempfile.mktemp(suffix=".xml")
        mujoco.mj_saveLastXML(flat_path, m)

        txt = Path(flat_path).read_text()
        os.unlink(flat_path)

        # Strip the mocap target_body block (parser stops at the first body).
        txt = re.sub(
            r'\s*<body name="target_body".*?</body>', "", txt, flags=re.DOTALL
        )

        out_path.write_text(txt)
        return str(out_path)

    # ------------------------------------------------------------------
    # State reordering helpers
    # ------------------------------------------------------------------

    def q_rm_to_pin(self, q_rm: np.ndarray) -> np.ndarray:
        """Convert state from robot_model.py order to pinocchio order."""
        return np.asarray(q_rm)[self._pin_from_rm]

    def q_pin_to_rm(self, q_pin: np.ndarray) -> np.ndarray:
        """Convert state from pinocchio order to robot_model.py order."""
        return np.asarray(q_pin)[self._rm_from_pin]

    # ------------------------------------------------------------------
    # Reduced arm model (for ABA in kino-dynamic dynamics)
    # ------------------------------------------------------------------

    def _build_reduced_arm_model(self) -> None:
        """
        Build a 6-DOF fixed-base arm model by loading a standalone UR5e MJCF.

        Gravity in the arm-root frame is invariant to base_yaw (rotation about
        world-z) and base_z (translation along world-z), so a single reduced
        model with the shoulder fixed at the origin is dynamically exact for the arm
        under the hybrid assumption (base velocity-controlled, arm torque-controlled).

        Note: buildReducedModel() loses inertia data when locking joints, so we
        use a separate standalone MJCF instead.

        CRITICAL: We add armature (rotor inertia) to the model here so that
        computeABADerivatives() produces correct Jacobians automatically.
        """
        arm_mjcf_path = str(Path(__file__).resolve().parents[1] / "assets" / "ur5e_arm_6dof.xml")

        # Compile and load the standalone arm model
        m_mj = mujoco.MjModel.from_xml_path(arm_mjcf_path)
        flat_arm_path = tempfile.mktemp(suffix=".xml")
        mujoco.mj_saveLastXML(flat_arm_path, m_mj)

        self.arm_model = pin.buildModelFromMJCF(flat_arm_path)
        self.arm_data = self.arm_model.createData()
        os.unlink(flat_arm_path)

        if self.arm_model.nq != 6:
            raise RuntimeError(
                f"Reduced arm model has {self.arm_model.nq} DOF, expected 6."
            )

        # Add armature (rotor inertia) to the model
        # These values match the MuJoCo XML and should be consistent with
        # hybrid_dynamics.py's armature array
        # [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
        armature = np.array([0.1, 0.1, 0.1, 0.01, 0.01, 0.01])

        # Add armature to rotor inertia for each joint
        # Pinocchio stores rotor inertia in model.rotorInertia (size = njoints including universe)
        # For a 6-DOF arm, njoints = 7 (including universe joint at index 0)
        # We need to set rotorInertia[1:7] for the 6 actual joints
        if self.arm_model.njoints != 7:
            print(f"Warning: Expected 7 joints (including universe), got {self.arm_model.njoints}")

        for joint_idx in range(6):  # 0-5 for the 6 arm joints
            pinocchio_joint_idx = joint_idx + 1  # Skip universe joint
            if pinocchio_joint_idx < len(self.arm_model.rotorInertia):
                self.arm_model.rotorInertia[pinocchio_joint_idx] = armature[joint_idx]
                self.arm_model.rotorGearRatio[pinocchio_joint_idx] = 1.0

    # ------------------------------------------------------------------
    # Forward kinematics
    # ------------------------------------------------------------------

    def fk_pose(self, q_rm: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Forward kinematics of the EE frame.

        Input:  q in robot_model.py order, shape (10,)
        Output: (p, R) world position (3,) and rotation matrix (3,3)
        """
        q_pin = self.q_rm_to_pin(q_rm)
        pin.forwardKinematics(self.model, self.data, q_pin)
        pin.updateFramePlacements(self.model, self.data)
        oMf = self.data.oMf[self.ee_frame_id]
        return np.array(oMf.translation), np.array(oMf.rotation)

    def fk_numpy(self, q_rm: np.ndarray) -> np.ndarray:
        """EE world position only (drop-in for robot_model.fk_numpy)."""
        p, _ = self.fk_pose(q_rm)
        return p

    def frame_jacobian(self, q_rm: np.ndarray, local: bool = False) -> np.ndarray:
        """
        Geometric Jacobian of the EE frame, shape (6, 10), in robot_model.py
        column order. Rows: [linear (3); angular (3)].

        local=False -> LOCAL_WORLD_ALIGNED (world-aligned axes at the frame)
        local=True  -> LOCAL frame
        """
        q_pin = self.q_rm_to_pin(q_rm)
        pin.computeJointJacobians(self.model, self.data, q_pin)
        pin.updateFramePlacements(self.model, self.data)
        ref = pin.LOCAL if local else pin.LOCAL_WORLD_ALIGNED
        J_pin = pin.getFrameJacobian(self.model, self.data, self.ee_frame_id, ref)
        # Reorder columns from pinocchio order back to robot_model order.
        # J_rm[:, i] = J_pin[:, pin_idx_of_rm_i]
        J_rm = J_pin[:, self._pin_from_rm]
        return np.array(J_rm)

    def position_jacobian(self, q_rm: np.ndarray) -> np.ndarray:
        """Linear (position) part of the EE Jacobian, shape (3, 10)."""
        return self.frame_jacobian(q_rm)[:3, :]

