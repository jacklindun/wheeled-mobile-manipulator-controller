"""Low-level controller: converts MPC velocity commands to position targets."""

import numpy as np
from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel


def wrap_to_pi(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return float((angle + np.pi) % (2 * np.pi) - np.pi)


class LowLevelController:
    """
    Integrates MPC velocity output u0 into a desired joint position q_des.

    The position is then sent to MuJoCo position actuators.
    """

    def __init__(self, robot: WheeledUR5eModel, dt: float):
        self.robot = robot
        self.dt = dt

    def compute_q_des(self, q_current: np.ndarray, u0: np.ndarray) -> np.ndarray:
        """
        Integrate q_current by u0 over dt, clip to joint limits, wrap yaw.

        Parameters
        ----------
        q_current : (10,) current joint state
        u0        : (10,) velocity command from MPC

        Returns
        -------
        q_des : (10,) desired joint position for position actuator
        """
        q_des = self.robot.dynamics_numpy(q_current, u0, self.dt)
        q_des = np.clip(q_des, self.robot.q_min, self.robot.q_max)
        q_des[3] = wrap_to_pi(q_des[3])
        return q_des
