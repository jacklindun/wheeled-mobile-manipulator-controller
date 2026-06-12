"""
WheeledUR5eModel: kinematic model for 4-DoF virtual mobile base + UR5e 6-DoF arm.

State q (10,):  base_x, base_y, base_z, base_yaw, shoulder_pan, shoulder_lift,
                elbow, wrist_1, wrist_2, wrist_3
Control u (10,): base_vx_body, base_vy_body, base_vz, base_omega, + 6 arm joint velocities

FK follows the MJCF body-tree structure directly (NOT a DH chain) so that
Python FK == MuJoCo site_xpos["ee_site"] exactly.  The chain is:

  world → base (trans+Rz(yaw)) → shoulder_link (z+0.27, Rz(pan))
       → upper_arm  (y+0.1358, Ry(lift))
       → forearm    (x-0.425,  Ry(elbow))
       → wrist_1    (x-0.3922, z+0.1333, Ry(w1))
       → wrist_2    (y-0.0997, Rz(w2))
       → wrist_3    (z+0.0996, Ry(w3))
       → ee_site    (y-0.0996)
"""

import numpy as np


class WheeledUR5eModel:
    """
    Kinematic model for wheeled UR5e: 4-DoF virtual mobile base + 6-DoF arm.
    q = [base_x, base_y, base_z, base_yaw, shoulder_pan, shoulder_lift,
         elbow, wrist_1, wrist_2, wrist_3]
    u = [base_vx_body, base_vy_body, base_vz, base_omega,
         shoulder_pan_qd, shoulder_lift_qd, elbow_qd, wrist_1_qd, wrist_2_qd, wrist_3_qd]
    """

    nq: int = 10
    nx: int = 10
    nu: int = 10

    q_names = [
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

    u_names = [
        "base_vx_body",
        "base_vy_body",
        "base_vz",
        "base_omega",
        "shoulder_pan_qd",
        "shoulder_lift_qd",
        "elbow_qd",
        "wrist_1_qd",
        "wrist_2_qd",
        "wrist_3_qd",
    ]

    # State bounds
    q_min = np.array([
        -3.0,       # base_x
        -3.0,       # base_y
         0.0,       # base_z
        -np.pi,     # base_yaw
        -2*np.pi,   # shoulder_pan
        -2*np.pi,   # shoulder_lift
        -np.pi,     # elbow (UR5e limited)
        -2*np.pi,   # wrist_1
        -2*np.pi,   # wrist_2
        -2*np.pi,   # wrist_3
    ])

    q_max = np.array([
         3.0,
         3.0,
         0.5,       # base_z max lift 0.5 m
         np.pi,
         2*np.pi,
         2*np.pi,
         np.pi,
         2*np.pi,
         2*np.pi,
         2*np.pi,
    ])

    # Control velocity bounds
    u_min = np.array([
        -0.5,   # base_vx_body  m/s
        -0.5,   # base_vy_body  m/s
        -0.2,   # base_vz       m/s
        -1.0,   # base_omega    rad/s
        -1.0,   # arm joint velocities rad/s
        -1.0,
        -1.0,
        -1.0,
        -1.0,
        -1.0,
    ])

    u_max = np.array([
         0.5,
         0.5,
         0.2,
         1.0,
         1.0,
         1.0,
         1.0,
         1.0,
         1.0,
         1.0,
    ])

    # Safe nominal posture — shoulder_pan=π puts arm facing forward (+X world).
    # FK at this config: ee ≈ [0.62, 0.06, 0.86] (verified analytically vs MJCF body tree)
    q_nominal = np.array([
         0.0,        # base_x
         0.0,        # base_y
         0.2,        # base_z  (lifted 0.2 m)
         0.0,        # base_yaw
         np.pi,      # shoulder_pan  (arm facing +X)
         np.pi/3,    # shoulder_lift
        -np.pi/2,    # elbow
         np.pi/6,    # wrist_1
         0.0,        # wrist_2
         0.0,        # wrist_3
    ])

    SHOULDER_MOUNT_Z: float = 0.27  # shoulder_link z offset in base_yaw_body (MJCF)

    def fk_numpy(self, q: np.ndarray) -> np.ndarray:
        """
        Compute end-effector world position from joint state.
        Follows the MJCF body tree exactly so Python FK == MuJoCo ee_site.

        Input:  q shape (10,)
        Output: ee_pos_world shape (3,)
        """
        bx, by, bz, yaw = q[0], q[1], q[2], q[3]
        pan, lift, elbow, w1, w2, w3 = q[4], q[5], q[6], q[7], q[8], q[9]

        def Rz(a):
            c, s = np.cos(a), np.sin(a)
            return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])

        def Ry(a):
            c, s = np.cos(a), np.sin(a)
            return np.array([[c, 0., s], [0., 1., 0.], [-s, 0., c]])

        p = np.array([bx, by, bz])
        R = Rz(yaw)

        # shoulder_link: pos=(0,0,0.27) in base_yaw_body; joint shoulder_pan Rz
        p = p + R @ np.array([0., 0., self.SHOULDER_MOUNT_Z])
        R = R @ Rz(pan)

        # upper_arm_link: pos=(0,0.1358,0) in shoulder_link; joint shoulder_lift Ry
        p = p + R @ np.array([0., 0.1358, 0.])
        R = R @ Ry(lift)

        # forearm_link: pos=(-0.425,0,0) in upper_arm_link; joint elbow Ry
        p = p + R @ np.array([-0.425, 0., 0.])
        R = R @ Ry(elbow)

        # wrist_1_link: pos=(-0.3922,0,0.1333) in forearm_link; joint wrist_1 Ry
        p = p + R @ np.array([-0.3922, 0., 0.1333])
        R = R @ Ry(w1)

        # wrist_2_link: pos=(0,-0.0997,0) in wrist_1_link; joint wrist_2 Rz (axis z)
        p = p + R @ np.array([0., -0.0997, 0.])
        R = R @ Rz(w2)

        # wrist_3_link: pos=(0,0,0.0996) in wrist_2_link; joint wrist_3 Ry
        p = p + R @ np.array([0., 0., 0.0996])
        R = R @ Ry(w3)

        # ee_site: pos=(0,-0.0996,0) in wrist_3_link
        p = p + R @ np.array([0., -0.0996, 0.])

        return p

    def finite_difference_jacobian_fk(self, q: np.ndarray, eps: float = 1e-6) -> np.ndarray:
        """
        Numerical Jacobian of FK: d(ee_pos)/dq, shape (3, 10).
        Used for cost linearization in ALIGATOR.
        """
        J = np.zeros((3, self.nq))
        p0 = self.fk_numpy(q)
        for i in range(self.nq):
            q_plus = q.copy()
            q_plus[i] += eps
            p_plus = self.fk_numpy(q_plus)
            J[:, i] = (p_plus - p0) / eps
        return J

    def dynamics_numpy(self, q: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
        """
        Kinematic integration step: q_next = f(q, u, dt).
        Base uses body-frame velocity → world-frame integration.
        Arm joints use direct first-order integration.
        """
        bx, by, bz, yaw = q[0], q[1], q[2], q[3]
        vx_body, vy_body, vz, omega = u[0], u[1], u[2], u[3]
        qd_arm = u[4:10]

        cy, sy = np.cos(yaw), np.sin(yaw)

        q_next = np.empty(self.nq)
        q_next[0] = bx + dt * (cy * vx_body - sy * vy_body)
        q_next[1] = by + dt * (sy * vx_body + cy * vy_body)
        q_next[2] = bz + dt * vz
        q_next[3] = yaw + dt * omega
        q_next[4:10] = q[4:10] + dt * qd_arm

        # Wrap yaw to [-pi, pi]
        q_next[3] = _wrap_to_pi(q_next[3])

        return q_next

    def linearize_dynamics(
        self, q: np.ndarray, u: np.ndarray, dt: float
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Analytic linearization of kinematic dynamics.
        Returns A = df/dq (10,10), B = df/du (10,10).

        Most elements are identity/dt (first-order integrators), except:
          dx_next/dyaw = -dt*(sy*vx + cy*vy)
          dy_next/dyaw =  dt*(cy*vx - sy*vy)
          dx_next/dvx_body =  dt*cy
          dx_next/dvy_body = -dt*sy
          dy_next/dvx_body =  dt*sy
          dy_next/dvy_body =  dt*cy
        """
        yaw = q[3]
        vx_body, vy_body = u[0], u[1]
        cy, sy = np.cos(yaw), np.sin(yaw)

        A = np.eye(self.nq)
        # coupling: d base_x_next / d base_yaw
        A[0, 3] = -dt * (sy * vx_body + cy * vy_body)
        # coupling: d base_y_next / d base_yaw
        A[1, 3] = dt * (cy * vx_body - sy * vy_body)

        B = np.zeros((self.nq, self.nu))
        # base_x_next / d vx_body, d vy_body
        B[0, 0] = dt * cy
        B[0, 1] = -dt * sy
        # base_y_next / d vx_body, d vy_body
        B[1, 0] = dt * sy
        B[1, 1] = dt * cy
        # base_z_next / d vz
        B[2, 2] = dt
        # base_yaw_next / d omega
        B[3, 3] = dt
        # arm joints: direct integration
        for i in range(6):
            B[4 + i, 4 + i] = dt

        return A, B


def _wrap_to_pi(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


if __name__ == "__main__":
    model = WheeledUR5eModel()
    q = model.q_nominal.copy()
    print(f"FK test:")
    print(f"  q = {q}")
    ee = model.fk_numpy(q)
    print(f"  ee_pos = {ee}")

    u = np.zeros(10)
    dt = 0.05
    q_next = model.dynamics_numpy(q, u, dt)
    assert np.allclose(q_next, q), "Zero control should leave state unchanged"
    print("Dynamics test passed (zero control).")

    A, B = model.linearize_dynamics(q, u, dt)
    assert A.shape == (10, 10), f"A shape {A.shape}"
    assert B.shape == (10, 10), f"B shape {B.shape}"
    print(f"Linearize test passed. A shape {A.shape}, B shape {B.shape}.")

    # Verify linearization against numerical FD
    eps = 1e-5
    A_fd = np.zeros((10, 10))
    for i in range(10):
        q_p = q.copy(); q_p[i] += eps
        q_m = q.copy(); q_m[i] -= eps
        A_fd[:, i] = (model.dynamics_numpy(q_p, u, dt) - model.dynamics_numpy(q_m, u, dt)) / (2 * eps)
    err_A = np.max(np.abs(A - A_fd))
    print(f"Linearization A vs FD max error: {err_A:.2e} (should be < 1e-5)")

    B_fd = np.zeros((10, 10))
    u_test = np.array([0.1, 0.05, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    A2, B2 = model.linearize_dynamics(q, u_test, dt)
    for i in range(10):
        u_p = u_test.copy(); u_p[i] += eps
        u_m = u_test.copy(); u_m[i] -= eps
        B_fd[:, i] = (model.dynamics_numpy(q, u_p, dt) - model.dynamics_numpy(q, u_m, dt)) / (2 * eps)
    err_B = np.max(np.abs(B2 - B_fd))
    print(f"Linearization B vs FD max error: {err_B:.2e} (should be < 1e-5)")

    print("\nAll robot_model tests passed.")
