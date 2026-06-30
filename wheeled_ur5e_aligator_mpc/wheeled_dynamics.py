"""
Phase 5: 轮子动力学模型

差速驱动移动底盘的动力学模型，包含：
- 轮子动力学（电机扭矩 → 轮速）
- 基座运动学（轮速 → 基座速度）
- 非完整约束（无侧向滑动）

状态空间：
  x = [q_base(4), θ_wheels(2), q_arm(6),    # 位置 (12-dim)
       v_base(3), ω_wheels(2), v_arm(6)]    # 速度 (11-dim)
  = 23-dim

控制空间：
  u = [τ_left, τ_right, τ_arm(6)] = 8-dim

约束：
  - 非完整约束：vy_body = 0（不能侧滑）
  - 差速驱动运动学：v, ω = f(ω_left, ω_right)
"""

import numpy as np
try:
    import aligator
    import pinocchio as pin
except ImportError:
    import sys
    from pathlib import Path
    _repo_root = Path(__file__).resolve().parents[3]
    sys.path[:0] = [str(_repo_root / "build" / "bindings" / "python")]
    import aligator
    import pinocchio as pin


class WheelParameters:
    """轮子参数配置"""
    def __init__(self):
        self.radius = 0.1        # m, 轮子半径
        self.wheelbase = 0.5     # m, 轮间距
        self.mass = 2.0          # kg, 单个轮子质量
        self.inertia = 0.01      # kg·m², 轮子转动惯量
        self.friction = 0.5      # N·m·s/rad, 摩擦系数
        self.gear_ratio = 50.0   # 减速比


class WheeledUR5eDynamics(aligator.dynamics.ExplicitDynamicsModel):
    """
    带轮子动力学的移动机械臂

    状态：x = [q_base(4), θ_wheels(2), q_arm(6), v_base(3), ω_wheels(2), v_arm(6)]
    控制：u = [τ_left, τ_right, τ_arm(6)]

    动力学：
      1. 轮子动力学：I_wheel * α = τ_motor - τ_friction - F_ground * r
      2. 基座运动学：从轮速计算基座速度（差速驱动）
      3. 机械臂动力学：ABA（同Phase 4）
    """

    def __init__(self, pin_robot, dt, wheel_params=None):
        """
        Parameters
        ----------
        pin_robot : PinocchioWheeledUR5eModel
            Pinocchio模型（用于机械臂动力学）
        dt : float
            积分步长
        wheel_params : WheelParameters, optional
            轮子参数
        """
        nx = 23  # 状态维度
        nu = 8   # 控制维度
        space = aligator.manifolds.VectorSpace(nx)
        super().__init__(space, nu)

        self._pin_robot = pin_robot
        self._dt = dt

        # 轮子参数
        if wheel_params is None:
            wheel_params = WheelParameters()
        self._wheel = wheel_params

        # 机械臂参数（从Phase 4复用）
        self._arm_model = pin_robot.arm_model
        self._arm_data = pin_robot.arm_data
        self._arm_damping = np.array([1.0, 1.0, 0.5, 0.1, 0.1, 0.1])
        self._arm_armature = np.array([0.1, 0.1, 0.1, 0.01, 0.01, 0.01])

    def __reduce__(self):
        """支持deepcopy"""
        return (
            self.__class__,
            (self._pin_robot, self._dt, self._wheel),
        )

    def forward(self, x, u, data) -> None:
        """
        前向动力学：x_next = f(x, u)

        积分方案：semi-implicit Euler
        """
        # 提取状态
        q_base = np.asarray(x[0:4])      # [x, y, z, yaw]
        θ_wheels = np.asarray(x[4:6])    # [θ_left, θ_right]
        q_arm = np.asarray(x[6:12])      # 机械臂位置
        v_base = np.asarray(x[12:15])    # [vx_world, vy_world, ω_yaw]
        ω_wheels = np.asarray(x[15:17])  # [ω_left, ω_right]
        v_arm = np.asarray(x[17:23])     # 机械臂速度

        # 提取控制
        τ_left = u[0]
        τ_right = u[1]
        τ_arm = np.asarray(u[2:8])

        dt = self._dt

        # ============================================================
        # 1. 轮子动力学
        # ============================================================
        # 简化模型：忽略轮地接触力反馈（假设无滑动）
        # I * α = τ - b * ω
        α_left = (τ_left - self._wheel.friction * ω_wheels[0]) / self._wheel.inertia
        α_right = (τ_right - self._wheel.friction * ω_wheels[1]) / self._wheel.inertia

        # 积分轮速
        ω_wheels_next = ω_wheels + dt * np.array([α_left, α_right])
        θ_wheels_next = θ_wheels + dt * ω_wheels_next

        # ============================================================
        # 2. 基座运动学（差速驱动）
        # ============================================================
        # 从轮速计算基座速度
        v_linear, ω_angular = self._diff_drive_kinematics(ω_wheels_next)

        # 基座位置积分（world frame）
        yaw = q_base[3]
        q_base_next = self._integrate_base_position(q_base, v_linear, ω_angular, dt)

        # 基座速度（world frame）
        vx_world = v_linear * np.cos(yaw)
        vy_world = v_linear * np.sin(yaw)
        v_base_next = np.array([vx_world, vy_world, ω_angular])

        # ============================================================
        # 3. 机械臂动力学（ABA，同Phase 4）
        # ============================================================
        a_arm = self._compute_arm_acceleration(q_arm, v_arm, τ_arm)

        # 积分（semi-implicit Euler）
        v_arm_next = v_arm + dt * a_arm
        q_arm_next = q_arm + dt * v_arm_next

        # ============================================================
        # 组装下一状态
        # ============================================================
        data.xnext = np.concatenate([
            q_base_next,      # 4
            θ_wheels_next,    # 2
            q_arm_next,       # 6
            v_base_next,      # 3
            ω_wheels_next,    # 2
            v_arm_next,       # 6
        ])

    def _diff_drive_kinematics(self, ω_wheels):
        """
        差速驱动正向运动学：轮速 → 基座速度

        Parameters
        ----------
        ω_wheels : (2,) array
            [ω_left, ω_right] rad/s

        Returns
        -------
        v_linear : float
            线速度 (m/s)
        ω_angular : float
            角速度 (rad/s)
        """
        r = self._wheel.radius
        L = self._wheel.wheelbase

        v_linear = r * (ω_wheels[0] + ω_wheels[1]) / 2.0
        ω_angular = r * (ω_wheels[1] - ω_wheels[0]) / L

        return v_linear, ω_angular

    def _integrate_base_position(self, q_base, v_linear, ω_angular, dt):
        """
        积分基座位置

        Parameters
        ----------
        q_base : (4,) array
            [x, y, z, yaw]
        v_linear : float
            线速度
        ω_angular : float
            角速度
        dt : float
            时间步长

        Returns
        -------
        q_base_next : (4,) array
        """
        x, y, z, yaw = q_base

        # 积分（Euler）
        x_next = x + dt * v_linear * np.cos(yaw)
        y_next = y + dt * v_linear * np.sin(yaw)
        z_next = z  # 假设平地，z不变
        yaw_next = yaw + dt * ω_angular

        # Wrap yaw到[-π, π]
        yaw_next = np.arctan2(np.sin(yaw_next), np.cos(yaw_next))

        return np.array([x_next, y_next, z_next, yaw_next])

    def _compute_arm_acceleration(self, q_arm, v_arm, τ_arm):
        """
        计算机械臂加速度（ABA with armature）

        同Phase 4的实现
        """
        # 应用阻尼
        τ_damped = τ_arm - self._arm_damping * v_arm

        # ABA（无armature）
        a_no_armature = pin.aba(self._arm_model, self._arm_data, q_arm, v_arm, τ_damped)

        # Armature校正
        pin.crba(self._arm_model, self._arm_data, q_arm)
        M = self._arm_data.M
        M_eff = M + np.diag(self._arm_armature)

        rhs = M @ a_no_armature
        a_arm = np.linalg.solve(M_eff, rhs)

        return a_arm

    def dForward(self, x, u, data) -> None:
        """
        计算雅可比矩阵 Jx (23×23), Ju (23×8)

        简化实现：使用有限差分
        （完整解析雅可比需要大量推导）
        """
        # TODO: 使用有限差分或解析推导
        # 当前先填充单位矩阵占位
        data.Jx[:] = np.eye(23)  # 占位
        data.Ju[:] = 0.0          # 占位

        # 注意：这会导致梯度不准确，影响MPC性能
        # Phase 6的MPC+WBC架构会避免这个问题


class NonholonomicConstraint:
    """
    非完整约束：vy_body = 0

    差速驱动机器人不能侧向移动（在body frame中，y方向速度必须为0）
    """

    @staticmethod
    def compute_residual(x):
        """
        计算约束残差：r = vy_body(x)

        Parameters
        ----------
        x : (23,) array
            状态

        Returns
        -------
        residual : float
            约束残差（应该为0）
        """
        v_base = x[12:15]  # [vx_world, vy_world, ω_yaw]
        yaw = x[3]         # 基座yaw角

        vx_world, vy_world = v_base[0], v_base[1]

        # 转换到body frame
        vx_body = vx_world * np.cos(yaw) + vy_world * np.sin(yaw)
        vy_body = -vx_world * np.sin(yaw) + vy_world * np.cos(yaw)

        return vy_body

    @staticmethod
    def compute_jacobian(x):
        """
        计算约束雅可比：dr/dx

        Returns
        -------
        jacobian : (23,) array
        """
        yaw = x[3]

        jac = np.zeros(23)

        # d(vy_body)/d(yaw)
        vx_world, vy_world = x[12], x[13]
        jac[3] = -vx_world * np.cos(yaw) - vy_world * np.sin(yaw)

        # d(vy_body)/d(vx_world)
        jac[12] = -np.sin(yaw)

        # d(vy_body)/d(vy_world)
        jac[13] = np.cos(yaw)

        return jac


def inverse_diff_drive(v_linear, ω_angular, wheel_params):
    """
    差速驱动逆向运动学：基座速度 → 轮速

    给定期望的线速度和角速度，计算需要的轮速

    Parameters
    ----------
    v_linear : float
        期望线速度 (m/s)
    ω_angular : float
        期望角速度 (rad/s)
    wheel_params : WheelParameters
        轮子参数

    Returns
    -------
    ω_left : float
        左轮速度 (rad/s)
    ω_right : float
        右轮速度 (rad/s)
    """
    r = wheel_params.radius
    L = wheel_params.wheelbase

    ω_left = (v_linear - L/2 * ω_angular) / r
    ω_right = (v_linear + L/2 * ω_angular) / r

    return ω_left, ω_right
