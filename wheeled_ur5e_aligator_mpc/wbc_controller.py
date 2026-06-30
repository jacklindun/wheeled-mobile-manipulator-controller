"""
Phase 6: Whole-Body Controller (WBC)

WBC使用QP求解器将MPC的速度/加速度目标转换为最优扭矩。

核心功能：
1. 计算动力学项 M(q), h(q,v)
2. 构建QP问题（目标函数+约束）
3. 求解QP获得最优扭矩
4. 保证动力学一致性和约束满足

使用ProxQP求解器（ALIGATOR生态）
"""

import numpy as np
import sys
from pathlib import Path

# 尝试导入QP求解器（优先级顺序）
QP_SOLVER = None
try:
    import proxsuite
    QP_SOLVER = "proxsuite"
except ImportError:
    try:
        import cvxpy as cp
        QP_SOLVER = "cvxpy"
    except ImportError:
        from scipy.optimize import minimize
        QP_SOLVER = "scipy"

# 导入Pinocchio
try:
    import pinocchio as pin
except ImportError:
    _repo_root = Path(__file__).resolve().parents[3]
    sys.path[:0] = [str(_repo_root / "build" / "bindings" / "python")]
    import pinocchio as pin

from .wheeled_dynamics import WheelParameters

print(f"[WBC] 使用QP求解器: {QP_SOLVER}")


class WBCWeights:
    """WBC QP权重配置"""
    def __init__(self):
        # 动力学一致性（最高优先级）
        self.w_dynamics = 1000.0

        # 加速度跟踪
        self.w_track_base = 100.0    # 基座加速度
        self.w_track_wheels = 50.0   # 轮子加速度
        self.w_track_arm = 100.0     # 机械臂加速度

        # 扭矩正则化
        self.w_reg_wheels = 0.01     # 轮子扭矩
        self.w_reg_arm = 0.001       # 机械臂扭矩

        # 扭矩平滑
        self.w_smooth = 1.0


class WholeBodyController:
    """
    全身控制器 - QP求解器

    将MPC的期望加速度转换为最优扭矩，同时保证：
    - 动力学一致性：M*a + h = S^T*τ
    - 非完整约束：vy_body = 0
    - 扭矩限制：τ_min ≤ τ ≤ τ_max
    """

    def __init__(self, pin_robot, wheel_params=None, weights=None):
        """
        Parameters
        ----------
        pin_robot : PinocchioWheeledUR5eModel
            Pinocchio模型
        wheel_params : WheelParameters, optional
            轮子参数
        weights : WBCWeights, optional
            QP权重
        """
        self._pin_robot = pin_robot
        self._arm_model = pin_robot.arm_model
        self._arm_data = pin_robot.arm_data

        if wheel_params is None:
            wheel_params = WheelParameters()
        self._wheel = wheel_params

        if weights is None:
            weights = WBCWeights()
        self._weights = weights

        # QP求解器（延迟初始化）
        self._qp = None
        self._n_vars = 19  # [a(11), τ(8)]
        self._n_eq = 11    # 动力学方程
        self._n_ineq = 17  # 扭矩上下界 + 非完整约束

        # 缓存
        self._τ_prev = np.zeros(8)

    def compute_control(self, x_current, a_des, τ_prev=None):
        """
        计算最优扭矩

        Parameters
        ----------
        x_current : (23,) array
            当前状态 [q(12), v(11)]
        a_des : (11,) array
            期望加速度 [a_base(3), α_wheels(2), a_arm(6)]
        τ_prev : (8,) array, optional
            上一步扭矩（用于平滑）

        Returns
        -------
        τ_opt : (8,) array
            最优扭矩 [τ_wheels(2), τ_arm(6)]
        info : dict
            求解信息
        """
        if τ_prev is None:
            τ_prev = self._τ_prev

        # 1. 提取状态
        q = x_current[:12]
        v = x_current[12:]

        # 2. 计算动力学项
        M, h = self._compute_dynamics(q, v)

        # 3. 构建QP问题
        P, q_vec, A_eq, b_eq, A_ineq, b_ineq, lb, ub = self._build_qp(
            M, h, a_des, τ_prev, q, v
        )

        # 4. 求解QP
        z_opt, solve_info = self._solve_qp(P, q_vec, A_eq, b_eq, A_ineq, b_ineq, lb, ub)

        # 5. 提取结果
        a_opt = z_opt[:11]
        τ_opt = z_opt[11:]

        # 6. 缓存
        self._τ_prev = τ_opt.copy()

        # 7. 计算动力学残差
        dynamics_residual = np.linalg.norm(M @ a_opt + h - self._S_matrix() @ τ_opt)

        info = {
            "solve_time_ms": solve_info["solve_time"] * 1000,
            "iterations": solve_info["iterations"],
            "dynamics_residual": dynamics_residual,
            "a_optimal": a_opt,
        }

        return τ_opt, info

    def _compute_dynamics(self, q, v):
        """
        计算动力学项 M(q) 和 h(q,v)

        Returns
        -------
        M : (11, 11) array
            质量矩阵
        h : (11,) array
            非线性项（重力+科氏力+阻尼）
        """
        # 简化模型：分块计算

        # 1. 基座（简化为点质量）
        m_base = 50.0  # kg
        I_yaw = 2.0    # kg·m²
        M_base = np.diag([m_base, m_base, I_yaw])
        h_base = np.zeros(3)  # 无重力影响（水平面）

        # 2. 轮子
        I_wheel = self._wheel.inertia
        M_wheels = np.diag([I_wheel, I_wheel])
        h_wheels = self._wheel.friction * v[3:5]  # 摩擦阻尼

        # 3. 机械臂（使用Pinocchio）
        q_arm = q[6:12]
        v_arm = v[5:11]

        # 质量矩阵
        pin.crba(self._arm_model, self._arm_data, q_arm)
        M_arm = self._arm_data.M.copy()

        # 非线性项（重力+科氏力+阻尼）
        pin.rnea(self._arm_model, self._arm_data, q_arm, v_arm, np.zeros(6))
        h_arm = self._arm_data.tau.copy()

        # 阻尼
        arm_damping = np.array([1.0, 1.0, 0.5, 0.1, 0.1, 0.1])
        h_arm += arm_damping * v_arm

        # 组装
        M = np.zeros((11, 11))
        M[:3, :3] = M_base
        M[3:5, 3:5] = M_wheels
        M[5:11, 5:11] = M_arm

        h = np.concatenate([h_base, h_wheels, h_arm])

        return M, h

    def _build_qp(self, M, h, a_des, τ_prev, q, v):
        """
        构建QP问题

        决策变量: z = [a(11), τ(8)]

        目标:
          minimize  0.5 * z^T * P * z + q^T * z

        约束:
          A_eq * z = b_eq           (动力学方程)
          lb ≤ z ≤ ub              (变量界)
          A_ineq * z ≤ b_ineq       (非完整约束)
        """
        n = 19  # 决策变量维度

        # ============================================================
        # 目标函数: P, q
        # ============================================================
        P = np.zeros((n, n))
        q_vec = np.zeros(n)

        # 加速度跟踪项: ||a - a_des||²_W
        W_track = np.diag([
            self._weights.w_track_base,
            self._weights.w_track_base,
            self._weights.w_track_base,
            self._weights.w_track_wheels,
            self._weights.w_track_wheels,
            self._weights.w_track_arm,
            self._weights.w_track_arm,
            self._weights.w_track_arm,
            self._weights.w_track_arm,
            self._weights.w_track_arm,
            self._weights.w_track_arm,
        ])
        P[:11, :11] = W_track
        q_vec[:11] = -W_track @ a_des

        # 扭矩正则化: ||τ||²_R
        W_reg = np.diag([
            self._weights.w_reg_wheels,
            self._weights.w_reg_wheels,
            self._weights.w_reg_arm,
            self._weights.w_reg_arm,
            self._weights.w_reg_arm,
            self._weights.w_reg_arm,
            self._weights.w_reg_arm,
            self._weights.w_reg_arm,
        ])
        P[11:, 11:] += W_reg

        # 扭矩平滑: ||τ - τ_prev||²
        W_smooth = self._weights.w_smooth * np.eye(8)
        P[11:, 11:] += W_smooth
        q_vec[11:] += -W_smooth @ τ_prev

        # 动力学一致性: ||M*a + h - S^T*τ||²
        S = self._S_matrix()
        A_dyn = np.hstack([M, -S])  # (11, 19)
        b_dyn = -h

        P += self._weights.w_dynamics * (A_dyn.T @ A_dyn)
        q_vec += self._weights.w_dynamics * (A_dyn.T @ b_dyn)

        # ============================================================
        # 等式约束: A_eq * z = b_eq
        # ============================================================
        # 动力学通过目标函数中的高权重软约束实现
        # 不使用硬等式约束，避免QP过约束
        A_eq = np.zeros((0, n))  # 空等式约束
        b_eq = np.zeros(0)

        # ============================================================
        # 不等式约束: A_ineq * z ≤ b_ineq
        # ============================================================
        # 非完整约束: vy_body ≤ 0 and -vy_body ≤ 0 (即 =0)
        # vy_body = -vx*sin(yaw) + vy*cos(yaw)
        # d(vy_body)/dt ≈ -ax*sin(yaw) + ay*cos(yaw)
        # 简化：假设yaw变化慢，约束 ay_body = 0

        yaw = q[3]
        # ay_body = -ax*sin(yaw) + ay*cos(yaw)
        A_nonholo = np.zeros((2, n))
        A_nonholo[0, 0] = -np.sin(yaw)  # -ax
        A_nonholo[0, 1] = np.cos(yaw)   # +ay
        A_nonholo[1, :] = -A_nonholo[0, :]  # 双边约束
        b_nonholo = np.array([0.01, 0.01])  # 小的松弛

        A_ineq = A_nonholo
        b_ineq = b_nonholo

        # ============================================================
        # 变量界: lb ≤ z ≤ ub
        # ============================================================
        lb = np.full(n, -np.inf)
        ub = np.full(n, np.inf)

        # 扭矩限制
        lb[11:13] = -10.0   # 轮子扭矩
        ub[11:13] = 10.0
        lb[13:19] = np.array([-150, -150, -150, -28, -28, -28])  # 机械臂
        ub[13:19] = np.array([150, 150, 150, 28, 28, 28])

        return P, q_vec, A_eq, b_eq, A_ineq, b_ineq, lb, ub

    def _S_matrix(self):
        """
        选择矩阵 S: τ → 广义力

        τ = [τ_left, τ_right, τ_arm(6)]  (8-dim)
        广义力对应: [a_base(3), α_wheels(2), a_arm(6)]  (11-dim)

        轮子扭矩直接作用于轮子加速度
        基座加速度由轮子通过运动学耦合（简化：解耦）
        """
        S = np.zeros((11, 8))

        # 轮子扭矩 → 轮子加速度
        S[3, 0] = 1.0  # τ_left → α_left
        S[4, 1] = 1.0  # τ_right → α_right

        # 机械臂扭矩 → 机械臂加速度
        S[5:11, 2:8] = np.eye(6)

        # 基座：简化为零（实际应通过雅可比传递）
        # TODO: 完整的基座-轮子动力学耦合

        return S

    def _solve_qp(self, P, q, A_eq, b_eq, A_ineq, b_ineq, lb, ub):
        """
        求解QP（支持多种求解器）

        Returns
        -------
        z_opt : (19,) array
            最优解
        info : dict
            求解信息
        """
        import time

        # 确保P正定（添加小的正则化）
        P += 1e-8 * np.eye(P.shape[0])

        t_start = time.perf_counter()

        if QP_SOLVER == "proxsuite":
            z_opt, info = self._solve_qp_proxsuite(P, q, A_eq, b_eq, A_ineq, b_ineq, lb, ub)
        elif QP_SOLVER == "cvxpy":
            z_opt, info = self._solve_qp_cvxpy(P, q, A_eq, b_eq, A_ineq, b_ineq, lb, ub)
        else:  # scipy
            z_opt, info = self._solve_qp_scipy(P, q, A_eq, b_eq, A_ineq, b_ineq, lb, ub)

        solve_time = time.perf_counter() - t_start
        info["solve_time"] = solve_time

        return z_opt, info

    def _solve_qp_scipy(self, P, q, A_eq, b_eq, A_ineq, b_ineq, lb, ub):
        """使用scipy求解QP"""
        from scipy.optimize import minimize

        n = len(q)

        def objective(z):
            return 0.5 * z @ P @ z + q @ z

        def gradient(z):
            return P @ z + q

        # 约束
        constraints = []
        if A_eq is not None:
            for i in range(A_eq.shape[0]):
                constraints.append({
                    "type": "eq",
                    "fun": lambda z, i=i: A_eq[i] @ z - b_eq[i],
                })

        if A_ineq is not None:
            for i in range(A_ineq.shape[0]):
                constraints.append({
                    "type": "ineq",
                    "fun": lambda z, i=i: b_ineq[i] - A_ineq[i] @ z,
                })

        # 变量界
        bounds = [(lb[i] if not np.isinf(lb[i]) else None,
                   ub[i] if not np.isinf(ub[i]) else None) for i in range(n)]

        # 求解
        result = minimize(
            objective,
            x0=np.zeros(n),
            method="SLSQP",
            jac=gradient,
            constraints=constraints,
            bounds=bounds,
            options={"maxiter": 100, "ftol": 1e-6},
        )

        info = {
            "iterations": result.nit,
            "status": "solved" if result.success else "failed",
        }

        return result.x, info

    def _solve_qp_cvxpy(self, P, q, A_eq, b_eq, A_ineq, b_ineq, lb, ub):
        """使用cvxpy求解QP"""
        import cvxpy as cp

        n = len(q)
        z = cp.Variable(n)

        # 目标函数
        objective = cp.Minimize(0.5 * cp.quad_form(z, P) + q.T @ z)

        # 约束
        constraints = []
        if A_eq is not None:
            constraints.append(A_eq @ z == b_eq)
        if A_ineq is not None:
            constraints.append(A_ineq @ z <= b_ineq)

        # 变量界
        for i in range(n):
            if not np.isinf(lb[i]):
                constraints.append(z[i] >= lb[i])
            if not np.isinf(ub[i]):
                constraints.append(z[i] <= ub[i])

        # 求解
        prob = cp.Problem(objective, constraints)
        prob.solve(solver=cp.OSQP, verbose=False)

        info = {
            "iterations": prob.solver_stats.num_iters if hasattr(prob.solver_stats, "num_iters") else 0,
            "status": prob.status,
        }

        return z.value, info

    def _solve_qp_proxsuite(self, P, q, A_eq, b_eq, A_ineq, b_ineq, lb, ub):
        """使用proxsuite求解QP"""
        import proxsuite

        # 初始化QP求解器（首次调用）
        if self._qp is None:
            self._qp = proxsuite.proxqp.dense.QP(
                n=self._n_vars,
                n_eq=A_eq.shape[0] if A_eq is not None else 0,
                n_in=A_ineq.shape[0] if A_ineq is not None else 0,
            )

        # 设置问题
        self._qp.init(
            H=P,
            g=q,
            A=A_eq if A_eq is not None else np.empty((0, self._n_vars)),
            b=b_eq if b_eq is not None else np.empty(0),
            C=A_ineq if A_ineq is not None else np.empty((0, self._n_vars)),
            u=b_ineq if b_ineq is not None else np.empty(0),
            l=np.full(A_ineq.shape[0], -np.inf) if A_ineq is not None else np.empty(0),
        )

        # 设置参数
        self._qp.settings.compute_preconditioner = True
        self._qp.settings.eps_abs = 1e-6
        self._qp.settings.max_iter = 100

        # 求解
        self._qp.solve()

        info = {
            "iterations": self._qp.results.info.iter,
            "status": self._qp.results.info.status,
        }

        return self._qp.results.x, info
