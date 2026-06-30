"""
Phase 6: Kino-Dynamic MPC Controller

基于Phase 4的混合动力学模型，为Phase 6的MPC+WBC架构提供准确的加速度。

与Phase 4的区别:
- Phase 4: 直接输出力矩到MuJoCo
- Phase 6: 输出力矩和加速度给WBC，由WBC再次求解QP确保动力学一致性
"""

import sys
from pathlib import Path
import numpy as np
import time

try:
    import aligator
except ImportError:
    _repo_root = Path(__file__).resolve().parents[3]
    sys.path[:0] = [str(_repo_root / "build" / "bindings" / "python")]
    import aligator

try:
    import pinocchio as pin
except ImportError:
    pass

from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel
from wheeled_ur5e_aligator_mpc.pinocchio_model import PinocchioWheeledUR5eModel
from wheeled_ur5e_aligator_mpc.hybrid_problem import HybridWheeledUR5eProblemBuilder


class KinoDynamicMPCController:
    """
    基于混合动力学的MPC控制器

    状态: x = [q_base(4), q_arm(6), v_arm(6)] = 16-dim
    控制: u = [v_base(4), tau_arm(6)] = 10-dim

    输出:
    - 最优控制 u_opt
    - 完整轨迹 xs, us
    - 准确的加速度 (从ABA计算)
    """

    def __init__(
        self,
        robot: WheeledUR5eModel,
        pin_robot: PinocchioWheeledUR5eModel,
        horizon: int = 20,
        dt: float = 0.05,
        weights: dict = None,
        max_iters: int = 50,
    ):
        """
        Parameters
        ----------
        robot : WheeledUR5eModel
            运动学模型 (提供约束和nominal config)
        pin_robot : PinocchioWheeledUR5eModel
            Pinocchio模型 (提供FK和动力学)
        horizon : int
            MPC预测时域
        dt : float
            MPC时间步长
        weights : dict, optional
            代价权重
        max_iters : int
            ProxDDP最大迭代次数
        """
        self.robot = robot
        self.pin_robot = pin_robot
        self.horizon = horizon
        self.dt = dt
        self.max_iters = max_iters

        # 构建ALIGATOR问题
        self.problem_builder = HybridWheeledUR5eProblemBuilder(
            robot=robot,
            pin_robot=pin_robot,
            horizon=horizon,
            dt=dt,
            weights=weights,
            use_hard_state_bounds=False,
        )

        # 求解器 (延迟初始化)
        self.solver = None

        # Warm start缓存
        self.xs_prev = None
        self.us_prev = None

        # 统计信息
        self.stats = {
            "solve_count": 0,
            "converged_count": 0,
            "solve_times": [],
            "iterations": [],
        }

    def solve(self, x_current, ref_traj, u_prev=None):
        """
        求解kino-dynamic MPC

        Parameters
        ----------
        x_current : (16,) array
            当前状态 [q_base(4), q_arm(6), v_arm(6)]
        ref_traj : dict
            参考轨迹 {"ee_pos": (N+1, 3), "ee_rot": (N+1, 3, 3), ...}
        u_prev : (10,) array, optional
            上一步控制 (用于smoothness cost)

        Returns
        -------
        u_opt : (10,) array
            最优控制 [v_base(4), tau_arm(6)]
        trajectory : dict
            {
                'xs': (N+1, 16) 状态轨迹,
                'us': (N, 10) 控制轨迹,
                'accelerations': (N, 11) 加速度轨迹 [a_base(3), a_wheels(2), a_arm(6)]
            }
        info : dict
            求解信息
        """
        t_start = time.perf_counter()

        # 构建OCP问题 (返回tuple: problem, ee_costs)
        problem, _ = self.problem_builder.build_problem(
            x0=x_current,
            ref_traj=ref_traj,
            u_prev=u_prev,
        )

        # 初始化求解器 (第一次调用)
        if self.solver is None:
            self.solver = aligator.SolverProxDDP(
                tol=1e-4,
                mu_init=1e-4,  # 关键参数
                max_iters=self.max_iters,
                verbose=aligator.VerboseLevel.QUIET,
            )
            self.solver.rollout_type = aligator.ROLLOUT_LINEAR

        # Warm start
        if self.xs_prev is not None and self.us_prev is not None:
            xs_init, us_init = self._shift_warm_start(self.xs_prev, self.us_prev, x_current)
        else:
            xs_init = [x_current] * (self.horizon + 1)
            us_init = [np.zeros(10)] * self.horizon

        # 求解
        self.solver.setup(problem)
        try:
            self.solver.run(problem, xs_init, us_init)
            success = True
            # ALIGATOR 0.19没有converged属性，用其他方式判断
            converged = (self.solver.results.num_iters < self.max_iters - 1)
            status = "converged" if converged else "max_iters"
        except Exception as e:
            print(f"[KinoDynMPC] Solver failed: {e}")
            success = False
            status = "failed"
            converged = False

        t_solve = time.perf_counter() - t_start

        # 提取结果
        if success and len(self.solver.results.xs) > 0:
            xs_opt = np.array([np.array(x) for x in self.solver.results.xs])
            us_opt = np.array([np.array(u) for u in self.solver.results.us])
            u_opt = us_opt[0]

            # 缓存用于下次warm start
            self.xs_prev = xs_opt
            self.us_prev = us_opt

            # 计算加速度 (从动力学)
            accelerations = self._compute_accelerations(xs_opt, us_opt)
        else:
            # Fallback
            u_opt = u_prev if u_prev is not None else np.zeros(10)
            xs_opt = np.array([x_current] * (self.horizon + 1))
            us_opt = np.array([u_opt] * self.horizon)
            accelerations = np.zeros((self.horizon, 11))

        # 统计信息
        self.stats["solve_count"] += 1
        if success and converged:
            self.stats["converged_count"] += 1
        self.stats["solve_times"].append(t_solve * 1000)  # ms
        self.stats["iterations"].append(self.solver.results.num_iters if success else 0)

        info = {
            "success": success,
            "status": status,
            "converged": converged,
            "num_iters": self.solver.results.num_iters if success else 0,
            "solve_time_ms": t_solve * 1000,
            "cost": float(self.solver.results.traj_cost) if success else np.inf,
        }

        trajectory = {
            'xs': xs_opt,
            'us': us_opt,
            'accelerations': accelerations,
        }

        return u_opt, trajectory, info

    def _shift_warm_start(self, xs_prev, us_prev, x_current):
        """Shift轨迹用于warm start"""
        xs_init = list(xs_prev[1:]) + [xs_prev[-1]]
        us_init = list(us_prev[1:]) + [us_prev[-1]]

        # 第一个状态用当前测量值
        xs_init[0] = x_current

        return xs_init, us_init

    def _compute_accelerations(self, xs, us):
        """
        从状态和控制轨迹计算加速度

        这是关键：提供准确的动力学信息给WBC！

        Parameters
        ----------
        xs : (N+1, 16) array
            状态轨迹
        us : (N, 10) array
            控制轨迹

        Returns
        -------
        accelerations : (N, 11) array
            [a_base(3), a_wheels(2), a_arm(6)]
            注意: 这是WBC期望的11-dim加速度格式
        """
        N = len(us)
        accelerations = np.zeros((N, 11))

        arm_model = self.pin_robot.arm_model
        arm_data = self.pin_robot.arm_data

        # Damping from hybrid_dynamics.py
        damping = np.array([1.0, 1.0, 0.5, 0.1, 0.1, 0.1])

        for i in range(N):
            x = xs[i]
            u = us[i]

            q_arm = x[4:10]
            v_arm = x[10:16]
            tau_arm = u[4:10]

            # 机械臂加速度 (使用ABA - 与hybrid_dynamics一致)
            tau_damped = tau_arm - damping * v_arm
            a_arm = pin.aba(arm_model, arm_data, q_arm, v_arm, tau_damped)

            # 基座加速度 (从速度差分估计，因为是运动学控制)
            # 对于WBC，我们需要 [a_base_x, a_base_y, a_yaw]
            # 简化：假设基座匀速 (速度控制)
            a_base = np.zeros(3)  # [ax_world, ay_world, alpha_yaw]

            # 轮子加速度 (从基座速度计算)
            # 简化：假设轮子也匀速
            a_wheels = np.zeros(2)

            # 组装 WBC期望的格式: [a_base(3), a_wheels(2), a_arm(6)]
            accelerations[i] = np.concatenate([a_base, a_wheels, a_arm])

        return accelerations

    def get_statistics(self):
        """获取求解器统计信息"""
        stats = self.stats.copy()
        if stats["solve_count"] > 0:
            stats["convergence_rate"] = stats["converged_count"] / stats["solve_count"]
            stats["avg_solve_time_ms"] = np.mean(stats["solve_times"])
            stats["max_solve_time_ms"] = np.max(stats["solve_times"])
            stats["avg_iterations"] = np.mean(stats["iterations"])
        return stats

    def reset_statistics(self):
        """重置统计信息"""
        self.stats = {
            "solve_count": 0,
            "converged_count": 0,
            "solve_times": [],
            "iterations": [],
        }
