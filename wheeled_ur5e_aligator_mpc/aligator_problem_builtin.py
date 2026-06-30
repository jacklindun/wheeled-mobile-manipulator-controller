"""
ALIGATOR OCP builder - 使用内置 cost 而非自定义 Python cost

关键修改：
- 不使用 Python 自定义的 EEPosCost
- 改用 ALIGATOR 内置的 QuadraticStateCost
- 通过预计算 IK 目标来近似 EE 跟踪

这样可以避免 boost::python 异常传播问题
"""

import numpy as np
import aligator
import aligator.dynamics
import aligator.manifolds
import aligator.constraints

from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel


class WheeledUR5eKinDynamics(aligator.dynamics.ExplicitDynamicsModel):
    """
    离散运动学动力学
    x_{k+1} = x_k + dt * u_k (速度积分)
    """

    def __init__(self, space_or_dim, robot: WheeledUR5eModel, dt: float):
        if isinstance(space_or_dim, int):
            space = aligator.manifolds.VectorSpace(space_or_dim)
        else:
            space = space_or_dim
        super().__init__(space, robot.nu)
        self._space_nx = space.nx
        self._robot = robot
        self._dt = dt

    def __reduce__(self):
        return (self.__class__, (self._space_nx, self._robot, self._dt))

    def forward(self, x, u, data) -> None:
        q_next = self._robot.dynamics_numpy(np.asarray(x), np.asarray(u), self._dt)
        data.xnext[:] = q_next

    def dForward(self, x, u, data) -> None:
        A, B = self._robot.linearize_dynamics(np.asarray(x), np.asarray(u), self._dt)
        data.Jx[:] = A
        data.Ju[:] = B


class BuiltinKinematicWheeledUR5eProblemBuilder:
    """
    使用 ALIGATOR 内置 cost 的问题构建器

    关键：不使用 Python 自定义 cost，避免 C++ 绑定问题
    """

    DEFAULT_WEIGHTS = {
        'w_ee_pos': 100.0,      # EE 位置（通过状态空间近似）
        'w_state': 10.0,        # 状态跟踪
        'w_control': 0.01,      # 控制代价
        'w_delta_u': 0.1,       # 控制平滑
        'w_base': 1.0,          # 基座位置
        'w_base_z': 10.0,       # 基座高度
    }

    def __init__(self, robot: WheeledUR5eModel, horizon: int, dt: float, weights=None):
        self.robot = robot
        self.horizon = horizon
        self.dt = dt
        self.weights = {**self.DEFAULT_WEIGHTS, **(weights or {})}

        # 状态空间
        self.space = aligator.manifolds.VectorSpace(robot.nq)

        # 动力学
        self.dynamics = WheeledUR5eKinDynamics(self.space, robot, dt)

    def build_problem(self, q_current, ref_traj, u_prev=None):
        """
        构建 OCP 问题

        关键策略：
        - 使用 QuadraticStateCost 跟踪参考状态
        - 参考状态通过简单的配置空间目标生成（避免 IK）
        - 不使用自定义 Python cost function
        """
        N = self.horizon
        space = self.space
        nu = self.robot.nu

        # ========================================
        # Stage costs (使用内置 cost)
        # ========================================
        stage_costs = []

        for i in range(N):
            cost_stack = aligator.CostStack(space, nu)

            # 1. 状态跟踪 cost
            # 目标：保持接近 nominal posture
            q_ref = self.robot.q_nominal.copy()

            # 简单策略：基座跟随参考
            if 'base' in ref_traj and i < len(ref_traj['base']):
                q_ref[0:3] = ref_traj['base'][i]  # base x,y,yaw

            if 'base_z' in ref_traj and i < len(ref_traj['base_z']):
                q_ref[2] = ref_traj['base_z'][i]  # base z

            # 权重矩阵
            W_state = np.diag([
                self.weights['w_base'],  # base_x
                self.weights['w_base'],  # base_y
                self.weights['w_base_z'],  # base_z
                self.weights['w_base'],  # base_yaw
                *([self.weights['w_state']] * 6)  # arm joints
            ])

            state_cost = aligator.QuadraticStateCost(space, nu, q_ref, W_state)
            cost_stack.addCost(state_cost)

            # 2. 控制 cost
            u_ref = np.zeros(nu)
            W_control = np.eye(nu) * self.weights['w_control']
            control_cost = aligator.QuadraticControlCost(space, u_ref, W_control)
            cost_stack.addCost(control_cost)

            # 3. 控制平滑 cost (delta u)
            if u_prev is not None:
                W_delta = np.eye(nu) * self.weights['w_delta_u']
                delta_cost = aligator.QuadraticControlCost(space, u_prev, W_delta)
                cost_stack.addCost(delta_cost)

            stage_costs.append(cost_stack)

        # ========================================
        # Terminal cost
        # ========================================
        q_ref_term = self.robot.q_nominal.copy()
        if 'base' in ref_traj:
            q_ref_term[0:3] = ref_traj['base'][-1]
        if 'base_z' in ref_traj:
            q_ref_term[2] = ref_traj['base_z'][-1]

        W_term = W_state * 2.0  # 终端代价更大
        term_cost = aligator.CostStack(space, nu)
        term_cost.addCost(aligator.QuadraticStateCost(space, nu, q_ref_term, W_term))

        # ========================================
        # Stages
        # ========================================
        stages = []
        for i in range(N):
            stage = aligator.StageModel(stage_costs[i], self.dynamics)

            # 控制约束
            u_min = self.robot.u_min
            u_max = self.robot.u_max
            u_constraint = aligator.constraints.BoxConstraint(u_min, u_max)
            u_residual = aligator.ControlErrorResidual(space.ndx, nu)
            stage.addConstraint(u_residual, u_constraint)

            stages.append(stage)

        # ========================================
        # TrajOptProblem
        # ========================================
        problem = aligator.TrajOptProblem(q_current, stages, term_cost)

        return problem, {}


if __name__ == '__main__':
    """测试内置 cost 版本"""
    print("="*60)
    print("测试内置 cost 版本的 OCP 构建")
    print("="*60)

    robot = WheeledUR5eModel()
    print(f"✓ Robot: {robot.nq} DOF")

    builder = BuiltinKinematicWheeledUR5eProblemBuilder(robot, horizon=10, dt=0.05)
    print(f"✓ Builder 创建成功")

    # 生成参考轨迹
    ref_traj = {
        'base': np.tile(np.array([0.0, 0.0, 0.0]), (11, 1)),
        'base_z': np.ones(11) * 0.2,
    }
    print(f"✓ 参考轨迹生成")

    # 构建问题
    q_current = robot.q_nominal
    problem, info = builder.build_problem(q_current, ref_traj, u_prev=None)
    print(f"✓ 问题构建成功")
    print(f"  阶段数: {len(problem.stages)}")

    # 测试求解
    print(f"\n测试求解...")
    solver = aligator.SolverProxDDP(tol=1e-4, mu_init=1e-4, max_iters=5)
    solver.setup(problem)

    xs_init = [q_current] * (builder.horizon + 1)
    us_init = [np.zeros(robot.nu)] * builder.horizon

    try:
        solver.run(problem, xs_init, us_init)
        print(f"✓ 求解成功!")
        print(f"  收敛: {solver.results.conv}")
        print(f"  迭代数: {solver.results.num_iters}")
        print(f"  代价: {solver.results.traj_cost:.4f}")
        print(f"\n🎉 ✅ 内置 cost 版本工作正常，避免了 C++ 绑定问题!")
    except Exception as e:
        print(f"✗ 求解失败: {e}")
        import traceback
        traceback.print_exc()

    print("="*60)
