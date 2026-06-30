"""
Phase 6 Step 1: Full Dynamic MPC Controller

基于Phase 5完整动力学的MPC控制器

状态空间: x = [q(12), v(11)] = 23-dim
  q = [base(4), θ_wheels(2), arm(6)]
  v = [v_base(3), ω_wheels(2), v_arm(6)]

控制空间: u = [τ_wheels(2), τ_arm(6)] = 8-dim

特点:
- 全扭矩控制（vs Phase 4的混合控制）
- 输出完整轨迹供插值器使用
- 解决积分器不匹配问题的基础
"""

import sys
from pathlib import Path
import numpy as np
import time

try:
    import aligator
    import pinocchio as pin
except ImportError:
    _repo_root = Path(__file__).resolve().parents[3]
    sys.path[:0] = [str(_repo_root / "build" / "bindings" / "python")]
    import aligator
    import pinocchio as pin

from wheeled_ur5e_aligator_mpc.pinocchio_model import PinocchioWheeledUR5eModel
from wheeled_ur5e_aligator_mpc.wheeled_dynamics import WheeledUR5eDynamics, WheelParameters


class FullDynamicMPCWeights:
    """MPC代价函数权重"""
    def __init__(self):
        # EE跟踪权重
        self.ee_position = 1000.0      # EE位置跟踪（主要目标）
        self.ee_orientation = 0.0      # EE姿态（可选，暂时关闭）

        # 基座权重
        self.base_position = 10.0      # 基座位置
        self.base_yaw = 10.0           # 基座朝向

        # 姿态权重
        self.posture_arm = 1.0         # 机械臂姿态正则化
        self.posture_wheels = 0.1      # 轮子角度正则化

        # 控制权重
        self.control_reg = 0.01        # 扭矩正则化
        self.control_smooth = 1.0      # 扭矩平滑（减少抖动）

        # 终端权重（horizon末端）
        self.terminal_multiplier = 5.0  # 终端代价放大系数


class EEPositionCostFullDynamic(aligator.CostAbstract):
    """
    EE位置跟踪代价函数（适配23-dim状态）

    从23-dim状态中提取机械臂配置，计算EE位置误差
    """

    def __init__(self, space_or_nx, pin_robot, target_pos, weight=1.0):
        """
        Parameters
        ----------
        space_or_nx : aligator.manifolds.VectorSpace or int
            状态空间或维度 (23-dim)
        pin_robot : PinocchioWheeledUR5eModel
            Pinocchio模型
        target_pos : (3,) array
            目标EE位置
        weight : float
            权重
        """
        nu = 8  # 控制维度
        
        # 兼容处理：如果传入int，创建VectorSpace
        if isinstance(space_or_nx, int):
            space = aligator.manifolds.VectorSpace(space_or_nx)
        else:
            space = space_or_nx
            
        super().__init__(space, nu)

        self._pin_robot = pin_robot
        self._target_pos = np.asarray(target_pos)
        self._weight = weight

    def evaluate(self, x, u, data):
        """计算代价值"""
        # 从23-dim状态提取机械臂配置
        q_10dim = self._extract_kinematic_state(x)

        # 计算EE位置
        ee_pos = self._pin_robot.fk_ee(q_10dim)

        # 位置误差
        error = ee_pos - self._target_pos

        # 代价 = 0.5 * w * ||error||²
        data.value = 0.5 * self._weight * np.dot(error, error)

    def computeGradients(self, x, u, data):
        """计算梯度（简化：使用有限差分）"""
        # TODO: 实现解析梯度
        # 当前使用数值梯度作为占位
        data.Lx[:] = 0.0
        data.Lu[:] = 0.0

    def computeHessians(self, x, u, data):
        """计算Hessian（简化：使用Gauss-Newton近似）"""
        # TODO: 实现解析Hessian
        data.Lxx[:, :] = 0.0
        data.Luu[:, :] = 0.0
        data.Lxu[:, :] = 0.0

    def _extract_kinematic_state(self, x):
        """
        从23-dim完整状态提取10-dim运动学状态

        x_23 = [q_base(4), θ_wheels(2), q_arm(6), v_base(3), ω_wheels(2), v_arm(6)]
        q_10 = [q_base(4), q_arm(6)]
        """
        q_base = x[0:4]
        q_arm = x[6:12]
        return np.concatenate([q_base, q_arm])

    def __reduce__(self):
        """支持deepcopy"""
        return (
            self.__class__,
            (self.space.ndx, self._pin_robot, self._target_pos, self._weight),
        )


class FullDynamicMPCController:
    """
    Phase 6 Full Dynamic MPC控制器

    与Phase 4的区别:
    - Phase 4: 混合控制 (基座速度 + 机械臂扭矩)
    - Phase 6: 纯扭矩控制 (轮子扭矩 + 机械臂扭矩)

    输出:
    - 不直接输出给MuJoCo
    - 输出完整轨迹供插值器使用
    """

    def __init__(self, horizon=20, dt=0.05, weights=None):
        """
        Parameters
        ----------
        horizon : int
            MPC预测视野
        dt : float
            MPC时间步长 (0.05s = 20Hz)
        weights : FullDynamicMPCWeights
            代价权重
        """
        # 初始化模型
        self.pin_robot = PinocchioWheeledUR5eModel()
        self.wheel_params = WheelParameters()

        # 动力学模型
        self.dynamics = WheeledUR5eDynamics(
            self.pin_robot,
            dt=dt,
            wheel_params=self.wheel_params
        )

        # MPC参数
        self.horizon = horizon
        self.dt = dt
        self.nx = 23  # 状态维度
        self.nu = 8   # 控制维度

        # 权重
        if weights is None:
            weights = FullDynamicMPCWeights()
        self.weights = weights

        # ALIGATOR空间
        self.space = aligator.manifolds.VectorSpace(self.nx)

        # 求解器（延迟初始化）
        self.solver = None

        # 上一步的解（用于warm start）
        self.last_xs = None
        self.last_us = None

    def solve(self, x_current, ref_traj):
        """
        求解MPC优化问题

        Parameters
        ----------
        x_current : (23,) array
            当前状态
        ref_traj : dict
            参考轨迹，格式:
            {
                'ee_pos': (N, 3) EE目标位置
                'base_pos': (N, 4) 基座目标 [x, y, z, yaw]
                'times': (N,) 时间点
            }

        Returns
        -------
        result : dict
            {
                'xs': (N+1, 23) 状态轨迹
                'us': (N, 8) 控制轨迹
                'ts': (N+1,) 时间点
                'solve_time': float 求解时间(秒)
                'converged': bool 是否收敛
                'iterations': int 迭代次数
            }
        """
        t_start = time.time()

        # 构建OCP问题
        problem = self._build_problem(x_current, ref_traj)

        # 初始化求解器（ALIGATOR 0.19.0 API）
        if self.solver is None:
            self.solver = aligator.SolverProxDDP(
                tol=1e-4,
                mu_init=1e-4,
                max_iters=50,
                verbose=aligator.VerboseLevel.QUIET,
            )
            self.solver.rollout_type = aligator.ROLLOUT_LINEAR
            self.solver.setup(problem)
        else:
            # 更新问题
            self.solver.setup(problem)

        # Warm start
        if self.last_xs is not None and self.last_us is not None:
            # 平移上一步的轨迹
            xs_init = list(self.last_xs[1:]) + [self.last_xs[-1]]
            us_init = list(self.last_us[1:]) + [self.last_us[-1]]
        else:
            # 零初始化
            xs_init = [x_current] * (self.horizon + 1)
            us_init = [np.zeros(self.nu)] * self.horizon

        # 求解
        converged = self.solver.run(problem, xs_init, us_init)

        t_solve = time.time() - t_start

        # 提取结果
        xs = np.array(self.solver.results.xs)
        us = np.array(self.solver.results.us)
        ts = np.arange(len(xs)) * self.dt

        # 保存用于warm start
        self.last_xs = xs
        self.last_us = us

        return {
            'xs': xs,
            'us': us,
            'ts': ts,
            'solve_time': t_solve,
            'converged': converged,
            'iterations': self.solver.results.num_iters,
        }

    def _build_problem(self, x_init, ref_traj):
        """构建ALIGATOR OCP问题"""
        # 初始状态
        x0 = np.asarray(x_init)

        # 创建TrajOptProblem
        stages = []

        for k in range(self.horizon):
            # 构建第k步的stage
            stage = self._create_stage(k, ref_traj)
            stages.append(stage)

        # 终端代价
        term_cost = self._create_terminal_cost(ref_traj)

        # 组装问题
        problem = aligator.TrajOptProblem(x0, stages, term_cost)

        return problem

    def _create_stage(self, k, ref_traj):
        """创建单个stage"""
        # 1. 动力学
        dynamics = self.dynamics

        # 2. 代价函数
        cost_stack = aligator.CostStack(self.space, self.nu)

        # EE位置跟踪
        if 'ee_pos' in ref_traj:
            ee_target = ref_traj['ee_pos'][min(k, len(ref_traj['ee_pos'])-1)]
            ee_cost = EEPositionCostFullDynamic(
                self.space,
                self.pin_robot,
                ee_target,
                weight=self.weights.ee_position
            )
            cost_stack.addCost(ee_cost)

        # 控制正则化
        u_reg_cost = aligator.QuadraticControlCost(
            self.space,
            np.zeros(self.nu),
            self.weights.control_reg * np.eye(self.nu)
        )
        cost_stack.addCost(u_reg_cost)

        # 3. 创建StageModel
        stage = aligator.StageModel(cost_stack, dynamics)

        # 4. 约束（可选）
        # TODO: 添加扭矩限制约束
        # TODO: 添加非完整约束

        return stage

    def _create_terminal_cost(self, ref_traj):
        """创建终端代价"""
        cost_stack = aligator.CostStack(self.space, self.nu)

        # 终端EE位置
        if 'ee_pos' in ref_traj:
            ee_target = ref_traj['ee_pos'][-1]
            ee_cost = EEPositionCostFullDynamic(
                self.space,
                self.pin_robot,
                ee_target,
                weight=self.weights.ee_position * self.weights.terminal_multiplier
            )
            cost_stack.addCost(ee_cost)

        return cost_stack


def create_simple_reference_trajectory(scenario='stationary', duration=5.0, dt=0.05):
    """
    创建简单的参考轨迹

    Parameters
    ----------
    scenario : str
        场景类型: 'stationary', 'ee_circle', 'base_forward'
    duration : float
        轨迹时长
    dt : float
        时间步长

    Returns
    -------
    ref_traj : dict
        参考轨迹
    """
    N = int(duration / dt)
    times = np.arange(N+1) * dt

    if scenario == 'stationary':
        # 静止目标
        ee_pos_nominal = np.array([0.619, 0.064, 0.857])  # 从q_nominal计算得到
        ee_pos = np.tile(ee_pos_nominal, (N+1, 1))

    elif scenario == 'ee_circle':
        # EE画圆（YZ平面，半径10cm）
        ee_pos_nominal = np.array([0.619, 0.064, 0.857])
        radius = 0.10

        ee_pos = np.zeros((N+1, 3))
        for i, t in enumerate(times):
            theta = 2 * np.pi * t / duration
            ee_pos[i] = ee_pos_nominal + np.array([
                0.0,
                radius * np.sin(theta),
                radius * (np.cos(theta) - 1.0)
            ])

    else:
        raise ValueError(f"Unknown scenario: {scenario}")

    return {
        'ee_pos': ee_pos,
        'times': times,
    }


if __name__ == '__main__':
    """简单测试"""
    print("="*60)
    print("Phase 6 Full Dynamic MPC 测试")
    print("="*60)

    # 创建控制器
    controller = FullDynamicMPCController(horizon=10, dt=0.05)

    print(f"\n控制器参数:")
    print(f"  状态维度: {controller.nx}")
    print(f"  控制维度: {controller.nu}")
    print(f"  MPC horizon: {controller.horizon}")
    print(f"  MPC dt: {controller.dt} s")

    # 创建初始状态
    x_current = np.zeros(23)
    x_current[2] = 0.2  # base_z
    x_current[6:12] = [np.pi, np.pi/3, -np.pi/2, np.pi/6, 0.0, 0.0]  # arm nominal

    # 创建参考轨迹
    ref_traj = create_simple_reference_trajectory('stationary', duration=2.0, dt=0.05)

    print(f"\n参考轨迹:")
    print(f"  场景: stationary")
    print(f"  时长: 2.0 s")
    print(f"  EE目标: {ref_traj['ee_pos'][0]}")

    # 求解MPC
    print(f"\n求解MPC...")
    try:
        result = controller.solve(x_current, ref_traj)

        print(f"\n✓ MPC求解成功!")
        print(f"  求解时间: {result['solve_time']:.3f} s")
        print(f"  收敛: {result['converged']}")
        print(f"  迭代次数: {result['iterations']}")
        print(f"  轨迹长度: {len(result['xs'])} 步")
        print(f"  控制范围: τ∈[{result['us'].min():.2f}, {result['us'].max():.2f}] N·m")

    except Exception as e:
        print(f"\n✗ MPC求解失败:")
        print(f"  错误: {e}")
        import traceback
        traceback.print_exc()

    print("\n" + "="*60)
