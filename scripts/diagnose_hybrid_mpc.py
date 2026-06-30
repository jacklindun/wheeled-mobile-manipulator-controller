#!/usr/bin/env python3
"""
混合MPC深度诊断工具

当调参无法改善收敛率时，诊断根本问题：
1. 动力学模型预测 vs MuJoCo执行的匹配度
2. 梯度/Hessian数值稳定性
3. 代价函数各项的数值范围
4. 初始warm-start质量

Usage:
  python scripts/diagnose_hybrid_mpc.py [--render]
"""

import sys
from pathlib import Path

_project_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_project_root))

_aligator_root = _project_root.parents[1]
sys.path[:0] = [
    str(_aligator_root / "build" / "bindings" / "python"),
    str(_aligator_root / "bindings" / "python"),
]

import numpy as np
import aligator
import mujoco

from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel
from wheeled_ur5e_aligator_mpc.pinocchio_model import PinocchioWheeledUR5eModel
from wheeled_ur5e_aligator_mpc.mujoco_env_hybrid import MujocoWheeledUR5eHybridEnv
from wheeled_ur5e_aligator_mpc.hybrid_problem import HybridWheeledUR5eProblemBuilder
from wheeled_ur5e_aligator_mpc.hybrid_dynamics import HybridWheeledUR5eDynamics
from wheeled_ur5e_aligator_mpc.reference import ReferenceGenerator


def diagnose_dynamics_match():
    """诊断1：动力学模型预测 vs MuJoCo执行是否匹配"""
    print("\n" + "="*80)
    print("诊断1: 动力学模型匹配度")
    print("="*80)

    robot = WheeledUR5eModel()
    pin_robot = PinocchioWheeledUR5eModel()
    env = MujocoWheeledUR5eHybridEnv(render=False)

    dt = 0.05
    dynamics = HybridWheeledUR5eDynamics(pin_robot, dt)

    # 测试点：q_nominal + 小扰动
    x0 = np.zeros(16)
    x0[:10] = robot.q_nominal
    x0[10:16] = np.random.randn(6) * 0.1  # 随机小速度

    # 测试控制：小扭矩
    u_test = np.zeros(10)
    u_test[:4] = [0.01, 0.0, 0.0, 0.0]  # 小速度
    u_test[4:10] = np.random.randn(6) * 2.0  # 小扭矩

    print(f"\n初始状态 x0:")
    print(f"  q_base = {x0[:4]}")
    print(f"  q_arm  = {x0[4:10]}")
    print(f"  v_arm  = {x0[10:16]}")
    print(f"\n控制 u:")
    print(f"  v_base = {u_test[:4]}")
    print(f"  tau_arm = {u_test[4:10]}")

    # ALIGATOR动力学预测
    data = dynamics.createData()
    dynamics.forward(x0, u_test, data)
    x_pred = np.array(data.xnext)

    print(f"\nALIGATOR预测 x_next:")
    print(f"  q_base = {x_pred[:4]}")
    print(f"  q_arm  = {x_pred[4:10]}")
    print(f"  v_arm  = {x_pred[10:16]}")

    # MuJoCo执行
    env.reset(x0[:10])
    # 设置初始速度
    for i, jname in enumerate(["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]):
        env.data.qvel[env._joint_qvel_adr[jname]] = x0[10 + i]

    mujoco.mj_forward(env.model, env.data)

    # 执行控制
    env.set_control(u_test)
    substeps = int(dt / env.model.opt.timestep)
    env.step(substeps)

    x_mujoco = env.get_state()

    print(f"\nMuJoCo执行 x_next:")
    print(f"  q_base = {x_mujoco[:4]}")
    print(f"  q_arm  = {x_mujoco[4:10]}")
    print(f"  v_arm  = {x_mujoco[10:16]}")

    # 误差分析
    error = x_mujoco - x_pred
    print(f"\n误差 (MuJoCo - ALIGATOR):")
    print(f"  Δq_base = {error[:4]} (norm={np.linalg.norm(error[:4]):.6f})")
    print(f"  Δq_arm  = {error[4:10]} (norm={np.linalg.norm(error[4:10]):.6f})")
    print(f"  Δv_arm  = {error[10:16]} (norm={np.linalg.norm(error[10:16]):.6f})")
    print(f"  总误差范数 = {np.linalg.norm(error):.6f}")

    env.close()

    # 判断
    if np.linalg.norm(error[:10]) < 1e-4:
        print("\n✓ 位置误差很小 (<1e-4)，动力学模型匹配良好")
        return True
    elif np.linalg.norm(error[:10]) < 1e-3:
        print("\n⚠ 位置误差中等 (<1e-3)，可能有小的模型偏差")
        return True
    else:
        print("\n✗ 位置误差较大 (>1e-3)，动力学模型与MuJoCo不匹配！")
        print("  可能原因：")
        print("  - 关节阻尼参数不一致")
        print("  - ABA计算有误")
        print("  - 积分器差异（semi-implicit vs implicitfast）")
        return False


def diagnose_cost_scale():
    """诊断2：代价函数各项的数值范围"""
    print("\n" + "="*80)
    print("诊断2: 代价函数数值范围")
    print("="*80)

    robot = WheeledUR5eModel()
    pin_robot = PinocchioWheeledUR5eModel()

    horizon = 20
    dt = 0.05
    builder = HybridWheeledUR5eProblemBuilder(
        robot, pin_robot,
        horizon=horizon,
        dt=dt,
        use_hard_state_bounds=False,
    )

    # 生成参考
    p_ee_nominal, R_ee_nominal = pin_robot.fk_pose(robot.q_nominal)
    ref_gen = ReferenceGenerator(scenario='ee_circle', ee_start=p_ee_nominal, ee_start_rot=R_ee_nominal)
    ref_traj = ref_gen.get_reference(t=0.0, horizon=horizon, dt=dt)

    # 初始状态
    x0 = np.zeros(16)
    x0[:10] = robot.q_nominal

    # 构建问题
    problem, _ = builder.build_problem(x0, ref_traj, u_prev=None)

    # 分析初始代价
    print(f"\n初始点 x0 的代价函数分析:")

    # 运行代价（第一阶段）
    stage0 = problem.stages[0]
    cost_data = stage0.cost.createData()

    # 计算总代价
    u0 = np.zeros(10)
    stage0.cost.calc(cost_data, x0, u0)
    total_cost = cost_data.value_

    print(f"  初始点总代价: {total_cost:.6e}")

    # 尝试分解（这需要访问内部代价项，简化输出）
    print(f"  权重配置:")
    for key, val in builder.weights.items():
        print(f"    {key:20s} = {val}")

    # 测试不同状态的代价范围
    print(f"\n测试不同偏差下的代价:")

    # EE偏差1cm
    x_test = x0.copy()
    x_test[5] += 0.02  # shoulder_lift微调，导致EE偏移
    stage0.cost.calc(cost_data, x_test, u0)
    cost_1cm = cost_data.value_
    print(f"  EE偏差~1cm:   代价 = {cost_1cm:.6e}  (Δ={cost_1cm-total_cost:.6e})")

    # 扭矩10Nm
    u_test = np.zeros(10)
    u_test[4:10] = 10.0  # 10Nm扭矩
    stage0.cost.calc(cost_data, x0, u_test)
    cost_10nm = cost_data.value_
    print(f"  扭矩10Nm:     代价 = {cost_10nm:.6e}  (Δ={cost_10nm-total_cost:.6e})")

    # 速度0.5 rad/s
    x_test = x0.copy()
    x_test[10:16] = 0.5
    stage0.cost.calc(cost_data, x_test, u0)
    cost_05v = cost_data.value_
    print(f"  速度0.5rad/s: 代价 = {cost_05v:.6e}  (Δ={cost_05v-total_cost:.6e})")

    # 判断
    print(f"\n代价函数数值范围分析:")
    if total_cost < 1e-6:
        print("  ⚠ 初始代价接近0，可能权重过小")
    elif total_cost > 1e6:
        print("  ⚠ 初始代价过大，可能权重失衡")
    else:
        print("  ✓ 初始代价在合理范围")

    delta_ee = abs(cost_1cm - total_cost)
    delta_tau = abs(cost_10nm - total_cost)

    if delta_tau > delta_ee * 100:
        print("  ✗ 扭矩惩罚过强（比EE跟踪强100倍），限制了控制能力")
        print("    建议：降低 tau_arm 权重 10-100倍")
        return False
    elif delta_ee > delta_tau * 100:
        print("  ✗ EE跟踪权重过强（比扭矩正则化强100倍），可能导致过大扭矩")
        print("    建议：增加 tau_arm 权重 10-100倍")
        return False
    else:
        print("  ✓ EE跟踪与扭矩正则化权重比例合理")
        return True


def diagnose_solver_behavior():
    """诊断3：求解器行为（收敛曲线、KKT残差）"""
    print("\n" + "="*80)
    print("诊断3: 求解器收敛行为")
    print("="*80)

    robot = WheeledUR5eModel()
    pin_robot = PinocchioWheeledUR5eModel()

    horizon = 20
    dt = 0.05
    builder = HybridWheeledUR5eProblemBuilder(
        robot, pin_robot,
        horizon=horizon,
        dt=dt,
        use_hard_state_bounds=False,
    )

    # 参考轨迹
    p_ee_nominal, R_ee_nominal = pin_robot.fk_pose(robot.q_nominal)
    ref_gen = ReferenceGenerator(scenario='ee_circle', ee_start=p_ee_nominal, ee_start_rot=R_ee_nominal)
    ref_traj = ref_gen.get_reference(t=0.0, horizon=horizon, dt=dt)

    x0 = np.zeros(16)
    x0[:10] = robot.q_nominal

    problem, _ = builder.build_problem(x0, ref_traj, u_prev=None)

    # 测试不同mu_init
    print("\n测试不同 mu_init 的收敛行为:")
    print(f"{'mu_init':<12} {'iter':<6} {'cost':<14} {'prim_infeas':<14} {'dual_infeas':<14} {'收敛'}")
    print("-"*80)

    for mu in [1e-1, 5e-2, 2e-2, 1e-2, 5e-3, 1e-3]:
        solver = aligator.SolverProxDDP(tol=1e-2, mu_init=mu, max_iters=50)

        xs_init = [x0.copy() for _ in range(horizon + 1)]
        us_init = [np.zeros(10) for _ in range(horizon)]

        solver.setup(problem)
        solver.run(problem, xs_init, us_init)

        conv_str = "✓" if solver.results.conv else "✗"
        print(f"{mu:<12.1e} {solver.results.num_iters:<6d} {solver.results.traj_cost:<14.6e} "
              f"{solver.results.prim_infeas:<14.6e} {solver.results.dual_infeas:<14.6e} {conv_str}")

    print("\n说明:")
    print("  - prim_infeas (原始不可行性): 约束违反程度")
    print("  - dual_infeas (对偶不可行性): 梯度范数")
    print("  - 收敛条件: max(prim_infeas, dual_infeas) < tolerance")

    # 分析
    print("\n如果所有配置都:")
    print("  - num_iters = 50: 达到最大迭代，未收敛")
    print("  - prim_infeas > 1e-2: 原始不可行性过大，约束违反严重")
    print("  - dual_infeas > 1e-2: 梯度未归零，远离最优点")
    print("\n则问题可能是:")
    print("  1. 优化问题本身ill-conditioned（病态）")
    print("  2. 初始化太差（warm-start质量低）")
    print("  3. 动力学约束太难满足")


def main():
    import argparse
    parser = argparse.ArgumentParser(description="混合MPC深度诊断")
    parser.add_argument("--render", action="store_true", help="启用渲染")
    args = parser.parse_args()

    print("\n" + "="*80)
    print("混合MPC深度诊断工具")
    print("="*80)
    print("\n目标：找出阶段1调参无效的根本原因")
    print()

    # 运行三个诊断
    diag1_ok = diagnose_dynamics_match()
    diag2_ok = diagnose_cost_scale()
    diagnose_solver_behavior()

    # 总结
    print("\n" + "="*80)
    print("诊断总结")
    print("="*80)

    if not diag1_ok:
        print("\n✗ 主要问题：动力学模型与MuJoCo不匹配")
        print("  解决方案：")
        print("  1. 检查 hybrid_dynamics.py 中的关节阻尼参数")
        print("  2. 对比 MJCF 与 ABA 的惯性参数")
        print("  3. 验证 semi-implicit Euler 积分的正确性")

    elif not diag2_ok:
        print("\n✗ 主要问题：代价函数权重失衡")
        print("  解决方案：")
        print("  1. 调整 tau_arm/ee_pos 权重比例")
        print("  2. 使用归一化的代价项")
        print("  3. 尝试不同的权重组合")

    else:
        print("\n⚠ 动力学和代价函数看起来合理")
        print("  可能问题：")
        print("  1. 优化问题本身病态（Hessian条件数过大）")
        print("  2. warm-start初始化质量差（全零初值）")
        print("  3. 需要从运动学MPC提供初始解")
        print()
        print("  建议尝试：")
        print("  1. 使用运动学MPC warm-start")
        print("  2. 增加Gauss-Newton正则化")
        print("  3. 更激进地降低 mu_init (1e-3, 1e-4)")


if __name__ == "__main__":
    main()
