#!/usr/bin/env python3
"""
诊断Phase 6-v2的ALIGATOR C++绑定问题
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
import traceback

print("="*80)
print("诊断ALIGATOR C++绑定问题")
print("="*80)

try:
    print("\n[1/6] 导入基础模块...")
    import aligator
    print(f"  ✓ ALIGATOR {aligator.__version__}")

    from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel
    print("  ✓ Robot模型")

    from wheeled_ur5e_aligator_mpc.aligator_problem import KinematicWheeledUR5eProblemBuilder
    print("  ✓ Problem Builder")

    print("\n[2/6] 创建机器人模型...")
    robot = WheeledUR5eModel()
    print(f"  ✓ 10-DOF, q_nominal shape: {robot.q_nominal.shape}")

    print("\n[3/6] 创建Problem Builder...")
    builder = KinematicWheeledUR5eProblemBuilder(
        robot=robot,
        horizon=5,  # 小horizon
        dt=0.05
    )
    print("  ✓ Builder创建成功")

    print("\n[4/6] 准备参考轨迹...")
    q0 = robot.q_nominal.copy()
    ee_start = robot.fk_numpy(q0)

    ref_traj = {
        'ee_pos': np.tile(ee_start, (6, 1)),  # horizon+1 = 6
        'base': np.zeros((6, 3)),
        'base_z': np.full(6, 0.2)
    }
    print(f"  ✓ 参考轨迹: ee_pos shape {ref_traj['ee_pos'].shape}")

    print("\n[5/6] 构建OCP问题...")
    problem, costs = builder.build_problem(q0, ref_traj, u_prev=None)
    print("  ✓ OCP构建成功")
    print(f"    Stages: {len(problem.stages)}")
    print(f"    Terminal cost: {problem.term_cost is not None}")

    print("\n[6/6] 创建并运行求解器...")
    solver = aligator.SolverProxDDP(
        tol=1e-4,
        mu_init=1e-4,
        max_iters=3,  # 只运行3次迭代
    )
    print("  ✓ Solver创建成功")

    print("\n    初始化warm start...")
    xs_init = [q0.copy() for _ in range(6)]
    us_init = [np.zeros(robot.nu) for _ in range(5)]

    print("    Setup solver...")
    solver.setup(problem)
    print("    ✓ Setup完成")

    print("    运行求解器...")
    solver.run(problem, xs_init, us_init)
    print("    ✓ Solver运行成功!")

    res = solver.results
    print(f"\n结果:")
    print(f"  收敛: {res.conv}")
    print(f"  迭代次数: {res.num_iters}")
    print(f"  Cost: {res.traj_cost:.6f}")
    print(f"  xs shape: {np.array(res.xs).shape}")
    print(f"  us shape: {np.array(res.us).shape}")

    print("\n" + "="*80)
    print("✅ 所有测试通过！ALIGATOR绑定正常工作")
    print("="*80)

except Exception as e:
    print(f"\n❌ 错误发生在上述步骤:")
    print(f"\n错误类型: {type(e).__name__}")
    print(f"错误信息: {e}")
    print("\n详细堆栈:")
    traceback.print_exc()

    print("\n" + "="*80)
    print("诊断建议:")
    print("="*80)
    if "boost::python::error_already_set" in str(type(e)):
        print("- C++ Python绑定异常")
        print("- 可能原因: cost function的__reduce__方法有问题")
        print("- 或者: space对象的deepcopy失败")
    sys.exit(1)
