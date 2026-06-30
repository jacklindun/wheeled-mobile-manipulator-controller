#!/usr/bin/env python3
"""
详细诊断ALIGATOR C++绑定问题 - 逐步测试每个组件
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
print("ALIGATOR C++绑定详细诊断")
print("="*80)

step = 0

def test_step(description):
    global step
    step += 1
    print(f"\n[步骤 {step}] {description}")
    return step

try:
    test_step("导入ALIGATOR")
    import aligator
    print(f"  ✓ ALIGATOR {aligator.__version__}")

    test_step("导入机器人模型")
    from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel
    robot = WheeledUR5eModel()
    print(f"  ✓ Robot: {robot.nq}-DOF")

    test_step("创建VectorSpace")
    space = aligator.manifolds.VectorSpace(robot.nq)
    print(f"  ✓ Space: nx={space.nx}, ndx={space.ndx}")

    test_step("创建自定义dynamics")
    from wheeled_ur5e_aligator_mpc.aligator_problem import WheeledUR5eKinDynamics
    dt = 0.05
    dynamics = WheeledUR5eKinDynamics(space, robot, dt)
    print(f"  ✓ Dynamics创建成功")

    test_step("测试dynamics.forward()")
    x_test = robot.q_nominal.copy()
    u_test = np.zeros(robot.nu)
    dyn_data = dynamics.createData()
    dynamics.forward(x_test, u_test, dyn_data)
    print(f"  ✓ forward() 执行成功")
    print(f"    xnext: {dyn_data.xnext[:3]}...")

    test_step("测试dynamics.dForward()")
    dynamics.dForward(x_test, u_test, dyn_data)
    print(f"  ✓ dForward() 执行成功")
    print(f"    Jx shape: {dyn_data.Jx.shape}")
    print(f"    Ju shape: {dyn_data.Ju.shape}")

    test_step("创建自定义EEPosCost")
    from wheeled_ur5e_aligator_mpc.aligator_problem import EEPosCost
    ee_ref = robot.fk_numpy(robot.q_nominal)
    ee_cost = EEPosCost(space, robot.nu, robot, weight=100.0, p_ref=ee_ref)
    print(f"  ✓ EEPosCost创建成功")

    test_step("测试EEPosCost.evaluate()")
    cost_data = ee_cost.createData()
    ee_cost.evaluate(x_test, u_test, cost_data)
    print(f"  ✓ evaluate() 执行成功")
    print(f"    cost value: {cost_data.value:.6f}")

    test_step("测试EEPosCost.computeGradients()")
    ee_cost.computeGradients(x_test, u_test, cost_data)
    print(f"  ✓ computeGradients() 执行成功")
    print(f"    Lx: {cost_data.Lx[:3]}...")

    test_step("测试EEPosCost.computeHessians()")
    ee_cost.computeHessians(x_test, u_test, cost_data)
    print(f"  ✓ computeHessians() 执行成功")
    print(f"    Lxx shape: {cost_data.Lxx.shape}")

    test_step("创建CostStack并添加成本")
    cost_stack = aligator.CostStack(space, robot.nu)
    print(f"  ✓ CostStack创建成功")

    # 测试添加不同类型的成本
    print("  测试添加QuadraticStateCost...")
    q_target = robot.q_nominal.copy()
    W_q = np.diag(np.ones(robot.nq) * 0.1)  # 必须是矩阵!
    quad_cost = aligator.QuadraticStateCost(space, robot.nu, q_target, W_q)
    cost_stack.addCost(quad_cost)
    print("    ✓ QuadraticStateCost添加成功")

    print("  测试添加QuadraticControlCost...")
    u_target = np.zeros(robot.nu)
    W_u = np.eye(robot.nu) * 0.01  # 必须是矩阵!
    ctrl_cost = aligator.QuadraticControlCost(space, u_target, W_u)
    cost_stack.addCost(ctrl_cost)
    print("    ✓ QuadraticControlCost添加成功")

    print("  测试添加自定义EEPosCost...")
    cost_stack.addCost(ee_cost)
    print("    ✓ EEPosCost添加成功")

    test_step("创建StageModel")
    stage = aligator.StageModel(cost_stack, dynamics)
    print(f"  ✓ StageModel创建成功")

    test_step("添加控制约束")
    ctrl_res = aligator.ControlErrorResidual(space.ndx, np.zeros(robot.nu))
    box_cstr = aligator.constraints.BoxConstraint(robot.u_min, robot.u_max)
    stage.addConstraint(ctrl_res, box_cstr)
    print(f"  ✓ 约束添加成功")

    test_step("创建TrajOptProblem")
    x0 = robot.q_nominal.copy()

    # 创建terminal cost
    term_cost_stack = aligator.CostStack(space, robot.nu)
    term_quad = aligator.QuadraticStateCost(space, robot.nu, q_target, W_q * 10)  # W_q已经是矩阵
    term_cost_stack.addCost(term_quad)
    term_ee_cost = EEPosCost(space, robot.nu, robot, weight=1000.0, p_ref=ee_ref)
    term_cost_stack.addCost(term_ee_cost)

    problem = aligator.TrajOptProblem(x0, robot.nu, space, term_cost_stack)
    print(f"  ✓ TrajOptProblem创建成功")

    test_step("添加stages到problem")
    N = 5
    for k in range(N):
        problem.addStage(stage)
    print(f"  ✓ 添加了{N}个stages")

    test_step("创建SolverProxDDP")
    solver = aligator.SolverProxDDP(
        tol=1e-4,
        mu_init=1e-4,
        max_iters=3,
    )
    print(f"  ✓ Solver创建成功")

    test_step("Solver.setup()")
    solver.setup(problem)
    print(f"  ✓ setup()成功")

    test_step("准备warm start")
    xs_init = [x0.copy() for _ in range(N+1)]
    us_init = [np.zeros(robot.nu) for _ in range(N)]
    print(f"  ✓ warm start准备完成")

    test_step("Solver.run() - 关键测试！")
    print("  正在运行求解器...")
    solver.run(problem, xs_init, us_init)
    print(f"  ✓ Solver.run()成功!")

    res = solver.results
    print(f"\n求解结果:")
    print(f"  收敛: {res.conv}")
    print(f"  迭代: {res.num_iters}")
    print(f"  Cost: {res.traj_cost:.6f}")

    print("\n" + "="*80)
    print("✅ 所有测试通过！ALIGATOR绑定工作正常")
    print("="*80)

except Exception as e:
    print(f"\n" + "="*80)
    print(f"❌ 错误发生在步骤 {step}")
    print("="*80)
    print(f"\n错误类型: {type(e).__name__}")
    print(f"错误信息: {e}")
    print("\n完整堆栈:")
    traceback.print_exc()

    print("\n" + "="*80)
    print("诊断建议:")
    print("="*80)

    if "boost::python::error_already_set" in str(type(e)):
        print("- C++ Python绑定异常")
        print("- 可能是CostStack.addCost()时的deepcopy问题")
        print("- 检查自定义cost的__reduce__方法")
    elif "deepcopy" in str(e).lower():
        print("- deepcopy失败")
        print("- ALIGATOR内部会对cost对象进行deepcopy")
        print("- 自定义类的__reduce__方法可能有问题")

    sys.exit(1)
