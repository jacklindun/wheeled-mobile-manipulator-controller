#!/usr/bin/env python3
"""最小化测试：验证 MultibodyPhaseSpace 的基本用法"""

import numpy as np
import pinocchio as pin
import aligator

# 加载模型
model_path = "assets/wheeled_dual_ur5e_v2_torque.xml"
model = pin.buildModelFromMJCF(model_path)
data = model.createData()

print(f"模型: nq={model.nq}, nv={model.nv}")

# 创建 space 和 dynamics
space = aligator.manifolds.MultibodyPhaseSpace(model)
print(f"Space: nx={space.nx}, ndx={space.ndx}")

ode = aligator.dynamics.MultibodyFreeFwdDynamics(space)
dt = 0.05
dyn_model = aligator.dynamics.IntegratorSemiImplEuler(ode, dt)

# 初始状态
q0 = pin.neutral(model)
v0 = np.zeros(model.nv)
x0 = np.concatenate([q0, v0])
print(f"x0 shape: {x0.shape}")

# 构建简单问题
horizon = 3
nu = model.nv

stages = []
for i in range(horizon):
    rcost = aligator.CostStack(space, nu)
    # 添加简单的控制正则化
    u0 = np.zeros(nu)
    W_u = np.eye(nu) * 0.01
    rcost.addCost(aligator.QuadraticControlCost(space, u0, W_u))

    stage = aligator.StageModel(rcost, dyn_model)
    stages.append(stage)

term_cost = aligator.CostStack(space, nu)
problem = aligator.TrajOptProblem(x0, stages, term_cost)

print(f"✓ Problem 构建成功")
print(f"  阶段数: {len(problem.stages)}")

# 测试求解
print("\n测试求解...")
solver = aligator.SolverProxDDP(1e-4, 1e-8)
solver.max_iters = 5
solver.setup(problem)

print("调用 solver.run(problem, [], [])...")
try:
    solver.run(problem, [], [])
    print(f"✓ 求解成功!")
    print(f"  收敛: {solver.results.conv}")
    print(f"  迭代数: {solver.results.num_iters}")
    print(f"  xs shape: {np.array(solver.results.xs).shape}")
    print(f"  us shape: {np.array(solver.results.us).shape}")
except Exception as e:
    print(f"✗ 求解失败: {e}")
    import traceback
    traceback.print_exc()
