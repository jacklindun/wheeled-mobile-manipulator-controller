#!/usr/bin/env python3
"""测试 ALIGATOR API 的数组类型要求"""

import numpy as np
import pinocchio as pin
import aligator

# 加载模型
model_path = "assets/wheeled_dual_ur5e_v2.xml"
model = pin.buildModelFromMJCF(model_path)
data = model.createData()

# 创建 space 和 dynamics
space = aligator.manifolds.MultibodyPhaseSpace(model)
ode = aligator.dynamics.MultibodyFreeFwdDynamics(space)
dt = 0.05
dyn_model = aligator.dynamics.IntegratorSemiImplEuler(ode, dt)

# 初始状态
q0 = pin.neutral(model)
v0 = np.zeros(model.nv)
x0 = np.concatenate([q0, v0])

print(f"x0 shape: {x0.shape}, dtype: {x0.dtype}")
print(f"x0 flags: C={x0.flags['C_CONTIGUOUS']}, F={x0.flags['F_CONTIGUOUS']}")

# 测试不同的初始化方式
horizon = 5

print("\n=== 测试 1: 使用 .copy() ===")
xs_init1 = [x0.copy() for _ in range(horizon + 1)]
us_init1 = [np.zeros(model.nv) for _ in range(horizon)]
print(f"xs_init1[0] flags: C={xs_init1[0].flags['C_CONTIGUOUS']}")
print(f"us_init1[0] flags: C={us_init1[0].flags['C_CONTIGUOUS']}")

print("\n=== 测试 2: 使用 np.array() ===")
xs_init2 = [np.array(x0.copy()) for _ in range(horizon + 1)]
us_init2 = [np.array(np.zeros(model.nv)) for _ in range(horizon)]
print(f"xs_init2[0] flags: C={xs_init2[0].flags['C_CONTIGUOUS']}")
print(f"us_init2[0] flags: C={us_init2[0].flags['C_CONTIGUOUS']}")

print("\n=== 测试 3: 使用 np.ascontiguousarray() ===")
xs_init3 = [np.ascontiguousarray(x0.copy()) for _ in range(horizon + 1)]
us_init3 = [np.ascontiguousarray(np.zeros(model.nv)) for _ in range(horizon)]
print(f"xs_init3[0] flags: C={xs_init3[0].flags['C_CONTIGUOUS']}")
print(f"us_init3[0] flags: C={us_init3[0].flags['C_CONTIGUOUS']}")

# 创建简单的问题
print("\n=== 构建问题 ===")
rcost = aligator.CostStack(space, model.nv)
term_cost = aligator.CostStack(space, model.nv)

stages = []
for _ in range(horizon):
    stage = aligator.StageModel(rcost, dyn_model)
    stages.append(stage)

problem = aligator.TrajOptProblem(x0, stages, term_cost)

print("✓ 问题构建成功")

# 测试求解器
print("\n=== 测试求解器 ===")
solver = aligator.SolverProxDDP(1e-4, 1e-8)
solver.max_iters = 1
solver.setup(problem)

for i, (xs_init, us_init) in enumerate([(xs_init1, us_init1), (xs_init2, us_init2), (xs_init3, us_init3)], 1):
    print(f"\n测试 {i}:")
    try:
        solver.run(problem, xs_init, us_init)
        print(f"  ✓ 成功!")
    except Exception as e:
        print(f"  ✗ 失败: {e}")
