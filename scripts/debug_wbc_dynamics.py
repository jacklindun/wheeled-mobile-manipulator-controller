#!/usr/bin/env python3
"""
调试Phase 6动力学残差问题

检查：
1. MPC计算的加速度是否正确传递给WBC
2. WBC的动力学方程是否正确
3. 为什么残差是83.25
"""

import sys
from pathlib import Path
_repo_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_repo_root))

import numpy as np
from wheeled_ur5e_aligator_mpc.pinocchio_model import PinocchioWheeledUR5eModel
from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel
from wheeled_ur5e_aligator_mpc.wbc_controller import WholeBodyController
from wheeled_ur5e_aligator_mpc.wheeled_dynamics import WheelParameters

print("="*80)
print("调试WBC动力学残差问题")
print("="*80)

# 1. 初始化
robot = WheeledUR5eModel()
pin_robot = PinocchioWheeledUR5eModel()
wheel_params = WheelParameters()
wbc = WholeBodyController(pin_robot, wheel_params)

# 2. 创建测试状态
x_wbc = np.zeros(23)
x_wbc[0:4] = [0.0, 0.0, 0.2, 0.0]  # q_base
x_wbc[6:12] = robot.q_nominal[4:10]  # q_arm

print("\n1. 测试零加速度（期望静止）")
a_des_zero = np.zeros(11)
τ_opt_zero, info_zero = wbc.compute_control(x_wbc, a_des_zero)

print(f"   期望加速度: {a_des_zero}")
print(f"   输出扭矩范围: [{np.min(τ_opt_zero):.2f}, {np.max(τ_opt_zero):.2f}]")
print(f"   动力学残差: {info_zero['dynamics_residual']:.6f}")
print(f"   问题: 残差 = {info_zero['dynamics_residual']:.2f} (应该≈0)")

# 3. 测试非零加速度
print("\n2. 测试非零加速度（模拟重力补偿）")
a_des_gravity = np.zeros(11)
a_des_gravity[5:11] = [0.0, -9.81, 0.0, 0.0, 0.0, 0.0]  # 机械臂垂直方向重力

τ_opt_grav, info_grav = wbc.compute_control(x_wbc, a_des_gravity)

print(f"   期望加速度（机械臂）: {a_des_gravity[5:11]}")
print(f"   输出扭矩（机械臂）: {τ_opt_grav[2:8]}")
print(f"   动力学残差: {info_grav['dynamics_residual']:.6f}")

# 4. 检查WBC内部计算
print("\n3. 检查WBC内部动力学计算")
q = x_wbc[:12]
v = x_wbc[12:]

# 调用WBC的内部方法
M, h = wbc._compute_dynamics(q, v)

print(f"   质量矩阵 M 形状: {M.shape}")
print(f"   非线性项 h 形状: {h.shape}")
print(f"   M 对角线: {np.diag(M)}")
print(f"   h (非线性项): {h}")
print(f"   h 范数: {np.linalg.norm(h):.6f}")

# 5. 检查S矩阵
S = wbc._S_matrix()
print(f"\n4. 选择矩阵 S 形状: {S.shape}")
print(f"   S 秩: {np.linalg.matrix_rank(S)}")

# 6. 手动计算残差
print("\n5. 手动验证动力学残差计算")
# 使用零加速度和零扭矩
a_test = np.zeros(11)
τ_test = np.zeros(8)

residual_manual = np.linalg.norm(M @ a_test + h - S @ τ_test)
print(f"   M @ a + h - S @ τ 的范数: {residual_manual:.6f}")
print(f"   这应该等于 ||h||: {np.linalg.norm(h):.6f}")
print(f"   结论: WBC残差 ≈ ||h|| 因为 a=0, τ=0")

# 7. 分析h的来源
print("\n6. 分析非线性项 h 的来源")
print(f"   基座部分 h[0:3]: {h[0:3]} (应该≈0)")
print(f"   轮子部分 h[3:5]: {h[3:5]}")
print(f"   机械臂部分 h[5:11]: {h[5:11]}")
print(f"   机械臂 h 范数: {np.linalg.norm(h[5:11]):.6f}")

# 8. 结论
print("\n" + "="*80)
print("结论")
print("="*80)
print(f"\n问题根源: WBC的动力学残差 = ||M*a + h - S^T*τ||")
print(f"当 a_des=0, τ=0 时, 残差 = ||h|| ≈ {np.linalg.norm(h):.2f}")
print(f"\n这是因为:")
print(f"  1. 机械臂在nominal posture下受重力影响")
print(f"  2. h 包含重力项，norm ≈ 83")
print(f"  3. 如果MPC提供的a_des也是0，WBC会尝试用τ抵消h")
print(f"  4. 但如果QP权重不对，可能无法完全抵消")

print(f"\n解决方案:")
print(f"  ✓ 确保MPC提供准确的加速度（包含重力效应）")
print(f"  ✓ 调整WBC权重，提高动力学一致性权重")
print(f"  ✓ 检查QP是否正确求解")

print("="*80)
