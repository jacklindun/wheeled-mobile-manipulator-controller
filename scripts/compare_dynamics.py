#!/usr/bin/env python3
"""
详细对比ALIGATOR动力学预测 vs MuJoCo执行

逐项分析差异来源：
1. 重力项
2. 科氏力/离心力
3. 阻尼
4. Armature
5. 积分方法
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
import mujoco
import pinocchio as pin

from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel
from wheeled_ur5e_aligator_mpc.pinocchio_model import PinocchioWheeledUR5eModel
from wheeled_ur5e_aligator_mpc.mujoco_env_hybrid import MujocoWheeledUR5eHybridEnv


def compare_dynamics_detailed():
    """详细对比动力学计算的每一项"""

    robot = WheeledUR5eModel()
    pin_robot = PinocchioWheeledUR5eModel()
    env = MujocoWheeledUR5eHybridEnv(render=False)

    # 测试配置：q_nominal, 小速度, 小扭矩
    q_test = robot.q_nominal.copy()
    v_test = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])  # 只有第一个关节有速度
    tau_test = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # 零扭矩，看重力

    print("="*80)
    print("详细动力学对比")
    print("="*80)
    print(f"\n测试配置:")
    print(f"  q_arm = {q_test[4:]}")
    print(f"  v_arm = {v_test}")
    print(f"  tau_arm = {tau_test}")

    # ========== Pinocchio 计算 ==========
    print(f"\n{'='*80}")
    print("Pinocchio 动力学分析")
    print("="*80)

    arm_model = pin_robot.arm_model
    arm_data = pin_robot.arm_data
    q_arm = q_test[4:]

    # 计算质量矩阵
    pin.crba(arm_model, arm_data, q_arm)
    M = arm_data.M.copy()
    print(f"\n质量矩阵 M 对角线: {np.diag(M)}")

    # 重力项
    pin.computeGeneralizedGravity(arm_model, arm_data, q_arm)
    g = arm_data.g.copy()
    print(f"重力项 g: {g}")

    # 科氏力/离心力
    pin.computeCoriolisMatrix(arm_model, arm_data, q_arm, v_test)
    C = arm_data.C.copy()
    c_force = C @ v_test
    print(f"科氏/离心力 C*v: {c_force}")

    # 阻尼
    damping = np.array([1.0, 1.0, 0.5, 0.1, 0.1, 0.1])
    damping_force = damping * v_test
    print(f"阻尼力 D*v: {damping_force}")

    # Armature
    armature = np.array([0.1, 0.1, 0.1, 0.01, 0.01, 0.01])
    M_eff = M + np.diag(armature)
    print(f"\nArmature: {armature}")
    print(f"有效质量矩阵 (M+A) 对角线: {np.diag(M_eff)}")

    # 总动力学： (M + A) * a = tau - C*v - g - D*v
    rhs = tau_test - c_force - g - damping_force
    print(f"\n右端项 (tau - C*v - g - D*v): {rhs}")

    a_pin = np.linalg.solve(M_eff, rhs)
    print(f"加速度 a (Pinocchio): {a_pin}")

    # 积分（semi-implicit Euler）
    dt = 0.05
    v_next_pin = v_test + dt * a_pin
    q_next_pin = q_arm + dt * v_next_pin
    print(f"\nSemi-implicit Euler 积分 (dt={dt}):")
    print(f"  v_next: {v_next_pin}")
    print(f"  q_next: {q_next_pin}")

    # ========== MuJoCo 计算 ==========
    print(f"\n{'='*80}")
    print("MuJoCo 动力学执行")
    print("="*80)

    # 重置到测试状态
    env.reset(q_test)
    for i, jname in enumerate(["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]):
        env.data.qvel[env._joint_qvel_adr[jname]] = v_test[i]

    # 应用控制
    u_mujoco = np.zeros(10)
    u_mujoco[4:10] = tau_test
    env.set_control(u_mujoco)

    # 单步前向（获取加速度）
    mujoco.mj_forward(env.model, env.data)

    # 获取加速度（在应用控制后）
    a_mujoco_before = np.array([env.data.qacc[env._joint_qvel_adr[jn]]
                                 for jn in ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                           "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]])
    print(f"加速度 a (MuJoCo forward): {a_mujoco_before}")

    # 执行积分
    substeps = int(dt / env.model.opt.timestep)
    print(f"\nMuJoCo积分 (implicitfast, {substeps} substeps):")
    env.step(substeps)

    # 获取结果
    q_next_mujoco = env.get_q()[4:]
    v_next_mujoco = env.get_state()[10:16]

    print(f"  v_next: {v_next_mujoco}")
    print(f"  q_next: {q_next_mujoco}")

    # ========== 对比 ==========
    print(f"\n{'='*80}")
    print("差异分析")
    print("="*80)

    print(f"\n加速度差异:")
    a_diff = a_mujoco_before - a_pin
    print(f"  Δa = {a_diff}")
    print(f"  |Δa| = {np.linalg.norm(a_diff):.6f}")

    print(f"\n速度差异:")
    v_diff = v_next_mujoco - v_next_pin
    print(f"  Δv = {v_diff}")
    print(f"  |Δv| = {np.linalg.norm(v_diff):.6f}")

    print(f"\n位置差异:")
    q_diff = q_next_mujoco - q_next_pin
    print(f"  Δq = {q_diff}")
    print(f"  |Δq| = {np.linalg.norm(q_diff):.6f}")

    env.close()

    # 判断
    print(f"\n{'='*80}")
    print("结论")
    print("="*80)

    if np.linalg.norm(a_diff) < 0.1:
        print("✓ 加速度计算匹配良好")
    else:
        print("✗ 加速度计算有显著差异")
        print("  可能原因:")
        print("  - 质量/惯性参数不一致")
        print("  - 科氏力计算方法不同")
        print("  - Armature实现方式不同")

    if np.linalg.norm(v_diff) > 0.01:
        print("✗ 速度积分有差异")
        print("  可能原因:")
        print("  - 积分器不同 (semi-implicit vs implicitfast)")
        print("  - 需要匹配MuJoCo的积分方法")

    if np.linalg.norm(q_diff) > 0.001:
        print("✗ 位置积分有差异")


def test_zero_velocity_gravity():
    """测试零速度下的重力影响（最简单情况）"""
    print("\n" + "="*80)
    print("测试：零速度下的重力")
    print("="*80)

    robot = WheeledUR5eModel()
    pin_robot = PinocchioWheeledUR5eModel()
    env = MujocoWheeledUR5eHybridEnv(render=False)

    q_test = robot.q_nominal.copy()
    v_zero = np.zeros(6)
    tau_zero = np.zeros(6)

    # Pinocchio
    arm_model = pin_robot.arm_model
    arm_data = pin_robot.arm_data
    q_arm = q_test[4:]

    pin.crba(arm_model, arm_data, q_arm)
    M = arm_data.M
    pin.computeGeneralizedGravity(arm_model, arm_data, q_arm)
    g = arm_data.g

    armature = np.array([0.1, 0.1, 0.1, 0.01, 0.01, 0.01])
    M_eff = M + np.diag(armature)

    a_pin = np.linalg.solve(M_eff, -g)

    print(f"\nPinocchio (零速度, 零扭矩):")
    print(f"  重力加速度: {a_pin}")

    # MuJoCo
    env.reset(q_test)
    # 零速度已经是reset的默认状态
    u_zero = np.zeros(10)
    env.set_control(u_zero)

    mujoco.mj_forward(env.model, env.data)
    a_mujoco = np.array([env.data.qacc[env._joint_qvel_adr[jn]]
                         for jn in ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                   "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]])

    print(f"MuJoCo (零速度, 零扭矩):")
    print(f"  重力加速度: {a_mujoco}")

    diff = a_mujoco - a_pin
    print(f"\n差异:")
    print(f"  Δa = {diff}")
    print(f"  |Δa| = {np.linalg.norm(diff):.6f}")

    env.close()

    if np.linalg.norm(diff) < 0.1:
        print("\n✓ 重力项匹配")
    else:
        print("\n✗ 重力项不匹配 - 模型参数（质量/惯性）可能不一致")


if __name__ == "__main__":
    test_zero_velocity_gravity()
    print("\n" + "="*80 + "\n")
    compare_dynamics_detailed()
