#!/usr/bin/env python3
"""
检查机器人当前构形（无GUI版本）

显示：
1. 当前关节角度
2. EE位置
3. 机器人姿态信息
"""

import sys
from pathlib import Path
_repo_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_repo_root))

import numpy as np
import mujoco

from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel
from wheeled_ur5e_aligator_mpc.pinocchio_model import PinocchioWheeledUR5eModel

print("="*80)
print("检查机器人构形（文本模式）")
print("="*80)

# 1. 加载模型
mjcf_path = Path(__file__).parent.parent / "assets" / "wheeled_ur5e_wheels.xml"
m = mujoco.MjModel.from_xml_path(str(mjcf_path))
d = mujoco.MjData(m)

robot = WheeledUR5eModel()
pin_robot = PinocchioWheeledUR5eModel()

print(f"\n1. MuJoCo模型:")
print(f"   DOF: {m.nq}")
print(f"   控制器数: {m.nu}")
print(f"   关节列表:")
for i in range(m.njnt):
    print(f"     {i}: {m.joint(i).name}")

print(f"\n   执行器列表:")
for i in range(m.nu):
    print(f"     {i}: {m.actuator(i).name}")

# 2. 设置到nominal配置
print(f"\n2. 设置到nominal配置...")
mujoco.mj_resetData(m, d)
d.qpos[0:4] = robot.q_nominal[0:4]    # base
d.qpos[4:6] = [0.0, 0.0]              # wheels
d.qpos[6:12] = robot.q_nominal[4:10]  # arm
d.qvel[:] = 0.0
mujoco.mj_forward(m, d)

print(f"\n3. 当前关节状态:")
print(f"   基座位置 [x, y, z, yaw]:")
print(f"     qpos[0:4] = {d.qpos[0:4]}")
print(f"\n   轮子角度 [left, right]:")
print(f"     qpos[4:6] = {d.qpos[4:6]} rad")
print(f"\n   机械臂关节角度:")
joint_names = ['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3']
for i, name in enumerate(joint_names):
    rad = d.qpos[6+i]
    deg = np.rad2deg(rad)
    print(f"     {name:15s}: {rad:7.3f} rad = {deg:7.2f} deg")

# 3. 检查EE位置
ee_site_id = m.site("ee_site").id
p_ee_mujoco = d.site_xpos[ee_site_id]
R_ee_mujoco = d.site_xmat[ee_site_id].reshape(3, 3)

print(f"\n4. 末端执行器状态:")
print(f"   MuJoCo EE位置: [{p_ee_mujoco[0]:.4f}, {p_ee_mujoco[1]:.4f}, {p_ee_mujoco[2]:.4f}]")
print(f"   MuJoCo EE姿态 (旋转矩阵):")
for i in range(3):
    print(f"     {R_ee_mujoco[i]}")

# 用Pinocchio计算FK验证
# robot.q_nominal是10维: [base(4), arm(6)]
# Pinocchio wheels模型需要12维: [base(4), wheels(2), arm(6)]
q_nominal_12d = np.zeros(12)
q_nominal_12d[0:4] = robot.q_nominal[0:4]  # base
q_nominal_12d[4:6] = [0.0, 0.0]  # wheels
q_nominal_12d[6:12] = robot.q_nominal[4:10]  # arm
p_ee_pin, R_ee_pin = pin_robot.fk_pose(q_nominal_12d)
print(f"\n   Pinocchio EE位置: [{p_ee_pin[0]:.4f}, {p_ee_pin[1]:.4f}, {p_ee_pin[2]:.4f}]")
print(f"   位置差异: {np.linalg.norm(p_ee_mujoco - p_ee_pin)*1000:.3f} mm")

if np.linalg.norm(p_ee_mujoco - p_ee_pin) > 0.01:
    print(f"   ⚠️ 警告: MuJoCo和Pinocchio的FK结果不一致！")
else:
    print(f"   ✅ MuJoCo和Pinocchio的FK结果一致")

# 4. 检查是否有明显问题
print(f"\n5. 构形健康检查:")
issues = []

# 基座高度检查
if d.qpos[3] < 0.15 or d.qpos[3] > 0.3:
    issues.append(f"基座高度异常: {d.qpos[3]:.3f}m (期望约0.2m)")
else:
    print(f"   ✅ 基座高度正常: {d.qpos[3]:.3f}m")

# EE高度检查
if p_ee_mujoco[2] < 0.5 or p_ee_mujoco[2] > 1.2:
    issues.append(f"EE高度异常: {p_ee_mujoco[2]:.3f}m (期望0.5-1.2m)")
else:
    print(f"   ✅ EE高度正常: {p_ee_mujoco[2]:.3f}m")

# EE前伸距离检查
ee_reach = np.sqrt(p_ee_mujoco[0]**2 + p_ee_mujoco[1]**2)
if ee_reach < 0.3 or ee_reach > 1.0:
    issues.append(f"EE前伸距离异常: {ee_reach:.3f}m (期望0.3-1.0m)")
else:
    print(f"   ✅ EE前伸距离正常: {ee_reach:.3f}m")

# 关节限制检查
print(f"\n6. 关节限制检查:")
for i in range(6):
    joint_name = f"{joint_names[i]}_joint"
    joint_id = m.joint(joint_name).id
    q_val = d.qpos[6+i]
    q_min, q_max = m.jnt_range[joint_id]

    within_limit = q_min <= q_val <= q_max
    status = "✅" if within_limit else "❌"
    print(f"   {status} {joint_names[i]:15s}: {q_val:7.3f} ∈ [{q_min:7.3f}, {q_max:7.3f}]")

    if not within_limit:
        issues.append(f"{joint_names[i]}超出限制: {q_val:.3f} not in [{q_min:.3f}, {q_max:.3f}]")

# 7. 总结
print(f"\n" + "="*80)
print("总结")
print("="*80)

if issues:
    print(f"\n⚠️ 发现 {len(issues)} 个问题:")
    for issue in issues:
        print(f"   - {issue}")
else:
    print(f"\n✅ 机器人构形完全正常！")
    print(f"\n关键参数:")
    print(f"   基座: (x={d.qpos[0]:.2f}, y={d.qpos[1]:.2f}, z={d.qpos[3]:.2f}, yaw={d.qpos[2]:.2f})")
    print(f"   EE位置: ({p_ee_mujoco[0]:.3f}, {p_ee_mujoco[1]:.3f}, {p_ee_mujoco[2]:.3f})")
    print(f"   EE工作空间: 前伸{ee_reach:.3f}m, 高度{p_ee_mujoco[2]:.3f}m")

print("="*80)
