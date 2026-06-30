#!/usr/bin/env python3
"""
最小化诊断：测试坐标映射是否正确
"""

import sys
sys.path.insert(0, '.')

import numpy as np
import mujoco

from wheeled_ur5e_aligator_mpc.coordinate_mapping import q_to_ctrl, ctrl_to_q

# 加载模型
mjcf_path = 'assets/wheeled_dual_ur5e_v2_torque.xml'
model = mujoco.MjModel.from_xml_path(mjcf_path)
data = mujoco.MjData(model)

print("=" * 60)
print("坐标映射诊断")
print("=" * 60)

# 测试映射函数
q_order = np.arange(16)
ctrl_order = q_to_ctrl(q_order)
q_back = ctrl_to_q(ctrl_order)

print(f"\nqpos 顺序: {q_order}")
print(f"ctrl 顺序: {ctrl_order}")
print(f"逆映射回: {q_back}")
print(f"映射正确: {np.allclose(q_order, q_back)}")

# Base 顺序测试
print(f"\nBase 映射测试:")
print(f"  qpos base [0,1,2,3] -> ctrl base [{ctrl_order[0]},{ctrl_order[1]},{ctrl_order[2]},{ctrl_order[3]}]")
print(f"  预期: [0, 1, 3, 2] (yaw 和 z 互换)")

# 设置 nominal 配置
q_nominal = np.array([
    0.0, 0.0, 0.0, 0.2,  # base: x, y, yaw, z
    -2.5434, -0.6884,  1.6850, 0.4209, -1.3484,  0.0000,  # left
     1.4529, -0.7472,  2.3605, 0.3727, -1.9646,  0.0000,  # right
])

print(f"\nNominal qpos:")
print(f"  base: {q_nominal[:4]}")
print(f"  应为: [0.0, 0.0, 0.0, 0.2] (z=0.2)")

# 应用到 MuJoCo
data.qpos[:16] = q_nominal
mujoco.mj_forward(model, data)

print(f"\nMuJoCo 读取的 qpos:")
print(f"  base: {data.qpos[:4]}")

# 测试力矩映射
tau_q_order = np.array([10, 20, 30, 40,  # base
                        1, 2, 3, 4, 5, 6,  # left
                        7, 8, 9, 10, 11, 12])  # right

tau_ctrl_order = q_to_ctrl(tau_q_order)

print(f"\n力矩映射测试:")
print(f"  qpos 顺序 tau: {tau_q_order[:4]} (base)")
print(f"  ctrl 顺序 tau: {tau_ctrl_order[:4]} (base)")
print(f"  预期: [10, 20, 40, 30] (yaw=30 和 z=40 互换)")

print("\n" + "=" * 60)
print("✓ 坐标映射诊断完成")
print("=" * 60)
