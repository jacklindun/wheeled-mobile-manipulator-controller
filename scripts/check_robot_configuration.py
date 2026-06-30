#!/usr/bin/env python3
"""
检查机器人当前构形

显示：
1. 当前关节角度
2. EE位置
3. 机器人姿态可视化
"""

import sys
from pathlib import Path
_repo_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_repo_root))

import numpy as np
import mujoco
import mujoco.viewer

from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel
from wheeled_ur5e_aligator_mpc.pinocchio_model import PinocchioWheeledUR5eModel

print("="*80)
print("检查机器人构形")
print("="*80)

# 1. 加载模型
mjcf_path = Path(__file__).parent.parent / "assets" / "wheeled_ur5e_wheels.xml"
m = mujoco.MjModel.from_xml_path(str(mjcf_path))
d = mujoco.MjData(m)

robot = WheeledUR5eModel()
pin_robot = PinocchioWheeledUR5eModel()

print(f"\n1. MuJoCo模型:")
print(f"   DOF: {m.nq}")
print(f"   关节:")
for i in range(m.njnt):
    print(f"     {i}: {m.joint(i).name}")

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
print(f"     {d.qpos[0:4]}")
print(f"\n   轮子角度 [left, right]:")
print(f"     {d.qpos[4:6]}")
print(f"\n   机械臂关节角度 (rad):")
joint_names = ['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3']
for i, name in enumerate(joint_names):
    print(f"     {name:15s}: {d.qpos[6+i]:7.3f} rad ({np.rad2deg(d.qpos[6+i]):7.2f} deg)")

# 3. 检查EE位置
ee_site_id = m.site("ee_site").id
p_ee_mujoco = d.site_xpos[ee_site_id]

print(f"\n4. 末端执行器位置:")
print(f"   MuJoCo EE位置: {p_ee_mujoco}")

# 用Pinocchio计算FK验证
p_ee_pin, R_ee_pin = pin_robot.fk_pose(robot.q_nominal)
print(f"   Pinocchio EE位置: {p_ee_pin}")
print(f"   差异: {np.linalg.norm(p_ee_mujoco - p_ee_pin):.6f} m")

# 4. 检查是否有明显问题
print(f"\n5. 构形检查:")
issues = []

# 基座高度检查
if d.qpos[3] < 0.15 or d.qpos[3] > 0.3:
    issues.append(f"基座高度异常: {d.qpos[3]:.3f}m (期望0.2m)")

# EE高度检查
if p_ee_mujoco[2] < 0.5 or p_ee_mujoco[2] > 1.2:
    issues.append(f"EE高度异常: {p_ee_mujoco[2]:.3f}m")

# 关节限制检查
for i in range(6):
    joint_id = m.joint(f"{joint_names[i]}_joint").id
    q_val = d.qpos[6+i]
    q_min, q_max = m.jnt_range[joint_id]
    if q_val < q_min or q_val > q_max:
        issues.append(f"{joint_names[i]}超出限制: {q_val:.3f} not in [{q_min:.3f}, {q_max:.3f}]")

if issues:
    print("   ⚠️ 发现问题:")
    for issue in issues:
        print(f"     - {issue}")
else:
    print("   ✅ 构形正常")

# 5. 启动可视化
print(f"\n6. 启动MuJoCo可视化...")
print(f"   关闭窗口退出")
print(f"   按空格键暂停/继续")
print(f"   鼠标拖拽旋转视角")
print("="*80)

# 使用passive viewer查看静态构形
with mujoco.viewer.launch_passive(m, d) as viewer:
    # 设置相机位置
    viewer.cam.azimuth = 45
    viewer.cam.elevation = -20
    viewer.cam.distance = 3.0
    viewer.cam.lookat = [0.5, 0.0, 0.5]

    # 保持窗口打开
    while viewer.is_running():
        mujoco.mj_step(m, d)
        viewer.sync()

print("\n可视化已关闭")
