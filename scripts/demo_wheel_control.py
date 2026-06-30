#!/usr/bin/env python3
"""
Phase 5简单演示：轮子控制

测试场景：
1. 直线前进
2. 原地旋转
3. 圆弧轨迹
4. 非完整约束验证
"""

import sys
from pathlib import Path

_project_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_project_root))

_aligator_root = _project_root.parents[1]
sys.path[:0] = [
    str(_aligator_root / "build" / "bindings" / "python"),
]

import numpy as np
import mujoco
import mujoco.viewer

from wheeled_ur5e_aligator_mpc.wheeled_dynamics import WheelParameters, inverse_diff_drive


def demo_wheel_control(duration=10.0, render=True):
    """
    演示轮子控制

    Parameters
    ----------
    duration : float
        演示时长（秒）
    render : bool
        是否显示渲染
    """
    print("="*80)
    print("Phase 5: 轮子控制演示")
    print("="*80)

    # 加载MJCF
    mjcf_path = _project_root / "assets" / "wheeled_ur5e_wheels.xml"
    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    data = mujoco.MjData(model)

    # 获取关节和执行器索引
    left_wheel_qvel_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "left_wheel_joint")
    right_wheel_qvel_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "right_wheel_joint")

    left_wheel_act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_left_wheel")
    right_wheel_act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_right_wheel")

    base_x_qpos_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_x")
    base_y_qpos_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_y")
    base_yaw_qpos_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_yaw")

    # 初始化
    mujoco.mj_resetData(model, data)
    data.qpos[2] = 0.2  # base_z

    # 设置机械臂名义姿态
    shoulder_pan_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "shoulder_pan_joint")
    data.qpos[shoulder_pan_id] = np.pi
    data.qpos[shoulder_pan_id + 1] = np.pi / 3

    wheel_params = WheelParameters()

    # 启动viewer
    viewer = None
    if render:
        viewer = mujoco.viewer.launch_passive(model, data)

    dt = model.opt.timestep
    num_steps = int(duration / dt)

    print(f"\n控制方案（{duration}秒）：")
    print(f"  0-3s: 直线前进 (v=0.5 m/s)")
    print(f"  3-6s: 原地旋转 (ω=1.0 rad/s)")
    print(f"  6-10s: 圆弧轨迹 (v=0.3 m/s, ω=0.5 rad/s)")
    print()

    # 记录数据
    positions = []

    for step in range(num_steps):
        t = step * dt

        # 根据时间选择控制模式
        if t < 3.0:
            # 直线前进
            v_linear = 0.5
            ω_angular = 0.0
            mode = "直线"
        elif t < 6.0:
            # 原地旋转
            v_linear = 0.0
            ω_angular = 1.0
            mode = "旋转"
        else:
            # 圆弧
            v_linear = 0.3
            ω_angular = 0.5
            mode = "圆弧"

        # 计算目标轮速
        ω_left, ω_right = inverse_diff_drive(v_linear, ω_angular, wheel_params)

        # 简单P控制器：扭矩 = kp * (目标轮速 - 当前轮速)
        kp = 2.0
        ω_left_current = data.qvel[left_wheel_qvel_id]
        ω_right_current = data.qvel[right_wheel_qvel_id]

        τ_left = kp * (ω_left - ω_left_current)
        τ_right = kp * (ω_right - ω_right_current)

        # 限制扭矩
        τ_left = np.clip(τ_left, -10, 10)
        τ_right = np.clip(τ_right, -10, 10)

        # 应用控制
        data.ctrl[left_wheel_act_id] = τ_left
        data.ctrl[right_wheel_act_id] = τ_right

        # 步进仿真
        mujoco.mj_step(model, data)

        if render and viewer is not None:
            viewer.sync()

        # 记录位置
        x = data.qpos[base_x_qpos_id]
        y = data.qpos[base_y_qpos_id]
        yaw = data.qpos[base_yaw_qpos_id]
        positions.append([t, x, y, yaw])

        # 每秒输出一次
        if step % int(1.0 / dt) == 0:
            print(f"  t={t:4.1f}s [{mode:4s}]: pos=({x:+6.3f}, {y:+6.3f}), yaw={np.degrees(yaw):+7.2f}°, "
                  f"ω_wheels=({ω_left_current:+6.2f}, {ω_right_current:+6.2f})")

    if viewer is not None:
        viewer.close()

    # 分析轨迹
    positions = np.array(positions)

    print(f"\n" + "="*80)
    print("轨迹分析")
    print("="*80)

    # 阶段1: 直线
    idx_straight = positions[:, 0] < 3.0
    dx_straight = positions[idx_straight, 1][-1] - positions[idx_straight, 1][0]
    dy_straight = positions[idx_straight, 2][-1] - positions[idx_straight, 2][0]
    print(f"\n直线阶段（0-3s）：")
    print(f"  距离: Δx={dx_straight:.3f}m, Δy={dy_straight:.3f}m")
    print(f"  预期: Δx≈1.5m (0.5m/s × 3s)")
    print(f"  侧向偏移: {abs(dy_straight):.4f}m (应接近0)")

    # 阶段2: 旋转
    idx_rotate = (positions[:, 0] >= 3.0) & (positions[:, 0] < 6.0)
    dyaw_rotate = positions[idx_rotate, 3][-1] - positions[idx_rotate, 3][0]
    dx_rotate = positions[idx_rotate, 1][-1] - positions[idx_rotate, 1][0]
    dy_rotate = positions[idx_rotate, 2][-1] - positions[idx_rotate, 2][0]
    print(f"\n旋转阶段（3-6s）：")
    print(f"  转角: Δyaw={np.degrees(dyaw_rotate):.1f}° (预期≈172°, 1.0rad/s × 3s)")
    print(f"  平移: Δx={abs(dx_rotate):.4f}m, Δy={abs(dy_rotate):.4f}m (应接近0)")

    print(f"\n✓ Phase 5轮子控制演示完成！")
    print("="*80)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Phase 5轮子控制演示")
    parser.add_argument("--duration", type=float, default=10.0, help="演示时长(秒)")
    parser.add_argument("--no-render", action="store_true", help="禁用渲染")
    args = parser.parse_args()

    demo_wheel_control(duration=args.duration, render=not args.no_render)
