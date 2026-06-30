#!/usr/bin/env python3
"""
Phase 6演示：MPC+WBC双层控制

测试完整的MPC+WBC架构：
- MPC层：轨迹规划（简化P控制）
- WBC层：QP力矩求解
- 任务：EE画圆轨迹

Usage:
  python scripts/demo_mpc_wbc.py [--duration 10] [--render]
"""

import sys
from pathlib import Path

_project_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_project_root))

_aligator_root = _project_root.parents[1]
sys.path[:0] = [str(_aligator_root / "build" / "bindings" / "python")]

import numpy as np
import mujoco
import mujoco.viewer

from wheeled_ur5e_aligator_mpc.mpc_wbc_controller import MPCWBCController
from wheeled_ur5e_aligator_mpc.mpc_wbc_interface import create_simple_reference_trajectory
from wheeled_ur5e_aligator_mpc.pinocchio_model import PinocchioWheeledUR5eModel
from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel
from wheeled_ur5e_aligator_mpc.wheeled_dynamics import WheelParameters
from wheeled_ur5e_aligator_mpc.wbc_controller import WBCWeights


def demo_mpc_wbc(duration=10.0, render=True, scenario="ee_circle"):
    """
    MPC+WBC双层控制演示

    Parameters
    ----------
    duration : float
        演示时长（秒）
    render : bool
        是否显示渲染
    scenario : str
        场景：'ee_circle', 'stationary'
    """
    print("="*80)
    print("Phase 6: MPC+WBC双层控制演示")
    print("="*80)
    print(f"\n配置:")
    print(f"  场景:         {scenario}")
    print(f"  时长:         {duration} s")
    print(f"  MPC频率:      20 Hz (0.05s)")
    print(f"  WBC频率:      100 Hz (0.01s)")
    print(f"  任务:         EE轨迹跟踪")
    print("="*80 + "\n")

    # 初始化
    robot = WheeledUR5eModel()
    pin_robot = PinocchioWheeledUR5eModel()
    wheel_params = WheelParameters()
    wbc_weights = WBCWeights()

    # 创建MPC+WBC控制器
    controller = MPCWBCController(pin_robot, wheel_params, wbc_weights)

    # 创建参考轨迹
    ref_traj = create_simple_reference_trajectory(scenario=scenario, duration=duration, dt=0.05)

    # 加载MuJoCo环境
    mjcf_path = _project_root / "assets" / "wheeled_ur5e_wheels.xml"
    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    data = mujoco.MjData(model)

    # 初始化状态
    mujoco.mj_resetData(model, data)
    data.qpos[2] = 0.2  # base_z

    # 设置机械臂名义姿态
    shoulder_pan_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "shoulder_pan_joint")
    data.qpos[shoulder_pan_id] = np.pi
    data.qpos[shoulder_pan_id + 1] = np.pi / 3
    data.qpos[shoulder_pan_id + 2] = -np.pi / 2

    mujoco.mj_forward(model, data)

    # 获取关节索引
    joint_names = ["base_x", "base_y", "base_z", "base_yaw",
                   "left_wheel_joint", "right_wheel_joint",
                   "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                   "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

    actuator_names = ["act_left_wheel", "act_right_wheel",
                      "act_shoulder_pan", "act_shoulder_lift", "act_elbow",
                      "act_wrist_1", "act_wrist_2", "act_wrist_3"]

    # Viewer
    viewer = None
    if render:
        viewer = mujoco.viewer.launch_passive(model, data)

    # 控制循环
    dt_wbc = 0.01
    num_steps = int(duration / dt_wbc)

    print("开始控制循环...")
    print(f"{'时间':<8} {'EE误差':<12} {'WBC时间':<14} {'动力学残差':<16}")
    print("-"*80)

    ee_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")

    for step in range(num_steps):
        t = step * dt_wbc

        # 1. 获取当前状态（构建23-dim WBC状态）
        x_wbc = np.zeros(23)

        # 位置 q(12)
        for i, jname in enumerate(joint_names[:12]):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            if i < 4:  # base
                x_wbc[i] = data.qpos[jid]
            else:  # wheels + arm
                x_wbc[i] = data.qpos[jid]

        # 速度 v(11): [v_base(3), ω_wheels(2), v_arm(6)]
        yaw = x_wbc[3]
        # 基座速度（简化：从qvel推导）
        v_base_x = data.qvel[0]
        v_base_y = data.qvel[1]
        omega_yaw = data.qvel[2]
        x_wbc[12:15] = [v_base_x, v_base_y, omega_yaw]

        # 轮速
        left_wheel_vid = 3  # qvel索引
        right_wheel_vid = 4
        x_wbc[15] = data.qvel[left_wheel_vid]
        x_wbc[16] = data.qvel[right_wheel_vid]

        # 机械臂速度
        x_wbc[17:23] = data.qvel[5:11]

        # 2. MPC+WBC控制
        τ_opt, info = controller.control_step(x_wbc, ref_traj, t)

        # 3. 应用控制
        for i, aname in enumerate(actuator_names):
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, aname)
            data.ctrl[aid] = τ_opt[i]

        # 4. 仿真步进
        mujoco.mj_step(model, data)

        if render and viewer is not None:
            viewer.sync()

        # 5. 计算EE误差
        ee_pos = data.site_xpos[ee_site_id].copy()
        idx = int(t / 0.05)
        if idx >= len(ref_traj["ee_pos"]):
            idx = -1
        ee_ref = ref_traj["ee_pos"][idx]
        ee_err = np.linalg.norm(ee_pos - ee_ref)

        controller.stats["ee_errors"].append(ee_err)

        # 6. 输出（每秒一次）
        if step % 100 == 0 or step == 0:
            wbc_info = info["wbc_info"]
            print(f"{t:>6.1f}s  {ee_err*100:>10.2f}cm  {wbc_info['solve_time_ms']:>12.2f}ms  "
                  f"{wbc_info['dynamics_residual']:>14.6f}")

    if viewer is not None:
        viewer.close()

    # 统计
    controller.print_stats()

    print(f"\n✓ Phase 6 MPC+WBC演示完成！")
    print("="*80)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Phase 6 MPC+WBC演示")
    parser.add_argument("--duration", type=float, default=10.0, help="演示时长(秒)")
    parser.add_argument("--render", action="store_true", help="启用渲染")
    parser.add_argument("--scenario", default="ee_circle",
                        choices=["ee_circle", "stationary"],
                        help="场景选择")
    args = parser.parse_args()

    demo_mpc_wbc(duration=args.duration, render=args.render, scenario=args.scenario)
