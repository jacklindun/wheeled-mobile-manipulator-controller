#!/usr/bin/env python3
"""
Phase 6-v3 Step1 优化版本 - 修复大误差问题

问题诊断：
1. PD增益太低 (Kp_arm=500 vs Phase 6-v2的1800)
2. 力矩控制需要更高的增益
3. 可能存在模型不匹配

优化策略：
1. 提升PD增益到Phase 6-v2水平
2. 增加重力补偿精度
3. 使用自适应增益（启动阶段）
"""

import sys
import time
import argparse

sys.path.insert(0, ".")
sys.path.insert(0, "../../build/bindings/python")

import mujoco
import numpy as np

from wheeled_ur5e_aligator_mpc.coordinate_mapping import DUAL_ARM_Q_NOMINAL, q_to_ctrl
from wheeled_ur5e_aligator_mpc.dual_arm_pinocchio_model import DualArmPinocchioModel
from wheeled_ur5e_aligator_mpc.feedforward_pd_controller import FeedforwardPDController, FeedforwardPDGains
from wheeled_ur5e_aligator_mpc.phase6_v3_common import (
    CONTROL_DT,
    INTERPOLATION_RATIO,
    MJCF_PIN,
    MJCF_TORQUE,
    DUAL_ARM_TAU_MAX_Q,
    FixedBaseIKPlanner,
    JointInterpolator,
    circle_trajectory,
    compute_gravity_torque,
    ee_tracking_errors,
)


def make_high_gain_pd_controller(t: float, startup_duration: float = 3.0):
    """
    创建高增益PD控制器（借鉴Phase 6-v2的成功经验）

    0-3s: 高增益 (3× Phase 6-v2)
    3-6s: 线性降低
    6s+: 正常增益 (1.5× Phase 6-v2)
    """
    # Phase 6-v2单臂最优增益: Kp_arm=1800
    # Phase 6-v3双臂需要更高（惯性更大）
    base_kp = 2400.0  # 1.33× v2
    base_kd = 240.0

    if t < startup_duration:
        scale = 3.0  # 启动高增益
    elif t < startup_duration + 3.0:
        alpha = (t - startup_duration) / 3.0
        scale = 3.0 - alpha * 2.0  # 3.0 → 1.0
    else:
        scale = 1.0  # 稳态

    gains = FeedforwardPDGains(
        Kp_base_xy=300.0 * scale, Kd_base_xy=60.0 * scale,
        Kp_base_z=2000.0 * scale, Kd_base_z=400.0 * scale,
        Kp_base_yaw=200.0 * scale, Kd_base_yaw=40.0 * scale,
        Kp_arm=base_kp * scale, Kd_arm=base_kd * scale,
    )
    controller = FeedforwardPDController(gains)
    controller.set_control_limits(-DUAL_ARM_TAU_MAX_Q, DUAL_ARM_TAU_MAX_Q)
    return controller


def main(duration: float = 15.0, omega: float = 0.5, radius: float = 0.08,
         use_gravity_ff: bool = True, render: bool = False):
    print("=" * 60)
    print("Phase 6-v3 Step1 优化版本 - 高增益PD")
    print(f"  重力前馈: {use_gravity_ff}")
    print(f"  启动增益: Kp_arm=7200 (0-3s)")
    print(f"  稳态增益: Kp_arm=2400 (6s+)")
    print("=" * 60)

    # 加载模型
    pin_model = DualArmPinocchioModel(mjcf_path=MJCF_PIN)
    mj_model = mujoco.MjModel.from_xml_path(MJCF_TORQUE)
    mj_data = mujoco.MjData(mj_model)

    if render:
        viewer = mujoco.viewer.launch_passive(mj_model, mj_data)
    else:
        viewer = None

    # IK规划器和插值器
    ik_planner = FixedBaseIKPlanner(pin_model)
    interpolator = JointInterpolator()

    # 初始化
    mj_data.qpos[:16] = DUAL_ARM_Q_NOMINAL
    interpolator.set_segment(DUAL_ARM_Q_NOMINAL, DUAL_ARM_Q_NOMINAL)

    # 数据记录
    errors_left, errors_right = [], []
    ik_residuals = []
    saturation_count = 0
    total_steps = 0

    t_wall_start = time.time()
    print()

    for step in range(int(duration / CONTROL_DT)):
        t = step * CONTROL_DT
        step_in_mpc = step % INTERPOLATION_RATIO

        # 每个MPC周期更新IK目标
        if step_in_mpc == 0:
            target_left, target_right = circle_trajectory(t, omega=omega, radius=radius)
            q_current = mj_data.qpos[:16].copy()
            q_ik, residual = ik_planner(q_current, target_left, target_right)
            ik_residuals.append(residual)
            interpolator.set_segment(q_current, q_ik)

        # 插值获取期望状态
        q_des, v_des = interpolator.interpolate(step_in_mpc)

        # 计算重力补偿
        tau_ff = compute_gravity_torque(pin_model, q_des) if use_gravity_ff else np.zeros(16)

        # 动态更新PD控制器（自适应增益）
        pd_controller = make_high_gain_pd_controller(t)

        # PD控制
        q_current = mj_data.qpos[:16]
        v_current = mj_data.qvel[:16]
        tau_pd, _ = pd_controller.compute_control(
            q_current, v_current, q_des, v_des, u_feedforward=tau_ff,
        )

        # 应用力矩
        mj_data.ctrl[:] = q_to_ctrl(tau_pd, mj_model)
        mujoco.mj_step(mj_model, mj_data)

        if viewer:
            viewer.sync()

        # 统计
        target_left, target_right = circle_trajectory(t, omega=omega, radius=radius)
        el, er = ee_tracking_errors(pin_model, q_current, target_left, target_right)
        errors_left.append(el)
        errors_right.append(er)

        if np.any(np.abs(tau_pd) > DUAL_ARM_TAU_MAX_Q * 0.95):
            saturation_count += 1
        total_steps += 1

        # 打印进度
        if step % (int(0.2 / CONTROL_DT)) == 0:
            print(f"t={t:.2f}s: Left={el*100:.2f}cm, Right={er*100:.2f}cm, "
                  f"IK={ik_residuals[-1]*100:.2f}cm, Kp={pd_controller.gains.Kp_arm[0]:.0f}")

    if viewer:
        viewer.close()

    wall_time = time.time() - t_wall_start
    rms_left = np.sqrt(np.mean(np.array(errors_left) ** 2))
    rms_right = np.sqrt(np.mean(np.array(errors_right) ** 2))
    rms_avg = (rms_left + rms_right) / 2

    # 分阶段分析
    errors_left = np.array(errors_left)
    errors_right = np.array(errors_right)
    times = np.arange(len(errors_left)) * CONTROL_DT

    rms_startup = np.sqrt(np.mean((errors_left[times <= 3] + errors_right[times <= 3])**2 / 2))
    rms_steady = np.sqrt(np.mean((errors_left[times >= 6] + errors_right[times >= 6])**2 / 2))

    print()
    print("=" * 60)
    print("优化结果")
    print("=" * 60)
    print(f"左臂 RMS:   {rms_left * 100:6.2f} cm (最大: {np.max(errors_left) * 100:6.2f} cm)")
    print(f"右臂 RMS:   {rms_right * 100:6.2f} cm (最大: {np.max(errors_right) * 100:6.2f} cm)")
    print(f"平均 RMS:   {rms_avg * 100:6.2f} cm")
    print(f"启动(0-3s): {rms_startup * 100:6.2f} cm")
    print(f"稳态(6s+):  {rms_steady * 100:6.2f} cm")
    print(f"IK残差:     {np.mean(ik_residuals) * 100:.3f} cm")
    print(f"力矩饱和:   {saturation_count}/{total_steps} ({100 * saturation_count / total_steps:.1f}%)")
    print(f"墙钟时间:   {wall_time:.2f} s (模拟 {duration:.1f} s)")
    print()
    print("对比:")
    print(f"  原始版本(Kp=500):   16.00 cm  ❌")
    print(f"  优化版本(自适应):   {rms_avg * 100:6.2f} cm  {'✅' if rms_avg * 100 < 5.0 else '⚠️'}")
    print(f"  Phase 6-v2单臂:     1.75 cm   (参考)")
    print("=" * 60)

    return {"rms_avg": rms_avg, "saturation_rate": saturation_count / total_steps}


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--omega", type=float, default=0.5)
    parser.add_argument("--radius", type=float, default=0.08)
    parser.add_argument("--no-gravity-ff", action="store_true")
    parser.add_argument("--render", action="store_true")
    args = parser.parse_args()

    main(
        duration=args.duration,
        omega=args.omega,
        radius=args.radius,
        use_gravity_ff=not args.no_gravity_ff,
        render=args.render,
    )
