#!/usr/bin/env python3
"""
Phase 6-v3 参数优化脚本

系统测试不同PD增益组合，找到最佳跟踪性能
目标：最小化跟踪误差 + 控制力矩饱和率
"""

import sys
sys.path.insert(0, ".")
sys.path.insert(0, "../../build/bindings/python")

import time
import numpy as np
import mujoco

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

print("="*80)
print("Phase 6-v3 参数优化")
print("="*80)

# 测试配置
TEST_CONFIGS = [
    {"name": "Baseline (原始)", "Kp_arm": 500, "Kd_arm": 50},
    {"name": "Medium", "Kp_arm": 1200, "Kd_arm": 120},
    {"name": "High", "Kp_arm": 1800, "Kd_arm": 180},
    {"name": "Very High", "Kp_arm": 2400, "Kd_arm": 240},
    {"name": "Ultra High", "Kp_arm": 3000, "Kd_arm": 300},
    {"name": "Extreme", "Kp_arm": 4000, "Kd_arm": 400},
]

def run_test(Kp_arm, Kd_arm, duration=10.0):
    """运行单次测试"""

    # 创建PD控制器
    gains = FeedforwardPDGains(
        Kp_base_xy=300.0, Kd_base_xy=60.0,
        Kp_base_z=2000.0, Kd_base_z=400.0,
        Kp_base_yaw=200.0, Kd_base_yaw=40.0,
        Kp_arm=Kp_arm, Kd_arm=Kd_arm,
    )
    pd_controller = FeedforwardPDController(gains)
    pd_controller.set_control_limits(-DUAL_ARM_TAU_MAX_Q, DUAL_ARM_TAU_MAX_Q)

    # 模型
    pin_model = DualArmPinocchioModel(mjcf_path=MJCF_PIN)
    mj_model = mujoco.MjModel.from_xml_path(MJCF_TORQUE)
    mj_data = mujoco.MjData(mj_model)

    # IK和插值
    ik_planner = FixedBaseIKPlanner(pin_model)
    interpolator = JointInterpolator()

    # 初始化
    mj_data.qpos[:16] = DUAL_ARM_Q_NOMINAL
    interpolator.set_segment(DUAL_ARM_Q_NOMINAL, DUAL_ARM_Q_NOMINAL)

    # 数据记录
    errors_left, errors_right = [], []
    saturation_count = 0
    total_steps = 0

    for step in range(int(duration / CONTROL_DT)):
        t = step * CONTROL_DT
        step_in_mpc = step % INTERPOLATION_RATIO

        # IK更新
        if step_in_mpc == 0:
            target_left, target_right = circle_trajectory(t, omega=0.5, radius=0.08)
            q_current = mj_data.qpos[:16].copy()
            q_ik = ik_planner.solve_ik_fixed_base(target_left, target_right)
            interpolator.set_segment(q_current, q_ik)

        # 插值
        q_des, v_des = interpolator.interpolate(step_in_mpc)

        # 重力前馈 + PD
        tau_ff = compute_gravity_torque(pin_model, q_des)
        q_current = mj_data.qpos[:16]
        v_current = mj_data.qvel[:16]
        tau_pd, _ = pd_controller.compute_control(
            q_current, v_current, q_des, v_des, u_feedforward=tau_ff,
        )

        # 应用力矩
        mj_data.ctrl[:] = q_to_ctrl(tau_pd)
        mujoco.mj_step(mj_model, mj_data)

        # 统计
        target_left, target_right = circle_trajectory(t, omega=0.5, radius=0.08)
        el, er = ee_tracking_errors(pin_model, q_current, target_left, target_right)
        errors_left.append(el)
        errors_right.append(er)

        if np.any(np.abs(tau_pd) > DUAL_ARM_TAU_MAX_Q * 0.95):
            saturation_count += 1
        total_steps += 1

    # 计算指标
    errors_left = np.array(errors_left)
    errors_right = np.array(errors_right)
    rms_left = np.sqrt(np.mean(errors_left**2))
    rms_right = np.sqrt(np.mean(errors_right**2))
    rms_avg = (rms_left + rms_right) / 2
    saturation_rate = saturation_count / total_steps

    return {
        "rms_avg": rms_avg,
        "rms_left": rms_left,
        "rms_right": rms_right,
        "max_left": np.max(errors_left),
        "max_right": np.max(errors_right),
        "saturation_rate": saturation_rate,
    }

# 运行所有配置
print("\n开始参数优化测试...\n")
results = []

for config in TEST_CONFIGS:
    print(f"[测试] {config['name']}")
    print(f"  Kp_arm={config['Kp_arm']}, Kd_arm={config['Kd_arm']}")

    try:
        result = run_test(config['Kp_arm'], config['Kd_arm'], duration=10.0)
        results.append({"config": config, "result": result})

        print(f"  结果:")
        print(f"    平均RMS: {result['rms_avg']*100:.2f} cm")
        print(f"    左臂RMS: {result['rms_left']*100:.2f} cm (最大: {result['max_left']*100:.2f} cm)")
        print(f"    右臂RMS: {result['rms_right']*100:.2f} cm (最大: {result['max_right']*100:.2f} cm)")
        print(f"    力矩饱和: {result['saturation_rate']*100:.1f}%")

        # 评分
        score = 0
        if result['rms_avg']*100 <= 2.5:
            score += 5
        elif result['rms_avg']*100 <= 3.5:
            score += 4
        elif result['rms_avg']*100 <= 5.0:
            score += 3
        elif result['rms_avg']*100 <= 8.0:
            score += 2
        else:
            score += 1

        if result['saturation_rate'] < 0.05:
            score += 3
        elif result['saturation_rate'] < 0.15:
            score += 2
        elif result['saturation_rate'] < 0.50:
            score += 1

        print(f"    综合评分: {'⭐'*score}/⭐⭐⭐⭐⭐⭐⭐⭐")

    except Exception as e:
        print(f"  ✗ 测试失败: {e}")
        results.append({"config": config, "result": None})

    print()

# 总结
print("="*80)
print("优化结果总结")
print("="*80)

print(f"\n{'配置':<20} | {'Kp':<6} | {'平均RMS':<10} | {'力矩饱和':<10} | {'评分'}")
print("-"*75)

for item in results:
    config = item["config"]
    result = item["result"]
    if result:
        # 重新计算评分
        score = 0
        if result['rms_avg']*100 <= 2.5:
            score += 5
        elif result['rms_avg']*100 <= 3.5:
            score += 4
        elif result['rms_avg']*100 <= 5.0:
            score += 3
        elif result['rms_avg']*100 <= 8.0:
            score += 2
        else:
            score += 1

        if result['saturation_rate'] < 0.05:
            score += 3
        elif result['saturation_rate'] < 0.15:
            score += 2
        elif result['saturation_rate'] < 0.50:
            score += 1

        score_str = '⭐'*score
        print(f"{config['name']:<20} | {config['Kp_arm']:<6} | {result['rms_avg']*100:>7.2f} cm | {result['saturation_rate']*100:>7.1f}%  | {score_str}")

# 找最佳配置
best = None
best_score = -1

for item in results:
    if item["result"]:
        result = item["result"]
        # 综合评分：精度权重70%，饱和权重30%
        score = (1 / (result['rms_avg']*100 + 0.1)) * 0.7 + (1 - result['saturation_rate']) * 0.3

        if score > best_score:
            best_score = score
            best = item

if best:
    print(f"\n{'='*80}")
    print(f"✅ 最佳配置: {best['config']['name']}")
    print(f"{'='*80}")
    result = best['result']
    print(f"   Kp_arm: {best['config']['Kp_arm']}")
    print(f"   Kd_arm: {best['config']['Kd_arm']}")
    print(f"   平均RMS: {result['rms_avg']*100:.2f} cm")
    print(f"   左臂RMS: {result['rms_left']*100:.2f} cm")
    print(f"   右臂RMS: {result['rms_right']*100:.2f} cm")
    print(f"   力矩饱和: {result['saturation_rate']*100:.1f}%")

    达标 = "✅ 优秀" if result['rms_avg']*100 <= 2.5 else "✅ 良好" if result['rms_avg']*100 <= 3.5 else "⚠️ 可接受"
    print(f"\n性能评估: {达标}")

    print(f"\n推荐配置:")
    print(f"  FeedforwardPDGains(")
    print(f"      Kp_arm={best['config']['Kp_arm']},")
    print(f"      Kd_arm={best['config']['Kd_arm']},")
    print(f"  )")

print("\n"+"="*80)
