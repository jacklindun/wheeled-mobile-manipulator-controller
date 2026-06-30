#!/usr/bin/env python3
"""
Phase 6-v1.1 真实对比：MPC力矩前馈 vs 重力前馈

目标：验证动力学MPC的力矩前馈是否比简单重力前馈更好
"""

import sys
sys.path.insert(0, ".")
sys.path.insert(0, "../../build/bindings/python")

import numpy as np
import mujoco

print("="*80)
print("Phase 6-v1.1: MPC力矩前馈 vs 重力前馈对比")
print("="*80)
print()
print("测试问题: 动力学MPC的力矩输出是否比重力前馈更准确？")
print("="*80)

from wheeled_ur5e_aligator_mpc.coordinate_mapping import DUAL_ARM_Q_NOMINAL, q_to_ctrl
from wheeled_ur5e_aligator_mpc.dual_arm_pinocchio_model import DualArmPinocchioModel
from wheeled_ur5e_aligator_mpc.phase6_v3_common import (
    CONTROL_DT,
    INTERPOLATION_RATIO,
    MJCF_PIN,
    MJCF_TORQUE,
    FixedBaseIKPlanner,
    JointInterpolator,
    circle_trajectory,
    compute_gravity_torque,
    ee_tracking_errors,
    make_pd_controller,
)

def simulate_mpc_torque(pin_model, q, v, q_des, v_des):
    """
    模拟动力学MPC的力矩输出

    在真实的Phase 4中，MPC会输出:
    tau_mpc = M(q) * a_des + h(q, v)

    这里我们用Pinocchio的RNEA计算准确的动力学前馈
    """
    # 计算期望加速度（简单PD）
    Kp = 100.0
    Kd = 20.0
    a_des = Kp * (q_des - q) + Kd * (v_des - v)

    # 使用RNEA计算准确的动力学前馈
    # tau = M(q)*a + h(q,v) where h = C(q,v)*v + g(q)
    import pinocchio as pin

    # 转换到Pinocchio坐标
    q_pin = np.zeros(pin_model.model.nq)
    v_pin = np.zeros(pin_model.model.nv)
    a_pin = np.zeros(pin_model.model.nv)

    # 简化：只填充臂关节（假设底盘固定）
    q_pin[4:16] = q[4:16]  # 跳过底盘4个DOF
    v_pin[4:16] = v[4:16]
    a_pin[4:16] = a_des[4:16]

    # RNEA: tau = M*a + h
    tau_rnea = pin.rnea(pin_model.model, pin_model.data, q_pin, v_pin, a_pin)

    tau_mpc = np.zeros(16)
    tau_mpc[4:16] = tau_rnea[4:16]

    return tau_mpc

def test_feedforward_type(name, feedforward_type, duration=10.0):
    """
    测试不同前馈类型

    feedforward_type:
        'none': 无前馈
        'gravity': 重力前馈
        'mpc': 模拟MPC动力学前馈
        'mujoco': 从MuJoCo获取真实动力学
    """
    print(f"\n{'='*60}")
    print(f"测试: {name}")
    print(f"  前馈类型: {feedforward_type}")
    print(f"{'='*60}")

    # 模型
    pin_model = DualArmPinocchioModel(mjcf_path=MJCF_PIN)
    mj_model = mujoco.MjModel.from_xml_path(MJCF_TORQUE)
    mj_data = mujoco.MjData(mj_model)

    # IK和插值
    ik_planner = FixedBaseIKPlanner(pin_model)
    interpolator = JointInterpolator()
    pd_controller = make_pd_controller()

    # 预热
    target_left_0, target_right_0 = circle_trajectory(0.0, omega=0.5, radius=0.08)
    q_init = ik_planner.solve_ik_fixed_base(target_left_0, target_right_0)
    mj_data.qpos[:16] = q_init
    mujoco.mj_forward(mj_model, mj_data)

    interpolator.set_segment(q_init, q_init)

    # 数据记录
    errors_left, errors_right = [], []
    tau_ff_magnitudes = []

    n_steps = int(duration / CONTROL_DT)

    for step in range(n_steps):
        t = step * CONTROL_DT
        step_in_mpc = step % INTERPOLATION_RATIO

        # IK更新（20Hz）
        if step_in_mpc == 0:
            target_left, target_right = circle_trajectory(t, omega=0.5, radius=0.08)
            q_current = mj_data.qpos[:16].copy()
            q_ik = ik_planner.solve_ik_fixed_base(target_left, target_right)
            interpolator.set_segment(q_current, q_ik)

        # 插值（500Hz）
        q_des, v_des = interpolator.interpolate(step_in_mpc)

        # 获取当前状态
        q_current = mj_data.qpos[:16]
        v_current = mj_data.qvel[:16]

        # 选择前馈类型
        if feedforward_type == 'none':
            tau_ff = np.zeros(16)

        elif feedforward_type == 'gravity':
            # 仅重力补偿
            tau_ff = compute_gravity_torque(pin_model, q_des)

        elif feedforward_type == 'mpc':
            # 模拟动力学MPC前馈（包含惯性+科氏力+重力）
            tau_ff = simulate_mpc_torque(pin_model, q_current, v_current, q_des, v_des)

        elif feedforward_type == 'mujoco':
            # 从MuJoCo获取真实动力学（理想情况）
            mj_data.qpos[:16] = q_des
            mj_data.qvel[:16] = v_des
            mujoco.mj_forward(mj_model, mj_data)
            tau_ff = mj_data.qfrc_bias[:16].copy()  # h(q,v) = 科氏力+重力
            # 恢复当前状态
            mj_data.qpos[:16] = q_current
            mj_data.qvel[:16] = v_current

        tau_ff_magnitudes.append(np.linalg.norm(tau_ff))

        # PD控制
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

    # 结果
    errors_left = np.array(errors_left)
    errors_right = np.array(errors_right)
    tau_ff_magnitudes = np.array(tau_ff_magnitudes)

    rms_left = np.sqrt(np.mean(errors_left**2))
    rms_right = np.sqrt(np.mean(errors_right**2))
    rms_avg = (rms_left + rms_right) / 2

    print(f"\n结果:")
    print(f"  左臂RMS:   {rms_left*100:.2f} cm")
    print(f"  右臂RMS:   {rms_right*100:.2f} cm")
    print(f"  平均RMS:   {rms_avg*100:.2f} cm")
    print(f"  前馈幅值:  {np.mean(tau_ff_magnitudes):.1f} Nm (平均)")

    return rms_avg * 100, np.mean(tau_ff_magnitudes)

# 运行测试
print("\n开始测试...\n")

configs = [
    ("配置A: 无前馈（纯PD）", 'none'),
    ("配置B: 重力前馈", 'gravity'),
    ("配置C: 动力学MPC前馈（模拟）", 'mpc'),
    ("配置D: 理想动力学前馈（MuJoCo）", 'mujoco'),
]

results = {}
for name, ff_type in configs:
    try:
        rms, tau_mag = test_feedforward_type(name, ff_type, duration=10.0)
        results[name] = {'rms': rms, 'tau': tau_mag}
    except Exception as e:
        print(f"  ✗ 测试失败: {e}")
        results[name] = {'rms': 999, 'tau': 0}

# 总结
print("\n" + "="*80)
print("对比结果")
print("="*80)

print(f"\n{'配置':<40} | {'RMS误差':<12} | {'前馈幅值':<12} | {'评价'}")
print("-"*90)

baseline = results[configs[0][0]]['rms']
for name, _ in configs:
    r = results[name]
    improve = (baseline - r['rms']) / baseline * 100 if baseline > 0 else 0

    if r['rms'] < 0.5:
        grade = "✅ 优秀"
    elif r['rms'] < 2.5:
        grade = "✅ 良好"
    elif r['rms'] < 5.0:
        grade = "⚠️ 可接受"
    else:
        grade = "❌ 较差"

    print(f"{name:<40} | {r['rms']:>8.2f} cm | {r['tau']:>8.1f} Nm | {grade}")

# 核心分析
print("\n" + "="*80)
print("核心分析")
print("="*80)

rms_gravity = results[configs[1][0]]['rms']
rms_mpc = results[configs[2][0]]['rms']
rms_ideal = results[configs[3][0]]['rms']

print(f"\n1. 重力前馈 vs 无前馈:")
print(f"   改善: {baseline:.2f} → {rms_gravity:.2f} cm")
if rms_gravity < baseline * 0.2:
    print(f"   ✅ 重力前馈至关重要（改善{(1-rms_gravity/baseline)*100:.0f}%）")

print(f"\n2. MPC动力学前馈 vs 重力前馈:")
print(f"   {rms_gravity:.2f} → {rms_mpc:.2f} cm")
if abs(rms_mpc - rms_gravity) / rms_gravity < 0.1:
    print(f"   ⚠️ MPC动力学前馈没有显著优势（<10%差异）")
    print(f"   → 说明科氏力+惯性项贡献很小")
elif rms_mpc < rms_gravity * 0.8:
    print(f"   ✅ MPC动力学前馈有明显改善（>{(1-rms_mpc/rms_gravity)*100:.0f}%）")
    print(f"   → Phase 4的MPC有价值！")
else:
    print(f"   ❌ MPC动力学前馈反而更差")
    print(f"   → 可能是模型误差累积")

print(f"\n3. 理想动力学前馈:")
print(f"   RMS: {rms_ideal:.2f} cm")
if rms_ideal < rms_gravity * 0.8:
    print(f"   ✅ 完美动力学有改善，说明动力学项有价值")
else:
    print(f"   ⚠️ 完美动力学也无显著改善，说明动力学项本身贡献小")

# 最终结论
print("\n" + "="*80)
print("最终结论")
print("="*80)

if abs(rms_mpc - rms_gravity) / rms_gravity < 0.2:
    print("\n❌ Phase 4的动力学MPC前馈价值有限")
    print("   - 相比简单重力前馈，改善<20%")
    print("   - 科氏力和惯性项在慢速运动中贡献很小")
    print("   - 不值得MPC的85ms计算成本")
    print("\n✅ 推荐: Phase 6-v3 (IK + 重力前馈)")
else:
    print("\n✅ Phase 4的动力学MPC前馈有显著价值")
    print("   - 相比简单重力前馈，有明显改善")
    print("   - 值得继续优化Phase 4")

print("\n" + "="*80)
