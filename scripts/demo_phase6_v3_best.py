#!/usr/bin/env python3
"""Phase 6-v3 Step1 可视化版本 - 最佳效果演示"""

import sys
import time

sys.path.insert(0, ".")
sys.path.insert(0, "../../build/bindings/python")

import mujoco
import mujoco.viewer
import numpy as np

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

print("="*60)
print("Phase 6-v3 Step1 可视化演示 (最佳配置)")
print("="*60)
print("配置: IK + 重力前馈 + PD控制")
print("预期性能: ~2.28 cm RMS")
print("="*60)

def main(duration: float = 15.0):
    # 模型
    pin_model = DualArmPinocchioModel(mjcf_path=MJCF_PIN)
    mj_model = mujoco.MjModel.from_xml_path(MJCF_TORQUE)
    mj_data = mujoco.MjData(mj_model)

    # 预热：让机器人先移动到轨迹起点
    print("\n预热阶段：移动到轨迹起点...")
    ik_planner = FixedBaseIKPlanner(pin_model)
    target_left_0, target_right_0 = circle_trajectory(0.0, omega=0.5, radius=0.08)
    q_init = ik_planner.solve_ik_fixed_base(target_left_0, target_right_0)
    mj_data.qpos[:16] = q_init
    mujoco.mj_forward(mj_model, mj_data)
    print("✓ 预热完成\n")

    # 启动可视化
    with mujoco.viewer.launch_passive(mj_model, mj_data) as viewer:
        # IK和插值
        interpolator = JointInterpolator()
        pd_controller = make_pd_controller()

        # 初始化（使用预热后的位置）
        interpolator.set_segment(q_init, q_init)

        # 查找mocap body用于显示目标
        try:
            left_target_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_BODY, 'left_target_body')
            right_target_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_BODY, 'right_target_body')
            has_mocap = left_target_id >= 0 and right_target_id >= 0
            if has_mocap:
                # 找到对应的mocap索引
                left_mocap_id = -1
                right_mocap_id = -1
                for i in range(mj_model.nmocap):
                    if mj_model.body_mocapid[left_target_id] == i:
                        left_mocap_id = i
                    if mj_model.body_mocapid[right_target_id] == i:
                        right_mocap_id = i

                if left_mocap_id >= 0 and right_mocap_id >= 0:
                    print(f"✓ 找到目标球（mocap {left_mocap_id}, {right_mocap_id}），将显示参考轨迹\n")
                    has_mocap = True
                    mocap_ids = (left_mocap_id, right_mocap_id)
                else:
                    print("⚠️ 找到body但无mocap索引\n")
                    has_mocap = False
            else:
                print("⚠️ 模型中没有目标球（mocap），仅显示机器人运动\n")
                has_mocap = False
        except Exception as e:
            has_mocap = False
            print(f"⚠️ 查找mocap失败: {e}\n")

        # 数据记录
        errors_left, errors_right = [], []
        ik_residuals = []

        print("\n开始运行...")
        print(f"{'时间':>8s} | {'左臂误差':>10s} | {'右臂误差':>10s} | {'IK残差':>10s}")
        print("-" * 60)

        for step in range(int(duration / CONTROL_DT)):
            t = step * CONTROL_DT
            step_in_mpc = step % INTERPOLATION_RATIO

            # IK更新（20Hz）
            if step_in_mpc == 0:
                target_left, target_right = circle_trajectory(t, omega=0.5, radius=0.08)
                q_current = mj_data.qpos[:16].copy()
                q_ik = ik_planner.solve_ik_fixed_base(target_left, target_right)

                # 计算IK残差
                pos_left_ik = pin_model.fk_left_ee(q_ik)
                pos_right_ik = pin_model.fk_right_ee(q_ik)
                residual = np.linalg.norm(pos_left_ik - target_left) + np.linalg.norm(pos_right_ik - target_right)
                ik_residuals.append(residual)

                interpolator.set_segment(q_current, q_ik)

            # 插值（500Hz）
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

            # 更新目标球位置
            if has_mocap:
                target_left, target_right = circle_trajectory(t, omega=0.5, radius=0.08)
                mj_data.mocap_pos[mocap_ids[0]] = target_left
                mj_data.mocap_pos[mocap_ids[1]] = target_right

            viewer.sync()

            # 统计
            target_left, target_right = circle_trajectory(t, omega=0.5, radius=0.08)
            el, er = ee_tracking_errors(pin_model, q_current, target_left, target_right)
            errors_left.append(el)
            errors_right.append(er)

            # 打印进度
            if step % int(1.0 / CONTROL_DT) == 0 and ik_residuals:
                print(f"{t:>7.1f}s | {el*100:>8.2f} cm | {er*100:>8.2f} cm | {ik_residuals[-1]*100:>8.3f} cm")

    # 结果
    errors_left = np.array(errors_left)
    errors_right = np.array(errors_right)
    rms_left = np.sqrt(np.mean(errors_left**2))
    rms_right = np.sqrt(np.mean(errors_right**2))
    rms_avg = (rms_left + rms_right) / 2

    print()
    print("="*60)
    print("最终结果")
    print("="*60)
    print(f"左臂 RMS:   {rms_left*100:6.2f} cm (最大: {np.max(errors_left)*100:6.2f} cm)")
    print(f"右臂 RMS:   {rms_right*100:6.2f} cm (最大: {np.max(errors_right)*100:6.2f} cm)")
    print(f"平均 RMS:   {rms_avg*100:6.2f} cm")
    print(f"IK残差:     {np.mean(ik_residuals)*100:.3f} cm")
    print()
    print("性能评估:")
    if rms_avg*100 <= 2.5:
        print("  ✅ 优秀 (≤2.5 cm)")
    elif rms_avg*100 <= 3.5:
        print("  ✅ 良好 (≤3.5 cm)")
    else:
        print("  ⚠️ 需改进 (>3.5 cm)")
    print("="*60)

if __name__ == "__main__":
    main(duration=15.0)
