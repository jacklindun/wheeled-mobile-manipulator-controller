#!/usr/bin/env python3
"""
Phase 6-v3 测试版本 - Step 1: 验证力矩控制+PD

测试目标:
1. 验证力矩执行器模型是否正常
2. 验证前馈PD控制器在力矩模式下是否稳定
3. 对比position actuator vs torque actuator的性能

架构: IK参考轨迹 → 插值 → 前馈PD (纯PD，无MPC前馈) → MuJoCo力矩控制
"""

import sys
sys.path.insert(0, '.')
sys.path.insert(0, '../../build/bindings/python')

import numpy as np
import mujoco
import mujoco.viewer
import time

from wheeled_ur5e_aligator_mpc.dual_arm_pinocchio_model import DualArmPinocchioModel
from wheeled_ur5e_aligator_mpc.feedforward_pd_controller import FeedforwardPDController, FeedforwardPDGains
from wheeled_ur5e_aligator_mpc.coordinate_mapping import (
    q_to_ctrl, DUAL_ARM_Q_NOMINAL, DUAL_ARM_TAU_MAX_Q,
)


class FixedBaseIKPlanner:
    """固定基座的双臂IK规划器"""

    def __init__(self, pinocchio_model: DualArmPinocchioModel):
        self.model = pinocchio_model
        self.q_nominal = DUAL_ARM_Q_NOMINAL.copy()

    def solve_ik_fixed_base(self, target_left, target_right, max_iter=100):
        """固定基座的双臂IK"""
        q_ik = self.q_nominal.copy()

        for i in range(max_iter):
            pos_left = self.model.fk_left_ee(q_ik)
            error_left = target_left - pos_left

            pos_right = self.model.fk_right_ee(q_ik)
            error_right = target_right - pos_right

            total_error = np.linalg.norm(error_left) + np.linalg.norm(error_right)

            if total_error < 1e-4:
                break

            J_left = self.model.jacobian_left_ee(q_ik)[:3, :]
            J_left[:, 0:4] = 0

            J_right = self.model.jacobian_right_ee(q_ik)[:3, :]
            J_right[:, 0:4] = 0

            J_stacked = np.vstack([J_left, J_right])
            error_stacked = np.concatenate([error_left, error_right])

            dq = np.linalg.lstsq(J_stacked, error_stacked, rcond=None)[0]
            dq[0:4] = 0

            q_ik += 0.3 * dq

        return q_ik


class SimpleInterpolator:
    """简单线性插值器"""

    def __init__(self, ratio=25):
        self.ratio = ratio
        self.q_start = None
        self.q_end = None

    def set_target(self, q_start, q_end):
        self.q_start = q_start.copy()
        self.q_end = q_end.copy()

    def interpolate(self, step):
        alpha = step / self.ratio
        q_interp = (1 - alpha) * self.q_start + alpha * self.q_end
        # 估计速度 (简单差分)
        v_interp = (self.q_end - self.q_start) / (self.ratio * 0.002)  # dt=0.002
        return q_interp, v_interp


class Phase6V3Test:
    """Phase 6-v3 测试版本"""

    def __init__(self):
        # 加载MuJoCo模型 - 力矩执行器版本
        mjcf_path = 'assets/wheeled_dual_ur5e_v2_torque.xml'
        self.mj_model = mujoco.MjModel.from_xml_path(mjcf_path)
        self.mj_data = mujoco.MjData(self.mj_model)

        # 加载Pinocchio模型
        self.pin_model = DualArmPinocchioModel('assets/wheeled_dual_ur5e_v2.xml')

        # IK规划器
        self.ik_planner = FixedBaseIKPlanner(self.pin_model)

        # 插值器
        self.mpc_dt = 0.05
        self.control_dt = 0.002
        self.interpolation_ratio = int(self.mpc_dt / self.control_dt)
        self.interpolator = SimpleInterpolator(ratio=self.interpolation_ratio)

        # 前馈PD控制器 - 力矩模式
        gains = FeedforwardPDGains(
            Kp_base_xy=200.0,  Kd_base_xy=50.0,
            Kp_base_z=1000.0,  Kd_base_z=200.0,
            Kp_base_yaw=100.0, Kd_base_yaw=20.0,
            Kp_arm=500.0,      Kd_arm=50.0  # 初始保守增益
        )
        self.pd_controller = FeedforwardPDController(gains)

        # 设置力矩限制
        self.pd_controller.set_control_limits(-DUAL_ARM_TAU_MAX_Q, DUAL_ARM_TAU_MAX_Q)

        # 轨迹参数
        self.omega = 0.5
        self.duration = 10.0
        self.time = 0.0

        # 统计
        self.errors_left = []
        self.errors_right = []
        self.ik_residuals = []

    def circle_trajectory(self, t):
        """双臂圆形轨迹 - 使用较小半径避免碰撞"""
        radius = 0.08  # 减小到8cm
        omega = self.omega
        center_left = np.array([0.600, 0.300, 0.800])
        center_right = np.array([0.600, -0.300, 0.800])

        target_left = center_left + radius * np.array([
            0.0,
            np.cos(omega * t),
            np.sin(omega * t)
        ])

        target_right = center_right + radius * np.array([
            0.0,
            np.cos(omega * t),
            np.sin(omega * t)
        ])

        return target_left, target_right

    def run(self):
        """运行控制循环"""
        print("启动 Phase 6-v3 测试版本")
        print("=" * 60)
        print(f"架构: IK → 插值 → 重力前馈PD → MuJoCo力矩控制")
        print(f"目标: 验证力矩控制器 + 重力补偿")
        print(f"")
        print(f"参数:")
        print(f"  执行器类型: motor (torque)")
        print(f"  MPC频率: {1.0/self.mpc_dt:.0f} Hz")
        print(f"  控制频率: {1.0/self.control_dt:.0f} Hz")
        print(f"  PD增益: Kp_arm={self.pd_controller.gains.Kp_arm[0]:.0f}, Kd_arm={self.pd_controller.gains.Kd_arm[0]:.0f}")
        print()

        # 初始化到nominal
        q_nominal = self.ik_planner.q_nominal
        mujoco.mj_resetData(self.mj_model, self.mj_data)
        self.mj_data.qpos[:16] = q_nominal
        self.mj_data.qvel[:16] = 0.0

        # 重要: 设置初始力矩为重力补偿
        mujoco.mj_forward(self.mj_model, self.mj_data)

        # 初始化插值器
        self.interpolator.set_target(q_nominal, q_nominal)

        # 启动viewer
        with mujoco.viewer.launch_passive(self.mj_model, self.mj_data) as viewer:

            # 获取mocap目标的ID
            left_body_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_BODY, "left_target_body")
            right_body_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_BODY, "right_target_body")
            left_mocap_id = self.mj_model.body_mocapid[left_body_id] if left_body_id >= 0 else -1
            right_mocap_id = self.mj_model.body_mocapid[right_body_id] if right_body_id >= 0 else -1

            mpc_counter = 0

            while viewer.is_running() and self.time < self.duration:
                t_start = time.time()

                # 获取当前目标位置
                target_left, target_right = self.circle_trajectory(self.time)

                # 更新mocap可视化
                if left_mocap_id >= 0:
                    self.mj_data.mocap_pos[left_mocap_id] = target_left
                if right_mocap_id >= 0:
                    self.mj_data.mocap_pos[right_mocap_id] = target_right

                # MPC更新 (20 Hz)
                if mpc_counter % self.interpolation_ratio == 0:
                    q_ik = self.ik_planner.solve_ik_fixed_base(target_left, target_right)

                    # 设置插值器
                    q_current = self.mj_data.qpos[:16].copy()
                    self.interpolator.set_target(q_current, q_ik)

                    # 验证IK精度
                    pos_left = self.pin_model.fk_left_ee(q_ik)
                    pos_right = self.pin_model.fk_right_ee(q_ik)
                    ik_error_left = np.linalg.norm(pos_left - target_left)
                    ik_error_right = np.linalg.norm(pos_right - target_right)
                    self.ik_residuals.append((ik_error_left + ik_error_right) / 2)

                # 插值 (500 Hz)
                step_in_mpc = mpc_counter % self.interpolation_ratio
                q_des, v_des = self.interpolator.interpolate(step_in_mpc)

                # 前馈PD控制 (带重力补偿)
                q_current = self.mj_data.qpos[:16].copy()
                v_current = self.mj_data.qvel[:16].copy()

                # 计算重力前馈
                tau_gravity = np.zeros(16)
                tau_gravity[4:] = self.mj_data.qfrc_bias[4:16]  # 从MuJoCo获取重力+科氏力

                tau_control, info = self.pd_controller.compute_control(
                    q_current=q_current,
                    v_current=v_current,
                    q_des=q_des,
                    v_des=v_des,
                    u_feedforward=tau_gravity  # ✅ 使用重力前馈
                )

                # 坐标映射：qpos顺序 → MuJoCo ctrl顺序
                tau_ctrl_order = q_to_ctrl(tau_control)
                self.mj_data.ctrl[:16] = tau_ctrl_order

                # Step simulation
                mujoco.mj_step(self.mj_model, self.mj_data)
                viewer.sync()

                # 记录误差
                pos_left_actual = self.pin_model.fk_left_ee(q_current)
                pos_right_actual = self.pin_model.fk_right_ee(q_current)

                error_left = np.linalg.norm(pos_left_actual - target_left)
                error_right = np.linalg.norm(pos_right_actual - target_right)

                self.errors_left.append(error_left)
                self.errors_right.append(error_right)

                # 实时打印
                if mpc_counter % 100 == 0:
                    ik_error = (self.ik_residuals[-1] if self.ik_residuals else 0) * 100
                    print(f"t={self.time:.2f}s: Left={error_left*100:.2f}cm, Right={error_right*100:.2f}cm, IK={ik_error:.2f}cm")

                # 更新时间
                self.time += self.control_dt
                mpc_counter += 1

                # 时间同步
                elapsed = time.time() - t_start
                if elapsed < self.control_dt:
                    time.sleep(self.control_dt - elapsed)

        # 统计结果
        self.print_statistics()

    def print_statistics(self):
        """打印统计结果"""
        print("\n" + "=" * 60)
        print("统计结果")
        print("=" * 60)

        errors_left = np.array(self.errors_left) * 100
        errors_right = np.array(self.errors_right) * 100
        ik_residuals = np.array(self.ik_residuals) * 100

        print(f"左臂 RMS: {np.sqrt(np.mean(errors_left**2)):6.2f} cm (最大: {np.max(errors_left):6.2f} cm)")
        print(f"右臂 RMS: {np.sqrt(np.mean(errors_right**2)):6.2f} cm (最大: {np.max(errors_right):6.2f} cm)")
        print(f"平均 RMS: {(np.sqrt(np.mean(errors_left**2)) + np.sqrt(np.mean(errors_right**2)))/2:6.2f} cm")
        print(f"平均IK残差: {np.mean(ik_residuals):.3f} cm")

        avg_rms = (np.sqrt(np.mean(errors_left**2)) + np.sqrt(np.mean(errors_right**2)))/2

        print(f"\n对比:")
        print(f"  Phase 6-v2 (position actuator): 14.5 cm")
        print(f"  Phase 6-v3 (torque + PD):       {avg_rms:.2f} cm")

        if avg_rms < 10.0:
            print(f"\n✓ 力矩控制器工作正常!")
        else:
            print(f"\n✗ 力矩控制器可能需要调参")


if __name__ == '__main__':
    controller = Phase6V3Test()
    controller.run()
