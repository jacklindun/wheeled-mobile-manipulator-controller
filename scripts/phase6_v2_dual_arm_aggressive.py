#!/usr/bin/env python3
"""
Phase 6-v2 双臂优化版本 - 激进增益
目标: 将跟踪误差从18cm降到<5cm

优化策略:
1. 大幅提高PD增益 (Kp: 1000→5000)
2. 添加速度前馈 (使用IK速度)
3. 更紧的控制循环
"""

import sys
sys.path.insert(0, '.')
sys.path.insert(0, '../../build/bindings/python')

import numpy as np
import pinocchio as pin
import mujoco
import mujoco.viewer
import time
from dataclasses import dataclass

from wheeled_ur5e_aligator_mpc.dual_arm_pinocchio_model import DualArmPinocchioModel
from wheeled_ur5e_aligator_mpc.feedforward_pd_controller import FeedforwardPDController, PDGains


@dataclass
class AggressiveGains:
    """激进增益配置"""
    Kp_base: float = 3000.0   # 基座增益 (原1000)
    Kd_base: float = 300.0     # 基座阻尼 (原100)
    Kp_arm: float = 5000.0     # 机械臂增益 (原1000)
    Kd_arm: float = 500.0      # 机械臂阻尼 (原100)


class ImprovedIKPlanner:
    """改进的IK规划器 - 带速度估计"""

    def __init__(self, pinocchio_model: DualArmPinocchioModel):
        self.model = pinocchio_model
        self.q_nominal = np.array([
            0.0, 0.0, 0.2, 0.0,  # base
            -2.5434, -0.6884,  1.6850, 0.4209, -1.3484,  0.0000,  # left
             1.4529, -0.7472,  2.3605, 0.3727, -1.9646,  0.0000,  # right
        ])

        # 存储上一帧状态用于速度估计
        self.q_prev = self.q_nominal.copy()
        self.t_prev = 0.0

    def solve_ik_with_velocity(self, target_left, target_right, t_current):
        """求解IK并估计速度"""
        # 求解IK
        q_ik = self.q_nominal.copy()

        # Left arm IK
        for _ in range(50):
            pos, R = self.model.fk_left_ee(q_ik)
            error = target_left - pos
            if np.linalg.norm(error) < 1e-4:
                break
            J = self.model.jacobian_left_ee(q_ik)[:3, :]
            dq = np.linalg.lstsq(J, error, rcond=None)[0]
            q_ik += 0.5 * dq

        # Right arm IK
        for _ in range(50):
            pos, R = self.model.fk_right_ee(q_ik)
            error = target_right - pos
            if np.linalg.norm(error) < 1e-4:
                break
            J = self.model.jacobian_right_ee(q_ik)[:3, :]
            dq = np.linalg.lstsq(J, error, rcond=None)[0]
            q_ik += 0.5 * dq

        # 估计速度 (数值微分)
        dt = t_current - self.t_prev
        if dt > 1e-6:
            v_ik = (q_ik - self.q_prev) / dt
        else:
            v_ik = np.zeros(16)

        # 更新历史
        self.q_prev = q_ik.copy()
        self.t_prev = t_current

        return q_ik, v_ik


class Phase6V2DualArmAggressive:
    """Phase 6-v2 双臂激进控制器"""

    def __init__(self):
        # 加载MuJoCo模型 - 使用高增益版本
        mjcf_path = 'assets/wheeled_dual_ur5e_v2_highgain.xml'
        self.mj_model = mujoco.MjModel.from_xml_path(mjcf_path)
        self.mj_data = mujoco.MjData(self.mj_model)

        # 加载Pinocchio模型
        urdf_path = 'assets/wheeled_dual_ur5e_v2.urdf'
        self.pin_model = DualArmPinocchioModel(urdf_path)

        # IK规划器
        self.ik_planner = ImprovedIKPlanner(self.pin_model)

        # PD控制器 - 激进增益
        gains = AggressiveGains()
        pd_gains = PDGains(
            Kp_base=gains.Kp_base,
            Kd_base=gains.Kd_base,
            Kp_arm=gains.Kp_arm,
            Kd_arm=gains.Kd_arm
        )
        self.pd_controller = FeedforwardPDController(pd_gains)

        # 控制频率
        self.mpc_dt = 0.05      # 20 Hz
        self.control_dt = 0.002  # 500 Hz
        self.interpolation_ratio = int(self.mpc_dt / self.control_dt)

        # 轨迹
        self.duration = 10.0
        self.time = 0.0

        # 统计
        self.errors_left = []
        self.errors_right = []
        self.ik_residuals = []

    def circle_trajectory(self, t):
        """双臂圆形轨迹"""
        radius = 0.15
        omega = 0.5  # rad/s
        center_left = np.array([0.500, 0.300, 0.800])
        center_right = np.array([0.500, -0.300, 0.800])

        target_left = center_left + radius * np.array([
            np.cos(omega * t),
            np.sin(omega * t),
            0.0
        ])

        target_right = center_right + radius * np.array([
            np.cos(omega * t),
            -np.sin(omega * t),  # 镜像
            0.0
        ])

        return target_left, target_right

    def run(self):
        """运行控制循环"""
        print("启动 Phase 6-v2 双臂激进控制器")
        print("=" * 60)
        print(f"PD增益: Kp_arm={self.pd_controller.gains.Kp_arm}, Kd_arm={self.pd_controller.gains.Kd_arm}")
        print(f"控制频率: {1.0/self.control_dt:.0f} Hz")
        print(f"轨迹时长: {self.duration}s")
        print()

        # 初始化到nominal
        q_nominal = self.ik_planner.q_nominal
        mujoco.mj_resetData(self.mj_model, self.mj_data)
        self.mj_data.qpos[:16] = q_nominal
        self.mj_data.qvel[:16] = 0.0
        mujoco.mj_forward(self.mj_model, self.mj_data)

        # 启动viewer
        with mujoco.viewer.launch_passive(self.mj_model, self.mj_data) as viewer:

            mpc_counter = 0
            q_des = q_nominal.copy()
            v_des = np.zeros(16)

            while viewer.is_running() and self.time < self.duration:
                t_start = time.time()

                # MPC更新 (20 Hz)
                if mpc_counter % self.interpolation_ratio == 0:
                    target_left, target_right = self.circle_trajectory(self.time)
                    q_ik, v_ik = self.ik_planner.solve_ik_with_velocity(target_left, target_right, self.time)

                    # 更新参考
                    q_des = q_ik
                    v_des = v_ik

                    # 验证IK精度
                    pos_left, _ = self.pin_model.fk_left_ee(q_ik)
                    pos_right, _ = self.pin_model.fk_right_ee(q_ik)
                    ik_error_left = np.linalg.norm(pos_left - target_left)
                    ik_error_right = np.linalg.norm(pos_right - target_right)
                    self.ik_residuals.append((ik_error_left + ik_error_right) / 2)

                # PD控制 (500 Hz)
                q_current = self.mj_data.qpos[:16].copy()
                v_current = self.mj_data.qvel[:16].copy()

                u_control = self.pd_controller.compute_control(
                    q_current=q_current,
                    v_current=v_current,
                    q_des=q_des,
                    v_des=v_des,
                    u_feedforward=np.zeros(16)
                )

                # 发送控制到MuJoCo
                self.mj_data.ctrl[:16] = q_des  # 位置目标

                # Step simulation
                mujoco.mj_step(self.mj_model, self.mj_data)
                viewer.sync()

                # 记录误差
                target_left, target_right = self.circle_trajectory(self.time)
                pos_left_actual, _ = self.pin_model.fk_left_ee(q_current)
                pos_right_actual, _ = self.pin_model.fk_right_ee(q_current)

                error_left = np.linalg.norm(pos_left_actual - target_left)
                error_right = np.linalg.norm(pos_right_actual - target_right)

                self.errors_left.append(error_left)
                self.errors_right.append(error_right)

                # 实时打印
                if mpc_counter % 50 == 0:
                    print(f"t={self.time:.2f}s: Left={error_left*100:.2f}cm, Right={error_right*100:.2f}cm")

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

        # 判断
        avg_rms = (np.sqrt(np.mean(errors_left**2)) + np.sqrt(np.mean(errors_right**2)))/2
        if avg_rms < 5.0:
            print(f"\n✓ 成功! 平均误差 {avg_rms:.2f} cm < 5 cm")
        else:
            print(f"\n✗ 未达标: 平均误差 {avg_rms:.2f} cm >= 5 cm")


if __name__ == '__main__':
    controller = Phase6V2DualArmAggressive()
    controller.run()
