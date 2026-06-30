#!/usr/bin/env python3
"""
Phase 6-v2 双臂高增益测试
目标: 将跟踪误差从18cm降到<5cm

策略: 使用高增益MuJoCo模型 (kp=10000) + IK预规划
"""

import sys
sys.path.insert(0, '.')
sys.path.insert(0, '../../build/bindings/python')

import numpy as np
import pinocchio as pin
import mujoco
import mujoco.viewer
import time

from wheeled_ur5e_aligator_mpc.dual_arm_pinocchio_model import DualArmPinocchioModel


class IKPlanner:
    """IK规划器"""

    def __init__(self, pinocchio_model: DualArmPinocchioModel):
        self.model = pinocchio_model
        self.q_nominal = np.array([
            0.0, 0.0, 0.2, 0.0,  # base
            -2.5434, -0.6884,  1.6850, 0.4209, -1.3484,  0.0000,  # left
             1.4529, -0.7472,  2.3605, 0.3727, -1.9646,  0.0000,  # right
        ])

    def solve_ik(self, target_left, target_right):
        """求解IK"""
        q_ik = self.q_nominal.copy()

        # Left arm IK
        for _ in range(50):
            pos = self.model.fk_left_ee(q_ik)
            error = target_left - pos
            if np.linalg.norm(error) < 1e-4:
                break
            J = self.model.jacobian_left_ee(q_ik)[:3, :]
            dq = np.linalg.lstsq(J, error, rcond=None)[0]
            q_ik += 0.5 * dq

        # Right arm IK
        for _ in range(50):
            pos = self.model.fk_right_ee(q_ik)
            error = target_right - pos
            if np.linalg.norm(error) < 1e-4:
                break
            J = self.model.jacobian_right_ee(q_ik)[:3, :]
            dq = np.linalg.lstsq(J, error, rcond=None)[0]
            q_ik += 0.5 * dq

        return q_ik


class HighGainTest:
    """高增益测试"""

    def __init__(self):
        # 加载MuJoCo模型 - 高增益版本
        mjcf_path = 'assets/wheeled_dual_ur5e_v2_highgain.xml'
        self.mj_model = mujoco.MjModel.from_xml_path(mjcf_path)
        self.mj_data = mujoco.MjData(self.mj_model)

        # 加载Pinocchio模型 (使用原始v2的MJCF)
        pin_mjcf_path = 'assets/wheeled_dual_ur5e_v2.xml'
        self.pin_model = DualArmPinocchioModel(pin_mjcf_path)

        # IK规划器
        self.ik_planner = IKPlanner(self.pin_model)

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
        print("启动 Phase 6-v2 双臂高增益测试")
        print("=" * 60)
        print(f"模型: wheeled_dual_ur5e_v2_highgain.xml")
        print(f"执行器增益: kp=10000 (shoulder), kp=8000 (elbow), kp=3000 (wrist)")
        print(f"控制频率: {1.0/self.control_dt:.0f} Hz")
        print(f"MPC频率: {1.0/self.mpc_dt:.0f} Hz")
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
            q_target = q_nominal.copy()

            while viewer.is_running() and self.time < self.duration:
                t_start = time.time()

                # MPC更新 (20 Hz)
                if mpc_counter % self.interpolation_ratio == 0:
                    target_left, target_right = self.circle_trajectory(self.time)
                    q_ik = self.ik_planner.solve_ik(target_left, target_right)

                    # 更新参考
                    q_target = q_ik

                    # 验证IK精度
                    pos_left = self.pin_model.fk_left_ee(q_ik)
                    pos_right = self.pin_model.fk_right_ee(q_ik)
                    ik_error_left = np.linalg.norm(pos_left - target_left)
                    ik_error_right = np.linalg.norm(pos_right - target_right)
                    self.ik_residuals.append((ik_error_left + ik_error_right) / 2)

                # 发送位置目标到MuJoCo (MuJoCo内置PD控制)
                self.mj_data.ctrl[:16] = q_target

                # Step simulation
                mujoco.mj_step(self.mj_model, self.mj_data)
                viewer.sync()

                # 记录误差
                q_current = self.mj_data.qpos[:16].copy()
                target_left, target_right = self.circle_trajectory(self.time)
                pos_left_actual = self.pin_model.fk_left_ee(q_current)
                pos_right_actual = self.pin_model.fk_right_ee(q_current)

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
        elif avg_rms < 10.0:
            print(f"\n△ 改善: 平均误差 {avg_rms:.2f} cm (从18cm降低)")
        else:
            print(f"\n✗ 未达标: 平均误差 {avg_rms:.2f} cm >= 10 cm")


if __name__ == '__main__':
    controller = HighGainTest()
    controller.run()
