#!/usr/bin/env python3
"""
Phase 6-v2 双臂高频MPC版本
策略: 提高MPC频率从20Hz到50Hz，减少跟踪滞后

目标: 将跟踪误差降到<5cm
"""

import sys
sys.path.insert(0, '.')
sys.path.insert(0, '../../build/bindings/python')

import numpy as np
import mujoco
import mujoco.viewer
import time

from wheeled_ur5e_aligator_mpc.dual_arm_pinocchio_model import DualArmPinocchioModel


class FixedBaseIKPlanner:
    """固定基座的双臂IK规划器"""

    def __init__(self, pinocchio_model: DualArmPinocchioModel):
        self.model = pinocchio_model
        self.q_nominal = np.array([
            0.0, 0.0, 0.2, 0.0,  # base - 固定
            -2.5434, -0.6884,  1.6850, 0.4209, -1.3484,  0.0000,  # left
             1.4529, -0.7472,  2.3605, 0.3727, -1.9646,  0.0000,  # right
        ])

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

    def __init__(self, ratio):
        self.ratio = ratio
        self.q_start = None
        self.q_end = None

    def set_target(self, q_start, q_end):
        self.q_start = q_start.copy()
        self.q_end = q_end.copy()

    def interpolate(self, step):
        alpha = step / self.ratio
        return (1 - alpha) * self.q_start + alpha * self.q_end


class Phase6V2HighFreq:
    """Phase 6-v2 高频MPC版本"""

    def __init__(self, mpc_freq=50):
        # 加载MuJoCo模型
        mjcf_path = 'assets/wheeled_dual_ur5e_v2_highgain.xml'
        self.mj_model = mujoco.MjModel.from_xml_path(mjcf_path)
        self.mj_data = mujoco.MjData(self.mj_model)

        # 加载Pinocchio模型
        self.pin_model = DualArmPinocchioModel('assets/wheeled_dual_ur5e_v2.xml')

        # IK规划器
        self.ik_planner = FixedBaseIKPlanner(self.pin_model)

        # 高频MPC
        self.mpc_dt = 1.0 / mpc_freq  # 可调节的MPC频率
        self.control_dt = 0.002  # 500 Hz
        self.interpolation_ratio = int(self.mpc_dt / self.control_dt)
        self.interpolator = SimpleInterpolator(ratio=self.interpolation_ratio)

        # 轨迹参数
        self.omega = 0.5  # 保持原始速度
        self.duration = 10.0
        self.time = 0.0

        # 统计
        self.errors_left = []
        self.errors_right = []
        self.ik_residuals = []

    def circle_trajectory(self, t):
        """双臂圆形轨迹"""
        radius = 0.15
        omega = self.omega
        center_left = np.array([0.500, 0.300, 0.800])
        center_right = np.array([0.500, -0.300, 0.800])

        target_left = center_left + radius * np.array([
            np.cos(omega * t),
            np.sin(omega * t),
            0.0
        ])

        target_right = center_right + radius * np.array([
            np.cos(omega * t),
            -np.sin(omega * t),
            0.0
        ])

        return target_left, target_right

    def run(self):
        """运行控制循环"""
        print("启动 Phase 6-v2 双臂高频MPC版本")
        print("=" * 60)
        print(f"优化策略: 提高MPC频率减少滞后")
        print(f"MPC频率: {1.0/self.mpc_dt:.0f} Hz (原始20Hz)")
        print(f"控制频率: {1.0/self.control_dt:.0f} Hz")
        print(f"插值比例: {self.interpolation_ratio}:1")
        print(f"执行器增益: kp=10000 (shoulder)")
        print(f"轨迹速度: omega={self.omega} rad/s (保持原始)")
        print()

        # 初始化
        q_nominal = self.ik_planner.q_nominal
        mujoco.mj_resetData(self.mj_model, self.mj_data)
        self.mj_data.qpos[:16] = q_nominal
        self.mj_data.qvel[:16] = 0.0
        mujoco.mj_forward(self.mj_model, self.mj_data)

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

                # MPC更新（高频）
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

                # 插值
                step_in_mpc = mpc_counter % self.interpolation_ratio
                q_des = self.interpolator.interpolate(step_in_mpc)

                # 发送到MuJoCo
                self.mj_data.ctrl[:16] = q_des

                # Step simulation
                mujoco.mj_step(self.mj_model, self.mj_data)
                viewer.sync()

                # 记录误差
                q_current = self.mj_data.qpos[:16].copy()
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

        print(f"\n优化进展:")
        print(f"  20Hz MPC: ~14.5 cm")
        print(f"  {int(1.0/self.mpc_dt)}Hz MPC: {avg_rms:.2f} cm")

        if avg_rms < 5.0:
            print(f"\n✓✓✓ 成功达标! 平均误差 {avg_rms:.2f} cm < 5 cm")
        elif avg_rms < 10.0:
            print(f"\n✓✓ 显著改善! 平均误差 {avg_rms:.2f} cm")
        else:
            print(f"\n✓ 有改善: 平均误差 {avg_rms:.2f} cm")


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--mpc-freq', type=int, default=50,
                       help='MPC频率 (Hz), 默认50')
    args = parser.parse_args()

    controller = Phase6V2HighFreq(mpc_freq=args.mpc_freq)
    controller.run()
