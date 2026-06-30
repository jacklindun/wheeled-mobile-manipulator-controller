#!/usr/bin/env python3
"""
Phase 6-v3 Step 1 - 避碰版本

修改轨迹以避免双臂碰撞:
1. 增大双臂间距 (Y方向 ±0.3m → ±0.4m)
2. 减小圆半径 (0.15m → 0.10m)
3. 反向旋转 (减少交叉)
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
        v_interp = (self.q_end - self.q_start) / (self.ratio * 0.002)
        return q_interp, v_interp


class Phase6V3CollisionFree:
    """Phase 6-v3 避碰版本"""

    def __init__(self):
        # 加载MuJoCo模型 - 力矩执行器版本 (带碰撞)
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

        # 前馈PD控制器
        gains = FeedforwardPDGains(
            Kp_base_xy=200.0,  Kd_base_xy=50.0,
            Kp_base_z=1000.0,  Kd_base_z=200.0,
            Kp_base_yaw=100.0, Kd_base_yaw=20.0,
            Kp_arm=500.0,      Kd_arm=50.0
        )
        self.pd_controller = FeedforwardPDController(gains)

        # 设置力矩限制
        tau_max = np.array([200, 200, 1000, 100,
                           150, 150, 150, 28, 28, 28,
                           150, 150, 150, 28, 28, 28])
        self.pd_controller.set_control_limits(-tau_max, tau_max)

        # 轨迹参数
        self.omega = 0.5
        self.duration = 10.0
        self.time = 0.0

        # 统计
        self.errors_left = []
        self.errors_right = []
        self.ik_residuals = []
        self.collision_count = 0

    def circle_trajectory_collision_free(self, t):
        """避碰的双臂圆形轨迹

        改进:
        1. 圆心向前移: X=0.5m → X=0.6m (远离基座)
        2. 减小圆半径: 0.15m → 0.08m
        3. 这样X范围: [0.52, 0.68]m，避免靠近基座
        """
        radius = 0.08  # 减小半径，避免极限姿态
        omega = self.omega

        # 圆心向前移动，远离基座
        center_left = np.array([0.600, 0.300, 0.800])   # X: 0.5→0.6
        center_right = np.array([0.600, -0.300, 0.800])  # X: 0.5→0.6

        # 左臂: 逆时针
        target_left = center_left + radius * np.array([
            np.cos(omega * t),
            np.sin(omega * t),
            0.0
        ])

        # 右臂: 顺时针 (反向)
        target_right = center_right + radius * np.array([
            np.cos(omega * t),
            -np.sin(omega * t),
            0.0
        ])

        return target_left, target_right

    def check_collision(self):
        """检测是否发生严重碰撞

        只统计穿透深度 > 1mm 的碰撞
        """
        for i in range(self.mj_data.ncon):
            contact = self.mj_data.contact[i]
            if contact.dist < -0.001:  # 穿透超过1mm才算碰撞
                # 获取碰撞体名称
                geom1_id = contact.geom1
                geom2_id = contact.geom2
                geom1_name = mujoco.mj_id2name(self.mj_model, mujoco.mjtObj.mjOBJ_GEOM, geom1_id)
                geom2_name = mujoco.mj_id2name(self.mj_model, mujoco.mjtObj.mjOBJ_GEOM, geom2_id)

                # 忽略与地板的接触
                if 'floor' in [geom1_name, geom2_name]:
                    continue

                return True, contact.dist, geom1_name, geom2_name
        return False, 0.0, None, None

    def run(self):
        """运行控制循环"""
        print("启动 Phase 6-v3 避碰版本")
        print("=" * 60)
        print(f"架构: IK → 插值 → 前馈PD (纯PD) → MuJoCo力矩控制 (带碰撞)")
        print(f"轨迹优化: 圆心前移+减小半径，避免极限姿态")
        print(f"")
        print(f"参数:")
        print(f"  执行器类型: motor (torque)")
        print(f"  碰撞检测: 启用")
        print(f"  圆心位置: X=0.6m (原0.5m)")
        print(f"  圆半径: 0.08m (原0.15m)")
        print(f"  X工作范围: [0.52, 0.68]m (避免靠近基座)")
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
                target_left, target_right = self.circle_trajectory_collision_free(self.time)

                # 更新mocap可视化
                if left_mocap_id >= 0:
                    self.mj_data.mocap_pos[left_mocap_id] = target_left
                if right_mocap_id >= 0:
                    self.mj_data.mocap_pos[right_mocap_id] = target_right

                # MPC更新 (20 Hz)
                if mpc_counter % self.interpolation_ratio == 0:
                    q_ik = self.ik_planner.solve_ik_fixed_base(target_left, target_right)

                    q_current = self.mj_data.qpos[:16].copy()
                    self.interpolator.set_target(q_current, q_ik)

                    pos_left = self.pin_model.fk_left_ee(q_ik)
                    pos_right = self.pin_model.fk_right_ee(q_ik)
                    ik_error_left = np.linalg.norm(pos_left - target_left)
                    ik_error_right = np.linalg.norm(pos_right - target_right)
                    self.ik_residuals.append((ik_error_left + ik_error_right) / 2)

                # 插值 (500 Hz)
                step_in_mpc = mpc_counter % self.interpolation_ratio
                q_des, v_des = self.interpolator.interpolate(step_in_mpc)

                # 前馈PD控制
                q_current = self.mj_data.qpos[:16].copy()
                v_current = self.mj_data.qvel[:16].copy()

                tau_control, info = self.pd_controller.compute_control(
                    q_current=q_current,
                    v_current=v_current,
                    q_des=q_des,
                    v_des=v_des,
                    u_feedforward=None
                )

                # 发送力矩
                self.mj_data.ctrl[:16] = tau_control

                # Step simulation
                mujoco.mj_step(self.mj_model, self.mj_data)
                viewer.sync()

                # 检测碰撞
                collision, depth, geom1, geom2 = self.check_collision()
                if collision:
                    self.collision_count += 1
                    if self.collision_count == 1:  # 首次碰撞时打印详细信息
                        print(f"\n[碰撞] t={self.time:.2f}s, 深度={depth*1000:.2f}mm, {geom1} <-> {geom2}\n")

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
                    print(f"t={self.time:.2f}s: Left={error_left*100:.2f}cm, Right={error_right*100:.2f}cm, Collisions={self.collision_count}")

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
        print(f"碰撞次数: {self.collision_count} / {len(self.errors_left)} ({self.collision_count/len(self.errors_left)*100:.1f}%)")

        avg_rms = (np.sqrt(np.mean(errors_left**2)) + np.sqrt(np.mean(errors_right**2)))/2

        print(f"\n对比:")
        print(f"  Phase 6-v2 (position, 无碰撞): 14.5 cm")
        print(f"  Phase 6-v3 (torque, 带碰撞):   {avg_rms:.2f} cm")

        if self.collision_count == 0:
            print(f"\n✓ 避碰成功!")
        else:
            print(f"\n⚠️ 仍有碰撞，需要进一步优化轨迹")


if __name__ == '__main__':
    controller = Phase6V3CollisionFree()
    controller.run()
