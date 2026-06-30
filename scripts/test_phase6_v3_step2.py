#!/usr/bin/env python3
"""
Phase 6-v3 Step 2: 完整动力学MPC

架构: ALIGATOR动力学MPC (20Hz) → 插值 (25:1) → 前馈PD (500Hz) → MuJoCo力矩控制

目标: < 1cm 跟踪误差
"""

import sys
sys.path.insert(0, '.')
sys.path.insert(0, '../../build/bindings/python')

import numpy as np
import mujoco
import mujoco.viewer
import time

from wheeled_ur5e_aligator_mpc.dual_arm_pinocchio_model import DualArmPinocchioModel
from wheeled_ur5e_aligator_mpc.dual_arm_dynamics_mpc import DualArmDynamicsMPC
from wheeled_ur5e_aligator_mpc.feedforward_pd_controller import FeedforwardPDController, FeedforwardPDGains
from wheeled_ur5e_aligator_mpc.coordinate_mapping import q_to_ctrl


class TrajectoryInterpolatorSimple:
    """
    简单的轨迹插值器

    修正版：每个 MPC 周期只执行第一段 [xs[0], xs[1]]，
    用 ratio 个 500Hz tick 插值。
    """

    def __init__(self, ratio=25):
        self.ratio = ratio
        self.x0 = None  # 当前状态
        self.x1 = None  # 下一状态
        self.u0 = None  # 当前控制

    def set_segment(self, xs, us):
        """
        设置当前要插值的段

        Parameters
        ----------
        xs : (ratio+1, 32) array
            状态轨迹，但只使用前两个: xs[0], xs[1]
        us : (ratio, 16) array
            控制轨迹，但只使用第一个: us[0]
        """
        # 只取第一段 [xs[0], xs[1]]
        self.x0 = xs[0]  # 当前状态
        self.x1 = xs[1] if len(xs) > 1 else xs[0]  # 下一状态
        self.u0 = us[0]  # 当前控制

    def interpolate(self, step):
        """
        线性插值: step in [0, ratio-1]

        Parameters
        ----------
        step : int
            当前步数 [0, ratio-1]

        Returns
        -------
        q_des : (16,) 期望位置
        v_des : (16,) 期望速度
        tau_ff : (16,) 前馈力矩
        """
        if self.x0 is None:
            # 未初始化，返回零
            return np.zeros(16), np.zeros(16), np.zeros(16)

        # 线性插值状态: 从 x0 到 x1
        alpha = step / self.ratio
        x_interp = (1 - alpha) * self.x0 + alpha * self.x1

        q_des = x_interp[:16]
        v_des = x_interp[16:]
        tau_ff = self.u0  # 前馈力矩使用第一个控制，不插值

        return q_des, v_des, tau_ff


class Phase6V3Complete:
    """Phase 6-v3 完整动力学MPC版本"""

    def __init__(self):
        # 加载MuJoCo模型 - 力矩执行器
        mjcf_path = 'assets/wheeled_dual_ur5e_v2_torque.xml'
        self.mj_model = mujoco.MjModel.from_xml_path(mjcf_path)
        self.mj_data = mujoco.MjData(self.mj_model)

        # 加载Pinocchio模型
        pin_mjcf = 'assets/wheeled_dual_ur5e_v2.xml'
        self.pin_model = DualArmPinocchioModel(pin_mjcf)

        # 动力学MPC - 使用更高的末端执行器跟踪权重
        self.mpc = DualArmDynamicsMPC(
            mjcf_path=pin_mjcf,
            horizon=10,  # 减小horizon加快求解
            dt=0.05,
            weights={
                "ee_left": 10000.0,       # 增加10倍
                "ee_right": 10000.0,
                "state_reg": 0.0001,      # 进一步减少
                "control": 0.001,         # 进一步减少
                "control_rate": 0.0001,
                "terminal_ee_left": 50000.0,   # 增加10倍
                "terminal_ee_right": 50000.0,
                "terminal_state": 0.1,    # 进一步减少
            }
        )

        # 插值器
        self.mpc_dt = 0.05
        self.control_dt = 0.002
        self.interpolation_ratio = int(self.mpc_dt / self.control_dt)
        self.interpolator = TrajectoryInterpolatorSimple(ratio=self.interpolation_ratio)

        # 前馈PD控制器
        gains = FeedforwardPDGains(
            Kp_base_xy=200.0,  Kd_base_xy=50.0,
            Kp_base_z=1000.0,  Kd_base_z=200.0,
            Kp_base_yaw=100.0, Kd_base_yaw=20.0,
            Kp_arm=300.0,      Kd_arm=30.0  # 降低增益，让MPC前馈主导
        )
        self.pd_controller = FeedforwardPDController(gains)

        # 设置力矩限制
        tau_max = np.array([200, 200, 1000, 100,
                           150, 150, 150, 28, 28, 28,
                           150, 150, 150, 28, 28, 28])
        self.pd_controller.set_control_limits(-tau_max, tau_max)

        # 轨迹参数
        self.omega = 0.5
        self.duration = 5.0  # 缩短到5秒用于快速测试
        self.time = 0.0

        # 统计
        self.errors_left = []
        self.errors_right = []
        self.mpc_solve_times = []

    def circle_trajectory(self, t):
        """双臂圆形轨迹（避碰版本）"""
        radius = 0.08
        omega = self.omega
        center_left = np.array([0.600, 0.300, 0.800])
        center_right = np.array([0.600, -0.300, 0.800])

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

    def generate_reference_trajectory(self, t_start):
        """生成MPC的参考轨迹"""
        N = self.mpc.horizon
        target_left_traj = np.zeros((N + 1, 3))
        target_right_traj = np.zeros((N + 1, 3))

        for k in range(N + 1):
            t_k = t_start + k * self.mpc_dt
            target_left_traj[k], target_right_traj[k] = self.circle_trajectory(t_k)

        return target_left_traj, target_right_traj

    def run(self):
        """运行控制循环"""
        print("启动 Phase 6-v3 Step 2: 完整动力学MPC")
        print("=" * 60)
        print(f"架构: ALIGATOR MPC → 插值 → 前馈PD → MuJoCo力矩控制")
        print(f"")
        print(f"参数:")
        print(f"  MPC频率: {1.0/self.mpc_dt:.0f} Hz")
        print(f"  控制频率: {1.0/self.control_dt:.0f} Hz")
        print(f"  MPC horizon: {self.mpc.horizon}")
        print(f"  PD增益: Kp_arm={self.pd_controller.gains.Kp_arm[0]:.0f}, Kd_arm={self.pd_controller.gains.Kd_arm[0]:.0f}")
        print()

        # 初始化
        q_nominal = self.mpc.q_nominal
        v_nominal = self.mpc.v_nominal
        x0 = np.concatenate([q_nominal, v_nominal])

        mujoco.mj_resetData(self.mj_model, self.mj_data)
        self.mj_data.qpos[:16] = q_nominal
        self.mj_data.qvel[:16] = v_nominal
        mujoco.mj_forward(self.mj_model, self.mj_data)

        # 初始化插值器（使用零力矩）
        xs_init = np.tile(x0, (self.interpolation_ratio + 1, 1))
        us_init = np.zeros((self.interpolation_ratio, 16))
        self.interpolator.set_segment(xs_init, us_init)

        # 启动viewer
        with mujoco.viewer.launch_passive(self.mj_model, self.mj_data) as viewer:

            # 获取mocap目标的ID
            left_body_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_BODY, "left_target_body")
            right_body_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_BODY, "right_target_body")
            left_mocap_id = self.mj_model.body_mocapid[left_body_id] if left_body_id >= 0 else -1
            right_mocap_id = self.mj_model.body_mocapid[right_body_id] if right_body_id >= 0 else -1

            mpc_counter = 0
            u_prev_mpc = None  # 初始化为None，后面会设置为完整的(N, 16)数组

            while viewer.is_running() and self.time < self.duration:
                t_start = time.time()

                # 获取当前目标
                target_left, target_right = self.circle_trajectory(self.time)

                # 更新mocap可视化
                if left_mocap_id >= 0:
                    self.mj_data.mocap_pos[left_mocap_id] = target_left
                if right_mocap_id >= 0:
                    self.mj_data.mocap_pos[right_mocap_id] = target_right

                # MPC更新 (20 Hz)
                if mpc_counter % self.interpolation_ratio == 0:
                    # 当前状态
                    q_current = self.mj_data.qpos[:16].copy()
                    v_current = self.mj_data.qvel[:16].copy()
                    x_current = np.concatenate([q_current, v_current])

                    # 生成参考轨迹
                    target_left_traj, target_right_traj = self.generate_reference_trajectory(self.time)

                    # 求解MPC
                    mpc_t_start = time.time()
                    try:
                        xs, us, results = self.mpc.solve(
                            x_current,
                            target_left_traj,
                            target_right_traj,
                            u_init=u_prev_mpc,
                            max_iters=20,  # 增加迭代次数以提高收敛性
                            verbose=False
                        )
                        mpc_solve_time = time.time() - mpc_t_start
                        self.mpc_solve_times.append(mpc_solve_time)

                        # 检查收敛性
                        if not results.conv:
                            # 未收敛：使用 fallback（纯 PD，不使用 MPC 前馈）
                            print(f"[MPC] t={self.time:.2f}s 未收敛，使用 fallback")
                            # 仍然使用 MPC 的 q_des/v_des，但 tau_ff=0
                            xs_segment = xs[:self.interpolation_ratio + 1]
                            us_segment = np.zeros((self.interpolation_ratio, 16))  # 零前馈
                        else:
                            # 收敛：正常使用 MPC 输出
                            xs_segment = xs[:self.interpolation_ratio + 1]
                            us_segment = us[:self.interpolation_ratio]

                        # DEBUG: 检查MPC输出的力矩范围
                        if mpc_counter == 0:
                            print(f"[DEBUG] MPC输出力矩范围:")
                            print(f"  us min: {us.min():.2f}, max: {us.max():.2f}, mean: {np.abs(us).mean():.2f}")
                            print(f"  第一个控制: {us[0]}")

                        self.interpolator.set_segment(xs_segment, us_segment)

                        # 保存完整的us用于下次warm start
                        u_prev_mpc = us  # (N, 16) 数组

                        if mpc_counter % 500 == 0:
                            print(f"[MPC] t={self.time:.2f}s, solve_time={mpc_solve_time*1000:.1f}ms")

                    except Exception as e:
                        import traceback
                        print(f"[MPC] 求解失败: {e}")
                        print(f"[MPC] 详细信息:")
                        traceback.print_exc()
                        if mpc_counter == 0:
                            # 第一次失败就退出
                            return
                        # 使用上一次的轨迹继续

                # 插值 (500 Hz)
                step_in_mpc = mpc_counter % self.interpolation_ratio
                q_des, v_des, tau_ff = self.interpolator.interpolate(step_in_mpc)

                # 前馈PD控制 (τ_total = τ_mpc + τ_pd)
                q_current = self.mj_data.qpos[:16].copy()
                v_current = self.mj_data.qvel[:16].copy()

                tau_control, info = self.pd_controller.compute_control(
                    q_current=q_current,
                    v_current=v_current,
                    q_des=q_des,
                    v_des=v_des,
                    u_feedforward=tau_ff  # 关键：使用MPC输出的力矩前馈
                )

                # 坐标映射：qpos顺序 → MuJoCo ctrl顺序
                # Pinocchio/MPC: [x, y, yaw, z, arms...]
                # MuJoCo ctrl:   [x, y, z, yaw, arms...]
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
        mpc_times = np.array(self.mpc_solve_times) * 1000

        print(f"左臂 RMS: {np.sqrt(np.mean(errors_left**2)):6.2f} cm (最大: {np.max(errors_left):6.2f} cm)")
        print(f"右臂 RMS: {np.sqrt(np.mean(errors_right**2)):6.2f} cm (最大: {np.max(errors_right):6.2f} cm)")
        print(f"平均 RMS: {(np.sqrt(np.mean(errors_left**2)) + np.sqrt(np.mean(errors_right**2)))/2:6.2f} cm")
        print(f"MPC求解时间: {np.mean(mpc_times):.1f} ± {np.std(mpc_times):.1f} ms")

        avg_rms = (np.sqrt(np.mean(errors_left**2)) + np.sqrt(np.mean(errors_right**2)))/2

        print(f"\n对比:")
        print(f"  Phase 6-v2 (Position, IK):      14.5 cm")
        print(f"  Phase 6-v3 Step 1 (Torque, PD):  run test_phase6_v3_step1_simple.py for baseline")
        print(f"  Phase 6-v3 Step 2 (MPC+PD):      {avg_rms:.2f} cm")

        if avg_rms < 1.0:
            print(f"\n✓✓✓ 完美! 平均误差 {avg_rms:.2f} cm < 1 cm")
        elif avg_rms < 2.0:
            print(f"\n✓✓ 优秀! 平均误差 {avg_rms:.2f} cm")
        else:
            print(f"\n✓ 良好: 平均误差 {avg_rms:.2f} cm")


if __name__ == '__main__':
    controller = Phase6V3Complete()
    controller.run()
