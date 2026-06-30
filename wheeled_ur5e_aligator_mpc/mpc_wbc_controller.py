"""
Phase 6: MPC+WBC完整控制器 (升级版 - 使用Kino-Dynamic MPC)

双层控制架构：
- MPC层：轨迹规划（使用Kino-Dynamic MPC，包含动力学）
- WBC层：力矩求解（QP）

关键改进：
- 旧版：运动学MPC + 差分估计加速度 ❌
- 新版：Kino-dynamic MPC + ABA精确加速度 ✅
"""

import numpy as np
import time
from .wbc_controller import WholeBodyController
from .mpc_wbc_interface import MPCWBCInterface
from .kinodynamic_mpc_controller import KinoDynamicMPCController
from .robot_model import WheeledUR5eModel


class MPCWBCController:
    """
    MPC+WBC双层控制器 (Kino-Dynamic版本)

    频率：
    - MPC: 20 Hz (0.05s)
    - WBC: 100 Hz (0.01s)
    """

    def __init__(self, pin_robot, wheel_params=None, wbc_weights=None,
                 mpc_horizon=20, mpc_weights=None):
        """
        Parameters
        ----------
        pin_robot : PinocchioWheeledUR5eModel
            Pinocchio模型
        wheel_params : WheelParameters, optional
            轮子参数
        wbc_weights : WBCWeights, optional
            WBC权重
        mpc_horizon : int
            MPC预测时域
        mpc_weights : dict, optional
            MPC代价权重
        """
        self.pin_robot = pin_robot

        # WBC控制器
        self.wbc = WholeBodyController(pin_robot, wheel_params, wbc_weights)

        # ✅ Kino-Dynamic MPC控制器 (新版)
        robot_model = WheeledUR5eModel()
        self.kinodynamic_mpc = KinoDynamicMPCController(
            robot=robot_model,
            pin_robot=pin_robot,
            horizon=mpc_horizon,
            dt=0.05,
            weights=mpc_weights,
            max_iters=50,
        )

        # MPC-WBC接口 (新版 - 支持加速度直接获取)
        self.interface = MPCWBCInterface(
            kinodynamic_mpc=self.kinodynamic_mpc,
            wbc_dt=0.01
        )

        # 统计信息
        self.stats = {
            "mpc_calls": 0,
            "wbc_calls": 0,
            "mpc_times": [],
            "wbc_times": [],
            "ee_errors": [],
        }

    def control_step(self, x_wbc, ref_traj, t):
        """
        单步控制 (Kino-Dynamic版本)

        Parameters
        ----------
        x_wbc : (23,) array
            当前WBC状态
        ref_traj : dict
            参考轨迹
        t : float
            当前时间

        Returns
        -------
        τ_opt : (8,) array
            最优扭矩
        info : dict
            控制信息
        """
        # 1. 检查是否需要更新MPC
        if self.interface.should_update_mpc(t):
            # ✅ 将WBC状态转换为Kino-Dynamic状态
            # WBC: [q_base(4), θ_wheels(2), q_arm(6), v_base(3), ω_wheels(2), v_arm(6)]
            # Kino: [q_base(4), q_arm(6), v_arm(6)]
            x_kinodyn = np.concatenate([
                x_wbc[0:4],      # q_base
                x_wbc[6:12],     # q_arm
                x_wbc[17:23],    # v_arm
            ])

            # ✅ 调用Kino-Dynamic MPC
            t_mpc_start = time.perf_counter()
            u_mpc, mpc_trajectory, mpc_info = self.kinodynamic_mpc.solve(
                x_current=x_kinodyn,
                ref_traj=ref_traj,
                u_prev=None,
            )
            t_mpc = time.perf_counter() - t_mpc_start

            # 缓存MPC轨迹 (包含准确的加速度！)
            self.interface.update_mpc_trajectory(mpc_trajectory, t)

            self.stats["mpc_calls"] += 1
            self.stats["mpc_times"].append(t_mpc * 1000)
            self.stats["mpc_info"] = mpc_info

        # 2. ✅ 从MPC轨迹直接获取期望加速度 (关键改进)
        a_des = self.interface.get_desired_acceleration_from_mpc(t)

        # 3. WBC求解扭矩
        t_wbc_start = time.perf_counter()
        τ_opt, wbc_info = self.wbc.compute_control(x_wbc, a_des)
        t_wbc = time.perf_counter() - t_wbc_start

        self.stats["wbc_calls"] += 1
        self.stats["wbc_times"].append(t_wbc * 1000)

        # 4. 组装信息
        info = {
            "wbc_info": wbc_info,
            "a_des": a_des,
            "mpc_info": self.stats.get("mpc_info", {}),
        }

        return τ_opt, info

    def _simple_mpc_control(self, x_mpc, ref_traj, t):
        """
        简化MPC控制（用于Phase 6初步测试）

        Phase 6完整版会复用Phase 1-3的运动学MPC
        现在使用简单的P控制器

        Parameters
        ----------
        x_mpc : (10,) array
            MPC状态 [q_base(4), q_arm(6)]
        ref_traj : dict
            参考轨迹
        t : float
            当前时间

        Returns
        -------
        u_mpc : (10,) array
            MPC控制 [vx_body, vy_body, vz, ω_yaw, v_arm(6)]
        """
        # 获取参考EE位置
        idx = int(t / 0.05)
        if idx >= len(ref_traj["ee_pos"]):
            idx = -1

        ee_ref = ref_traj["ee_pos"][idx]

        # 计算当前EE位置（使用Pinocchio FK）
        from .pinocchio_model import PinocchioWheeledUR5eModel
        import pinocchio as pin

        pin_robot = PinocchioWheeledUR5eModel()

        q_full = np.zeros(10)
        q_full[:4] = x_mpc[:4]
        q_full[4:10] = x_mpc[4:10]

        ee_current, _ = pin_robot.fk_pose(q_full)

        # 简单P控制
        kp = 1.0
        ee_error = ee_ref - ee_current

        # 转换为基座和机械臂速度
        u_mpc = np.zeros(10)

        # 基座：只用x和y方向（简化）
        u_mpc[0] = kp * ee_error[0]  # vx_body
        u_mpc[1] = 0.0  # vy_body = 0（非完整约束）
        u_mpc[2] = 0.0  # vz = 0
        u_mpc[3] = 0.0  # ω_yaw = 0

        # 机械臂：简化控制（比例控制到笛卡尔空间误差）
        # 只控制前3个关节（肩和肘）
        u_mpc[4] = kp * ee_error[0] * 0.5  # shoulder_pan
        u_mpc[5] = kp * ee_error[2] * 0.5  # shoulder_lift
        u_mpc[6] = kp * ee_error[1] * 0.5  # elbow
        u_mpc[7:10] = 0.0  # 腕关节不动

        # 限制速度
        u_mpc = np.clip(u_mpc, -0.5, 0.5)

        return u_mpc

    def print_stats(self):
        """打印统计信息"""
        print("\n" + "="*80)
        print("MPC+WBC控制器统计")
        print("="*80)

        print(f"\nMPC调用次数: {self.stats['mpc_calls']}")
        print(f"WBC调用次数: {self.stats['wbc_calls']}")

        if self.stats["wbc_times"]:
            wbc_times = np.array(self.stats["wbc_times"])
            print(f"\nWBC求解时间:")
            print(f"  平均: {np.mean(wbc_times):.2f} ms")
            print(f"  最大: {np.max(wbc_times):.2f} ms")
            print(f"  最小: {np.min(wbc_times):.2f} ms")

        if self.stats["ee_errors"]:
            ee_errors = np.array(self.stats["ee_errors"])
            print(f"\nEE跟踪误差:")
            print(f"  RMS: {np.sqrt(np.mean(ee_errors**2))*100:.2f} cm")
            print(f"  平均: {np.mean(ee_errors)*100:.2f} cm")
            print(f"  最大: {np.max(ee_errors)*100:.2f} cm")
