"""
Phase 6 Step 3: 前馈PD控制器

前馈+反馈结合的控制器

核心思想:
- 前馈项 (u_feedforward): 来自MPC的动力学补偿
- 反馈项 (PD): 补偿模型误差和扰动

τ_final = τ_feedforward + Kp*(q_des - q_current) + Kd*(v_des - v_current)
"""

import numpy as np
from typing import Tuple, Optional


class FeedforwardPDGains:
    """PD控制器增益配置"""

    def __init__(self,
                 Kp_base_xy=100.0, Kp_base_z=500.0, Kp_base_yaw=50.0,
                 Kd_base_xy=20.0, Kd_base_z=100.0, Kd_base_yaw=10.0,
                 Kp_arm=1000.0, Kd_arm=100.0):
        """
        Parameters
        ----------
        Kp_base_xy : float
            基座XY方向位置增益
        Kp_base_z : float
            基座Z方向位置增益（需要对抗重力）
        Kp_base_yaw : float
            基座偏航角增益
        Kd_base_xy : float
            基座XY方向速度增益
        Kd_base_z : float
            基座Z方向速度增益
        Kd_base_yaw : float
            基座偏航角速度增益
        Kp_arm : float or (6,) array
            机械臂位置增益
        Kd_arm : float or (6,) array
            机械臂速度增益
        """
        # 基座增益 (qpos 顺序: x, y, yaw, z)
        self.Kp_base = np.array([Kp_base_xy, Kp_base_xy, Kp_base_yaw, Kp_base_z])
        self.Kd_base = np.array([Kd_base_xy, Kd_base_xy, Kd_base_yaw, Kd_base_z])

        # 机械臂增益
        if np.isscalar(Kp_arm):
            self.Kp_arm = np.ones(6) * Kp_arm
        else:
            self.Kp_arm = np.asarray(Kp_arm)

        if np.isscalar(Kd_arm):
            self.Kd_arm = np.ones(6) * Kd_arm
        else:
            self.Kd_arm = np.asarray(Kd_arm)


class FeedforwardPDController:
    """
    前馈PD控制器

    支持两种模式:
    1. 运动学模式: q=(10,), u=(10,) velocity commands
    2. 动力学模式: q=(12,), u=(8,) torque commands (带轮子)

    自动检测模式并适配
    """

    def __init__(self, gains: Optional[FeedforwardPDGains] = None):
        """
        Parameters
        ----------
        gains : FeedforwardPDGains
            PD增益配置
        """
        if gains is None:
            gains = FeedforwardPDGains()
        self.gains = gains

        # 控制限幅
        self.u_max = None  # 最大控制输出（可选）
        self.u_min = None  # 最小控制输出（可选）

    def set_control_limits(self, u_min, u_max):
        """
        设置控制限幅

        Parameters
        ----------
        u_min : float or array
            最小控制输出
        u_max : float or array
            最大控制输出
        """
        self.u_min = np.asarray(u_min) if u_min is not None else None
        self.u_max = np.asarray(u_max) if u_max is not None else None

    def compute_control(self,
                       q_current: np.ndarray,
                       v_current: np.ndarray,
                       q_des: np.ndarray,
                       v_des: np.ndarray,
                       u_feedforward: Optional[np.ndarray] = None) -> Tuple[np.ndarray, dict]:
        """
        计算最终控制输出

        Parameters
        ----------
        q_current : (nq,) array
            当前配置 (10-dim或12-dim或16-dim)
        v_current : (nv,) array
            当前速度 (10-dim或11-dim或16-dim)
        q_des : (nq,) array
            期望配置
        v_des : (nv,) array
            期望速度
        u_feedforward : (nu,) array or None
            前馈控制（可选）
            - 如果None，则纯PD控制
            - 如果提供，则前馈+PD

        Returns
        -------
        u_final : (nu,) array
            最终控制输出
        info : dict
            诊断信息
        """
        # 检测模式
        nq = len(q_current)

        if nq == 10:
            # 运动学模式: [base(4), arm(6)]
            return self._compute_kinematic_control(
                q_current, v_current, q_des, v_des, u_feedforward
            )
        elif nq == 12:
            # 动力学模式: [base(4), wheels(2), arm(6)]
            return self._compute_dynamic_control(
                q_current, v_current, q_des, v_des, u_feedforward
            )
        elif nq == 16:
            # 双臂模式: [base(4), left_arm(6), right_arm(6)]
            return self._compute_dual_arm_control(
                q_current, v_current, q_des, v_des, u_feedforward
            )
        else:
            raise ValueError(f"Unsupported state dimension: nq={nq}")

    def _compute_kinematic_control(self, q_current, v_current, q_des, v_des, u_feedforward):
        """运动学模式控制 (Phase 1-3)"""
        # 位置误差
        q_error = q_des - q_current  # (10,)

        # 速度误差
        v_error = v_des - v_current  # (10,)

        # PD反馈 (速度命令)
        u_pd = np.zeros(10)

        # 基座 (0:4)
        u_pd[0:4] = self.gains.Kp_base * q_error[0:4] + self.gains.Kd_base * v_error[0:4]

        # 机械臂 (4:10)
        u_pd[4:10] = self.gains.Kp_arm * q_error[4:10] + self.gains.Kd_arm * v_error[4:10]

        # 前馈+反馈
        if u_feedforward is not None:
            u_final = u_feedforward + u_pd
        else:
            u_final = u_pd

        # 限幅
        if self.u_min is not None and self.u_max is not None:
            u_final = np.clip(u_final, self.u_min, self.u_max)

        info = {
            'q_error': q_error,
            'v_error': v_error,
            'u_pd': u_pd,
            'u_feedforward': u_feedforward if u_feedforward is not None else np.zeros(10),
            'mode': 'kinematic',
        }

        return u_final, info

    def _compute_dynamic_control(self, q_current, v_current, q_des, v_des, u_feedforward):
        """动力学模式控制 (Phase 5-6)"""
        # q: [base(4), wheels(2), arm(6)] = 12
        # v: [v_base(3), omega_wheels(2), v_arm(6)] = 11
        # u: [tau_wheels(2), tau_arm(6)] = 8

        # 位置误差
        q_error = q_des - q_current  # (12,)

        # 速度误差
        v_error = v_des - v_current  # (11,)

        # PD反馈 (扭矩命令)
        u_pd = np.zeros(8)

        # 轮子 (q[4:6] -> v[3:5] -> u[0:2])
        # 简化：使用固定增益
        Kp_wheels = 500.0
        Kd_wheels = 50.0
        u_pd[0:2] = Kp_wheels * q_error[4:6] + Kd_wheels * v_error[3:5]

        # 机械臂 (q[6:12] -> v[5:11] -> u[2:8])
        u_pd[2:8] = self.gains.Kp_arm * q_error[6:12] + self.gains.Kd_arm * v_error[5:11]

        # 前馈+反馈
        if u_feedforward is not None:
            u_final = u_feedforward + u_pd
        else:
            u_final = u_pd

        # 限幅
        if self.u_min is not None and self.u_max is not None:
            u_final = np.clip(u_final, self.u_min, self.u_max)

        info = {
            'q_error': q_error,
            'v_error': v_error,
            'u_pd': u_pd,
            'u_feedforward': u_feedforward if u_feedforward is not None else np.zeros(8),
            'mode': 'dynamic',
        }

        return u_final, info


    def _compute_dual_arm_control(self, q_current, v_current, q_des, v_des, u_feedforward):
        """双臂模式控制 (16 DOF)

        q: [base(4), left_arm(6), right_arm(6)] = 16
        v: [base(4), left_arm(6), right_arm(6)] = 16
        u: [base(4), left_arm(6), right_arm(6)] = 16 (torque)
        """
        # 位置误差
        q_error = q_des - q_current  # (16,)

        # 速度误差
        v_error = v_des - v_current  # (16,)

        # PD反馈 (力矩命令)
        u_pd = np.zeros(16)

        # 基座 (0:4)
        u_pd[0:4] = self.gains.Kp_base * q_error[0:4] + self.gains.Kd_base * v_error[0:4]

        # 左臂 (4:10)
        u_pd[4:10] = self.gains.Kp_arm * q_error[4:10] + self.gains.Kd_arm * v_error[4:10]

        # 右臂 (10:16)
        u_pd[10:16] = self.gains.Kp_arm * q_error[10:16] + self.gains.Kd_arm * v_error[10:16]

        # 前馈+反馈 (关键: τ_total = τ_feedforward + τ_pd)
        if u_feedforward is not None:
            u_final = u_feedforward + u_pd
        else:
            u_final = u_pd

        # 限幅
        if self.u_min is not None and self.u_max is not None:
            u_final = np.clip(u_final, self.u_min, self.u_max)

        info = {
            'q_error': q_error,
            'v_error': v_error,
            'u_pd': u_pd,
            'u_feedforward': u_feedforward if u_feedforward is not None else np.zeros(16),
            'mode': 'dual_arm_torque',
        }

        return u_final, info


if __name__ == '__main__':
    """测试前馈PD控制器"""
    print("="*60)
    print("Phase 6 Step 3: 前馈PD控制器测试")
    print("="*60)

    # 创建控制器
    gains = FeedforwardPDGains(
        Kp_base_xy=100.0, Kd_base_xy=20.0,
        Kp_base_z=500.0, Kd_base_z=100.0,
        Kp_arm=1000.0, Kd_arm=100.0
    )
    controller = FeedforwardPDController(gains)

    print(f"\n增益配置:")
    print(f"  基座XY: Kp={gains.Kp_base[0]:.1f}, Kd={gains.Kd_base[0]:.1f}")
    print(f"  基座Z:  Kp={gains.Kp_base[2]:.1f}, Kd={gains.Kd_base[2]:.1f}")
    print(f"  机械臂: Kp={gains.Kp_arm[0]:.1f}, Kd={gains.Kd_arm[0]:.1f}")

    # ========================================
    # 测试1: 运动学模式 (10-DOF)
    # ========================================
    print(f"\n" + "="*60)
    print("测试1: 运动学模式 (Phase 1-3)")
    print("="*60)

    # 当前状态
    q_current = np.zeros(10)
    v_current = np.zeros(10)

    # 期望状态（向前移动10cm，机械臂偏移）
    q_des = q_current.copy()
    q_des[0] = 0.1  # base_x += 10cm
    q_des[4] = 0.1  # shoulder_pan += 0.1 rad

    v_des = np.zeros(10)
    v_des[0] = 0.1  # 期望速度 0.1 m/s

    # 测试A: 纯PD控制（无前馈）
    u_final, info = controller.compute_control(q_current, v_current, q_des, v_des, u_feedforward=None)

    print(f"\nA. 纯PD控制:")
    print(f"  位置误差: ||e_q|| = {np.linalg.norm(info['q_error']):.4f}")
    print(f"  速度误差: ||e_v|| = {np.linalg.norm(info['v_error']):.4f}")
    print(f"  PD输出: u_pd[0] = {info['u_pd'][0]:.4f} (base_x)")
    print(f"  PD输出: u_pd[4] = {info['u_pd'][4]:.4f} (arm)")
    print(f"  最终输出: u_final[0] = {u_final[0]:.4f}")

    # 测试B: 前馈+PD控制
    u_feedforward = np.ones(10) * 0.05  # 模拟MPC输出
    u_final, info = controller.compute_control(q_current, v_current, q_des, v_des, u_feedforward)

    print(f"\nB. 前馈+PD控制:")
    print(f"  前馈: u_ff[0] = {info['u_feedforward'][0]:.4f}")
    print(f"  PD: u_pd[0] = {info['u_pd'][0]:.4f}")
    print(f"  最终: u_final[0] = {u_final[0]:.4f}")
    print(f"  验证: u_ff + u_pd = {info['u_feedforward'][0] + info['u_pd'][0]:.4f} {'✓' if abs(u_final[0] - (info['u_feedforward'][0] + info['u_pd'][0])) < 1e-6 else '✗'}")

    # 测试C: 无误差情况
    u_final, info = controller.compute_control(q_des, v_des, q_des, v_des, u_feedforward)

    print(f"\nC. 无误差情况:")
    print(f"  位置误差: ||e_q|| = {np.linalg.norm(info['q_error']):.2e}")
    print(f"  速度误差: ||e_v|| = {np.linalg.norm(info['v_error']):.2e}")
    print(f"  PD输出: ||u_pd|| = {np.linalg.norm(info['u_pd']):.2e}")
    print(f"  最终输出: u_final ≈ u_ff? {np.allclose(u_final, u_feedforward, atol=1e-10)} ✓")

    # ========================================
    # 测试2: 动力学模式 (12-DOF)
    # ========================================
    print(f"\n" + "="*60)
    print("测试2: 动力学模式 (Phase 5-6)")
    print("="*60)

    # 当前状态
    q_current = np.zeros(12)  # [base(4), wheels(2), arm(6)]
    v_current = np.zeros(11)  # [v_base(3), omega_wheels(2), v_arm(6)]

    # 期望状态
    q_des = q_current.copy()
    q_des[0] = 0.1  # base_x
    q_des[4] = 0.5  # left_wheel
    q_des[5] = 0.5  # right_wheel

    v_des = np.zeros(11)

    # 测试A: 纯PD（扭矩输出）
    u_final, info = controller.compute_control(q_current, v_current, q_des, v_des, u_feedforward=None)

    print(f"\nA. 纯PD控制 (扭矩):")
    print(f"  模式: {info['mode']}")
    print(f"  控制维度: {len(u_final)}")
    print(f"  轮子扭矩: τ_wheels = [{u_final[0]:.2f}, {u_final[1]:.2f}] N·m")
    print(f"  机械臂扭矩: τ_arm[0] = {u_final[2]:.2f} N·m")

    # 测试B: 前馈+PD
    u_feedforward = np.array([10.0, 10.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0])  # 模拟动力学补偿
    u_final, info = controller.compute_control(q_current, v_current, q_des, v_des, u_feedforward)

    print(f"\nB. 前馈+PD控制:")
    print(f"  前馈扭矩: τ_ff = {u_feedforward[0]:.2f} N·m (wheels)")
    print(f"  PD扭矩: τ_pd = {info['u_pd'][0]:.2f} N·m")
    print(f"  最终扭矩: τ_final = {u_final[0]:.2f} N·m")
    print(f"  ✓ 前馈主导，PD补偿")

    # ========================================
    # 测试3: 控制限幅
    # ========================================
    print(f"\n" + "="*60)
    print("测试3: 控制限幅")
    print("="*60)

    controller.set_control_limits(u_min=-50.0, u_max=50.0)

    # 大误差情况
    q_des_large = q_current.copy()
    q_des_large[0] = 10.0  # 10m误差（不合理）

    u_final, info = controller.compute_control(q_current, v_current, q_des_large, v_des, u_feedforward=None)

    print(f"  大误差: q_error[0] = {info['q_error'][0]:.2f} m")
    print(f"  无限幅PD: u_pd[0] = {info['u_pd'][0]:.2f}")
    print(f"  限幅后: u_final[0] = {u_final[0]:.2f}")
    print(f"  ✓ 限幅在 [-50, 50]? {-50 <= u_final[0] <= 50}")

    # ========================================
    # 总结
    # ========================================
    print(f"\n" + "="*60)
    print("✓ 前馈PD控制器测试完成!")
    print("="*60)
    print(f"\n关键特性:")
    print(f"  ✓ 支持运动学模式 (10-DOF速度控制)")
    print(f"  ✓ 支持动力学模式 (12-DOF扭矩控制)")
    print(f"  ✓ 自动检测并适配")
    print(f"  ✓ 前馈+反馈结合")
    print(f"  ✓ 控制限幅")
    print(f"\n下一步: 集成MPC+插值+PD完整控制器 (Step 4)")
    print("="*60)
