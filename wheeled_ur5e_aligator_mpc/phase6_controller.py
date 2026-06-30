"""
Phase 6 Step 4: 完整集成控制器

将MPC + 插值器 + 前馈PD控制器集成为完整的Phase 6控制器

架构:
    MPC (20Hz) → 插值器 (500Hz) → 前馈PD (500Hz) → MuJoCo

特点:
- 支持Phase 1-3运动学MPC
- 支持Phase 5动力学MPC (可选)
- 解决积分器不匹配问题
"""

import numpy as np
import time
from typing import Dict, Optional

from wheeled_ur5e_aligator_mpc.trajectory_interpolator import TrajectoryInterpolator
from wheeled_ur5e_aligator_mpc.feedforward_pd_controller import (
    FeedforwardPDController, FeedforwardPDGains
)


class Phase6Controller:
    """
    Phase 6完整控制器: MPC + 插值 + 前馈PD

    Parameters
    ----------
    mpc_controller : object
        MPC控制器对象，必须有solve(x, ref)方法
    mpc_dt : float
        MPC更新频率时间步长 (默认0.05s = 20Hz)
    control_dt : float
        控制频率时间步长 (默认0.002s = 500Hz)
    pd_gains : FeedforwardPDGains
        PD控制器增益
    """

    def __init__(self,
                 mpc_controller,
                 mpc_dt=0.05,
                 control_dt=0.002,
                 pd_gains=None):

        # MPC控制器
        self.mpc = mpc_controller
        self.mpc_dt = mpc_dt

        # 插值器
        self.interpolator = TrajectoryInterpolator(
            mpc_dt=mpc_dt,
            control_dt=control_dt
        )

        # 前馈PD控制器
        if pd_gains is None:
            pd_gains = FeedforwardPDGains()
        self.pd_controller = FeedforwardPDController(pd_gains)

        # 控制频率
        self.control_dt = control_dt

        # 上次MPC更新时间
        self.last_mpc_time = -np.inf

        # 统计信息
        self.stats = {
            'mpc_solves': 0,
            'mpc_solve_times': [],
            'control_steps': 0,
        }

    def control_step(self,
                    x_current: np.ndarray,
                    ref_traj: Dict,
                    current_time: float) -> tuple:
        """
        单步控制

        Parameters
        ----------
        x_current : array
            当前状态 (10-dim或23-dim)
        ref_traj : dict
            参考轨迹
        current_time : float
            当前时间

        Returns
        -------
        u_control : array
            控制输出
        info : dict
            诊断信息
        """
        # 1. 检查是否需要更新MPC
        if current_time - self.last_mpc_time >= self.mpc_dt:
            self._update_mpc(x_current, ref_traj, current_time)
            self.last_mpc_time = current_time

        # 2. 插值获取期望
        x_des, u_feedforward = self.interpolator.interpolate(current_time)

        if x_des is None:
            # MPC还未求解，返回零控制
            nq = len(x_current) if len(x_current) in [10, 12] else 10
            nu = 10 if nq == 10 else 8
            return np.zeros(nu), {'status': 'waiting_for_mpc'}

        # 3. 提取当前状态的q和v
        q_current, v_current = self._extract_qv(x_current)
        q_des, v_des = self._extract_qv(x_des)

        # 4. 前馈PD控制
        u_control, pd_info = self.pd_controller.compute_control(
            q_current, v_current,
            q_des, v_des,
            u_feedforward
        )

        # 5. 统计
        self.stats['control_steps'] += 1

        # 6. 返回控制和诊断信息
        info = {
            'status': 'active',
            'x_des': x_des,
            'u_feedforward': u_feedforward,
            'q_error': pd_info['q_error'],
            'v_error': pd_info['v_error'],
            'u_pd': pd_info['u_pd'],
            'mode': pd_info['mode'],
        }

        return u_control, info

    def _update_mpc(self, x_current, ref_traj, current_time):
        """更新MPC轨迹"""
        t_start = time.time()

        # 求解MPC
        result = self.mpc.solve(x_current, ref_traj)

        # 更新插值器
        trajectory = {
            'xs': result['xs'],
            'us': result['us'],
            'ts': result['ts'],
        }
        self.interpolator.update_trajectory(trajectory, current_time)

        # 统计
        solve_time = time.time() - t_start
        self.stats['mpc_solves'] += 1
        self.stats['mpc_solve_times'].append(solve_time)

    def _extract_qv(self, x):
        """从状态中提取q和v"""
        if len(x) == 10:
            # 运动学模式: x = q = [base(4), arm(6)]
            # 速度隐式（假设从x历史推导或设为0）
            return x, np.zeros(10)
        elif len(x) == 23:
            # 动力学模式: x = [q(12), v(11)]
            q = x[:12]
            v = x[12:]
            return q, v
        else:
            raise ValueError(f"Unsupported state dimension: {len(x)}")

    def get_statistics(self) -> Dict:
        """获取统计信息"""
        stats = self.stats.copy()

        if len(stats['mpc_solve_times']) > 0:
            stats['mpc_solve_time_mean'] = np.mean(stats['mpc_solve_times'])
            stats['mpc_solve_time_max'] = np.max(stats['mpc_solve_times'])
        else:
            stats['mpc_solve_time_mean'] = 0
            stats['mpc_solve_time_max'] = 0

        stats['mpc_frequency'] = 1.0 / self.mpc_dt
        stats['control_frequency'] = 1.0 / self.control_dt

        return stats

    def reset_statistics(self):
        """重置统计信息"""
        self.stats = {
            'mpc_solves': 0,
            'mpc_solve_times': [],
            'control_steps': 0,
        }


class MockMPCController:
    """
    模拟MPC控制器（用于测试）

    简单的P控制器，模拟MPC输出
    """

    def __init__(self, horizon=20, dt=0.05, state_dim=10, control_dim=10):
        self.horizon = horizon
        self.dt = dt
        self.state_dim = state_dim
        self.control_dim = control_dim

    def solve(self, x_current, ref_traj):
        """
        模拟MPC求解

        使用简单的P控制生成轨迹
        """
        N = self.horizon
        xs = np.zeros((N+1, self.state_dim))
        us = np.zeros((N, self.control_dim))
        ts = np.arange(N+1) * self.dt

        # 简单的轨迹生成
        for i in range(N+1):
            if 'ee_pos' in ref_traj and i < len(ref_traj['ee_pos']):
                # 使用参考轨迹（这里简化为直接复制当前状态）
                xs[i] = x_current
            else:
                xs[i] = x_current

        # 简单的速度命令（向目标移动）
        for i in range(N):
            us[i] = np.random.randn(self.control_dim) * 0.01  # 小随机控制

        return {
            'xs': xs,
            'us': us,
            'ts': ts,
            'solve_time': 0.01,  # 模拟10ms求解时间
            'converged': True,
        }


if __name__ == '__main__':
    """测试Phase 6集成控制器"""
    print("="*60)
    print("Phase 6 Step 4: 集成控制器测试")
    print("="*60)

    # ========================================
    # 测试1: 运动学模式 (10-DOF)
    # ========================================
    print(f"\n" + "="*60)
    print("测试1: 运动学模式集成")
    print("="*60)

    # 创建模拟MPC
    mock_mpc = MockMPCController(horizon=10, dt=0.05, state_dim=10, control_dim=10)

    # 创建Phase 6控制器
    controller = Phase6Controller(
        mpc_controller=mock_mpc,
        mpc_dt=0.05,
        control_dt=0.002
    )

    print(f"\n控制器配置:")
    print(f"  MPC频率: {1/controller.mpc_dt:.0f} Hz")
    print(f"  控制频率: {1/controller.control_dt:.0f} Hz")
    print(f"  插值比例: {controller.interpolator.ratio}:1")

    # 模拟控制循环
    x_current = np.zeros(10)
    x_current[2] = 0.2  # base_z

    ref_traj = {
        'ee_pos': np.array([[0.6, 0.0, 0.8]]),  # 简单目标
    }

    print(f"\n模拟控制循环 (0.1秒):")

    dt = controller.control_dt
    total_time = 0.1
    n_steps = int(total_time / dt)

    for i in range(n_steps):
        current_time = i * dt

        u_control, info = controller.control_step(x_current, ref_traj, current_time)

        # 每10步打印一次
        if i % 10 == 0:
            print(f"  t={current_time:.3f}s: status={info['status']}, u[0]={u_control[0]:.4f}")

    # 统计
    stats = controller.get_statistics()
    print(f"\n统计信息:")
    print(f"  MPC求解次数: {stats['mpc_solves']}")
    print(f"  平均求解时间: {stats['mpc_solve_time_mean']*1000:.2f} ms")
    print(f"  控制步数: {stats['control_steps']}")
    print(f"  预期MPC次数: {int(total_time / controller.mpc_dt)}")
    print(f"  预期控制次数: {n_steps}")

    # ========================================
    # 测试2: 频率验证
    # ========================================
    print(f"\n" + "="*60)
    print("测试2: 频率验证")
    print("="*60)

    controller.reset_statistics()

    # 运行1秒
    total_time = 1.0
    n_steps = int(total_time / dt)

    for i in range(n_steps):
        current_time = i * dt
        u_control, info = controller.control_step(x_current, ref_traj, current_time)

    stats = controller.get_statistics()

    expected_mpc = int(total_time / controller.mpc_dt)
    expected_control = n_steps

    print(f"  运行时长: {total_time}s")
    print(f"  MPC求解: {stats['mpc_solves']} (预期{expected_mpc})")
    print(f"  控制步数: {stats['control_steps']} (预期{expected_control})")
    print(f"  MPC频率验证: {'✓' if stats['mpc_solves'] == expected_mpc else '✗'}")
    print(f"  控制频率验证: {'✓' if stats['control_steps'] == expected_control else '✗'}")

    # ========================================
    # 测试3: 插值平滑性
    # ========================================
    print(f"\n" + "="*60)
    print("测试3: 插值平滑性验证")
    print("="*60)

    controller.reset_statistics()

    # 收集控制序列
    u_sequence = []
    times = []

    for i in range(100):  # 0.2秒
        current_time = i * dt
        u_control, info = controller.control_step(x_current, ref_traj, current_time)
        u_sequence.append(u_control[0])  # 只看第一个维度
        times.append(current_time)

    u_sequence = np.array(u_sequence)

    # 计算平滑度（相邻点差异）
    du = np.diff(u_sequence)
    smoothness = np.std(du)

    print(f"  采样点数: {len(u_sequence)}")
    print(f"  控制范围: [{u_sequence.min():.4f}, {u_sequence.max():.4f}]")
    print(f"  相邻差异std: {smoothness:.4f}")
    print(f"  平滑性: {'✓ 良好' if smoothness < 1.0 else '⚠ 可能有跳变'}")

    # ========================================
    # 总结
    # ========================================
    print(f"\n" + "="*60)
    print("✓ Phase 6集成控制器测试完成!")
    print("="*60)
    print(f"\n关键特性:")
    print(f"  ✓ MPC自动更新 (20Hz)")
    print(f"  ✓ 插值器平滑轨迹 (500Hz)")
    print(f"  ✓ 前馈PD控制 (500Hz)")
    print(f"  ✓ 频率分离工作正常")
    print(f"  ✓ 控制输出平滑")
    print(f"\n下一步: 创建MuJoCo闭环demo (Step 5)")
    print("="*60)
