"""
Phase 6 Step 2: 轨迹插值器

将MPC轨迹插值到MuJoCo控制频率

核心功能:
- MPC: 0.05s步长 (20 Hz)
- MuJoCo: 0.002s步长 (500 Hz)
- 插值比例: 25:1
- 支持Phase 1-3运动学MPC和Phase 5动力学MPC
"""

import numpy as np
from typing import Dict, Optional, Tuple


class TrajectoryInterpolator:
    """
    MPC轨迹插值器

    将MPC输出的粗时间步轨迹插值到精细控制频率

    Parameters
    ----------
    mpc_dt : float
        MPC时间步长 (默认0.05s = 20Hz)
    control_dt : float
        控制频率时间步长 (默认0.002s = 500Hz)
    interpolation_method : str
        插值方法: 'linear' 或 'cubic' (暂时只支持linear)
    """

    def __init__(self, mpc_dt=0.05, control_dt=0.002, interpolation_method='linear'):
        self.mpc_dt = mpc_dt
        self.control_dt = control_dt
        self.method = interpolation_method

        # 插值比例
        self.ratio = int(mpc_dt / control_dt)

        # 当前缓存的MPC轨迹
        self.trajectory = None
        self.trajectory_start_time = None

    def update_trajectory(self, trajectory: Dict, current_time: float):
        """
        更新MPC轨迹

        Parameters
        ----------
        trajectory : dict
            MPC输出的轨迹，格式:
            {
                'xs': (N+1, nx) 状态轨迹
                'us': (N, nu) 控制轨迹
                'ts': (N+1,) 时间点
            }
        current_time : float
            当前时间
        """
        self.trajectory = {
            'xs': np.asarray(trajectory['xs']),
            'us': np.asarray(trajectory['us']),
            'ts': np.asarray(trajectory['ts']),
        }
        self.trajectory_start_time = current_time

    def interpolate(self, current_time: float) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        插值获取当前时刻的期望状态和前馈控制

        Parameters
        ----------
        current_time : float
            当前时间

        Returns
        -------
        x_des : (nx,) array or None
            期望状态，如果轨迹未设置则返回None
        u_feedforward : (nu,) array or None
            前馈控制，如果轨迹未设置则返回None
        """
        if self.trajectory is None:
            return None, None

        # 计算相对时间
        t_rel = current_time - self.trajectory_start_time

        # 边界处理
        ts = self.trajectory['ts']
        if t_rel < 0:
            # 时间还未到轨迹开始
            return self.trajectory['xs'][0], self.trajectory['us'][0]
        elif t_rel >= ts[-1]:
            # 超出轨迹范围，返回最后一个点
            return self.trajectory['xs'][-1], self.trajectory['us'][-1]

        # 线性插值
        if self.method == 'linear':
            return self._linear_interpolate(t_rel)
        else:
            raise NotImplementedError(f"Interpolation method '{self.method}' not implemented")

    def _linear_interpolate(self, t_rel: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        线性插值

        Parameters
        ----------
        t_rel : float
            相对于轨迹起点的时间

        Returns
        -------
        x_des : (nx,) array
            插值后的期望状态
        u_feedforward : (nu,) array
            插值后的前馈控制
        """
        xs = self.trajectory['xs']
        us = self.trajectory['us']
        ts = self.trajectory['ts']

        # 找到插值区间 [ts[idx], ts[idx+1]]
        idx = np.searchsorted(ts, t_rel) - 1
        idx = max(0, min(idx, len(ts) - 2))

        # 插值权重
        t0, t1 = ts[idx], ts[idx + 1]
        if t1 - t0 < 1e-9:
            # 避免除零
            alpha = 0.0
        else:
            alpha = (t_rel - t0) / (t1 - t0)
        alpha = np.clip(alpha, 0.0, 1.0)

        # 状态插值
        x_des = (1 - alpha) * xs[idx] + alpha * xs[idx + 1]

        # 控制插值
        u_feedforward = (1 - alpha) * us[idx] + alpha * us[idx + 1]

        return x_des, u_feedforward

    def get_interpolation_info(self) -> Dict:
        """
        获取插值器信息

        Returns
        -------
        info : dict
            插值器状态信息
        """
        if self.trajectory is None:
            return {
                'has_trajectory': False,
                'mpc_dt': self.mpc_dt,
                'control_dt': self.control_dt,
                'ratio': self.ratio,
            }

        return {
            'has_trajectory': True,
            'mpc_dt': self.mpc_dt,
            'control_dt': self.control_dt,
            'ratio': self.ratio,
            'trajectory_length': len(self.trajectory['ts']),
            'trajectory_duration': self.trajectory['ts'][-1],
            'state_dim': self.trajectory['xs'].shape[1],
            'control_dim': self.trajectory['us'].shape[1],
        }


if __name__ == '__main__':
    """简单测试"""
    print("="*60)
    print("Phase 6 Step 2: 轨迹插值器测试")
    print("="*60)

    # 创建插值器
    interpolator = TrajectoryInterpolator(mpc_dt=0.05, control_dt=0.002)

    print(f"\n插值器参数:")
    print(f"  MPC频率: {1/interpolator.mpc_dt:.0f} Hz ({interpolator.mpc_dt}s)")
    print(f"  控制频率: {1/interpolator.control_dt:.0f} Hz ({interpolator.control_dt}s)")
    print(f"  插值比例: {interpolator.ratio}:1")

    # 创建模拟MPC轨迹
    N = 20  # MPC horizon
    nx = 10  # 状态维度
    nu = 10  # 控制维度

    ts = np.arange(N+1) * interpolator.mpc_dt
    xs = np.random.randn(N+1, nx)
    us = np.random.randn(N, nu)

    trajectory = {'xs': xs, 'us': us, 'ts': ts}

    print(f"\n模拟MPC轨迹:")
    print(f"  Horizon: {N}")
    print(f"  状态维度: {nx}")
    print(f"  控制维度: {nu}")
    print(f"  轨迹时长: {ts[-1]:.2f}s")

    # 更新轨迹
    current_time = 0.0
    interpolator.update_trajectory(trajectory, current_time)

    print(f"\n✓ 轨迹已更新")
    info = interpolator.get_interpolation_info()
    print(f"  插值器状态: {info['has_trajectory']}")
    print(f"  轨迹长度: {info['trajectory_length']} 步")

    # 测试插值
    print(f"\n插值测试:")
    test_times = [0.0, 0.025, 0.05, 0.1, 0.5, 1.0]

    for t in test_times:
        x_des, u_ff = interpolator.interpolate(current_time + t)
        if x_des is not None:
            print(f"  t={t:.3f}s: x_des shape={x_des.shape}, u_ff shape={u_ff.shape}")

            # 验证插值结果在合理范围内
            x_min, x_max = xs.min(), xs.max()
            assert x_des.min() >= x_min - 1e-6 and x_des.max() <= x_max + 1e-6, \
                f"插值状态超出范围! [{x_min}, {x_max}] vs [{x_des.min()}, {x_des.max()}]"
        else:
            print(f"  t={t:.3f}s: 无轨迹")

    # 测试高频插值
    print(f"\n高频插值测试 (模拟MuJoCo控制循环):")
    control_times = np.arange(0, 0.1, interpolator.control_dt)

    success_count = 0
    for t in control_times:
        x_des, u_ff = interpolator.interpolate(current_time + t)
        if x_des is not None:
            success_count += 1

    print(f"  测试点数: {len(control_times)}")
    print(f"  成功插值: {success_count}")
    print(f"  成功率: {success_count/len(control_times)*100:.1f}%")

    # 测试边界情况
    print(f"\n边界测试:")

    # 1. 轨迹开始前
    x_des, u_ff = interpolator.interpolate(current_time - 0.1)
    print(f"  轨迹前 (t=-0.1s): {'✓ 返回首点' if x_des is not None else '✗ 失败'}")

    # 2. 轨迹结束后
    x_des, u_ff = interpolator.interpolate(current_time + ts[-1] + 0.1)
    print(f"  轨迹后 (t={ts[-1]+0.1:.2f}s): {'✓ 返回末点' if x_des is not None else '✗ 失败'}")

    # 3. 轨迹精确点
    x_des, u_ff = interpolator.interpolate(current_time + ts[5])
    x_exact = xs[5]
    error = np.linalg.norm(x_des - x_exact)
    print(f"  精确点 (t={ts[5]:.2f}s): 误差={error:.2e} {'✓' if error < 1e-10 else '✗'}")

    # 4. 中间点
    t_mid = (ts[5] + ts[6]) / 2
    x_des, u_ff = interpolator.interpolate(current_time + t_mid)
    x_expected = (xs[5] + xs[6]) / 2
    error = np.linalg.norm(x_des - x_expected)
    print(f"  中间点 (t={t_mid:.3f}s): 误差={error:.2e} {'✓' if error < 1e-10 else '✗'}")

    print(f"\n" + "="*60)
    print("✓ 插值器测试完成!")
    print("="*60)
