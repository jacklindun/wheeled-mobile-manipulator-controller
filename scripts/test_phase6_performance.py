"""
测试插值+前馈PD方案的实际效果

对比三种方案:
1. 直接MPC输出 (Phase 1-3 baseline)
2. MPC + 插值 (无PD)
3. MPC + 插值 + 前馈PD (Phase 6完整方案)
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
import matplotlib.pyplot as plt
from wheeled_ur5e_aligator_mpc.phase6_controller import (
    MockMPCController, TrajectoryInterpolator,
    FeedforwardPDController, FeedforwardPDGains
)


def simulate_control(method='baseline', duration=2.0, noise_level=0.01):
    """
    模拟控制效果

    Parameters
    ----------
    method : str
        'baseline': 直接MPC输出 (20Hz)
        'interpolation': MPC + 插值 (500Hz)
        'phase6': MPC + 插值 + 前馈PD (500Hz)
    duration : float
        模拟时长
    noise_level : float
        状态噪声水平
    """
    # 创建MPC
    mpc = MockMPCController(horizon=10, dt=0.05, state_dim=10, control_dim=10)

    # 初始状态
    x_current = np.zeros(10)
    x_current[2] = 0.2

    # 目标
    x_target = x_current.copy()
    x_target[0] = 0.1  # 向前移动10cm

    ref_traj = {'ee_pos': np.array([[0.6, 0.0, 0.8]])}

    # 根据方法设置控制频率
    if method == 'baseline':
        dt = 0.05  # MPC频率
    else:
        dt = 0.002  # 控制频率

    n_steps = int(duration / dt)

    # 记录
    times = []
    positions = []
    controls = []
    errors = []

    # 插值器 (如果需要)
    if method in ['interpolation', 'phase6']:
        interpolator = TrajectoryInterpolator(mpc_dt=0.05, control_dt=0.002)

    # PD控制器 (如果需要)
    if method == 'phase6':
        pd_gains = FeedforwardPDGains(Kp_base_xy=50.0, Kd_base_xy=10.0,
                                     Kp_arm=500.0, Kd_arm=50.0)
        pd_controller = FeedforwardPDController(pd_gains)

    last_mpc_time = -1

    for step in range(n_steps):
        t = step * dt

        # 更新MPC
        if t - last_mpc_time >= 0.05:
            result = mpc.solve(x_current, ref_traj)
            if method in ['interpolation', 'phase6']:
                trajectory = {'xs': result['xs'], 'us': result['us'], 'ts': result['ts']}
                interpolator.update_trajectory(trajectory, t)
            last_mpc_time = t

        # 获取控制输出
        if method == 'baseline':
            # 直接使用MPC输出
            u_control = result['us'][0]

        elif method == 'interpolation':
            # 插值
            x_des, u_control = interpolator.interpolate(t)
            if u_control is None:
                u_control = np.zeros(10)

        elif method == 'phase6':
            # 插值 + 前馈PD
            x_des, u_feedforward = interpolator.interpolate(t)
            if x_des is None:
                u_control = np.zeros(10)
            else:
                q_current = x_current
                v_current = np.zeros(10)
                q_des = x_des
                v_des = np.zeros(10)
                u_control, _ = pd_controller.compute_control(
                    q_current, v_current, q_des, v_des, u_feedforward
                )

        # 应用控制 (简化动力学模型)
        x_current = x_current + u_control * dt

        # 添加噪声
        x_current += np.random.randn(10) * noise_level * dt

        # 记录
        times.append(t)
        positions.append(x_current[0])  # 只看x位置
        controls.append(u_control[0])
        errors.append(abs(x_current[0] - x_target[0]))

    return {
        'times': np.array(times),
        'positions': np.array(positions),
        'controls': np.array(controls),
        'errors': np.array(errors),
    }


def compare_methods():
    """对比三种方法"""
    print("="*70)
    print("测试插值+前馈PD方案效果")
    print("="*70)

    duration = 2.0
    noise_level = 0.01

    # 运行三种方法
    print("\n运行测试...")
    results = {}
    for method in ['baseline', 'interpolation', 'phase6']:
        print(f"  {method}...")
        results[method] = simulate_control(method, duration, noise_level)

    # 统计分析
    print("\n" + "="*70)
    print("性能对比")
    print("="*70)

    for method, result in results.items():
        controls = result['controls']
        errors = result['errors']

        # 控制平滑度
        control_diff = np.diff(controls)
        smoothness = np.std(control_diff)

        # 跟踪误差
        rms_error = np.sqrt(np.mean(errors**2))
        max_error = np.max(errors)

        # 控制频率
        freq = len(result['times']) / duration

        label = {
            'baseline': 'Baseline (直接MPC)',
            'interpolation': 'MPC + 插值',
            'phase6': 'Phase 6 (插值+前馈PD)'
        }[method]

        print(f"\n{label}:")
        print(f"  控制频率: {freq:.0f} Hz")
        print(f"  RMS误差: {rms_error*100:.2f} cm")
        print(f"  最大误差: {max_error*100:.2f} cm")
        print(f"  控制平滑度 (std): {smoothness:.4f}")
        print(f"  控制范围: [{controls.min():.3f}, {controls.max():.3f}]")

    # 计算改进百分比
    print("\n" + "="*70)
    print("改进分析 (相对于Baseline)")
    print("="*70)

    baseline_smooth = np.std(np.diff(results['baseline']['controls']))
    baseline_rms = np.sqrt(np.mean(results['baseline']['errors']**2))

    for method in ['interpolation', 'phase6']:
        smooth = np.std(np.diff(results[method]['controls']))
        rms = np.sqrt(np.mean(results[method]['errors']**2))

        smooth_improve = (baseline_smooth - smooth) / baseline_smooth * 100
        rms_improve = (baseline_rms - rms) / baseline_rms * 100

        label = {
            'interpolation': 'MPC + 插值',
            'phase6': 'Phase 6'
        }[method]

        print(f"\n{label}:")
        print(f"  平滑度改进: {smooth_improve:+.1f}%")
        print(f"  跟踪误差改进: {rms_improve:+.1f}%")

    # 可视化
    print("\n" + "="*70)
    print("生成对比图...")
    print("="*70)

    fig, axes = plt.subplots(3, 1, figsize=(12, 10))

    colors = {'baseline': 'blue', 'interpolation': 'orange', 'phase6': 'green'}
    labels = {
        'baseline': 'Baseline (20Hz MPC)',
        'interpolation': 'MPC + 插值 (500Hz)',
        'phase6': 'Phase 6 (插值+PD, 500Hz)'
    }

    # 1. 位置轨迹
    ax = axes[0]
    for method, result in results.items():
        ax.plot(result['times'], result['positions']*100,
               label=labels[method], color=colors[method], linewidth=2)
    ax.axhline(10, color='red', linestyle='--', label='目标 (10cm)')
    ax.set_ylabel('位置 (cm)')
    ax.set_title('位置跟踪对比')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 2. 控制输出
    ax = axes[1]
    for method, result in results.items():
        ax.plot(result['times'], result['controls'],
               label=labels[method], color=colors[method], linewidth=1.5)
    ax.set_ylabel('控制输出')
    ax.set_title('控制平滑度对比')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 3. 跟踪误差
    ax = axes[2]
    for method, result in results.items():
        ax.plot(result['times'], result['errors']*100,
               label=labels[method], color=colors[method], linewidth=2)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('误差 (cm)')
    ax.set_title('跟踪误差对比')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    # 保存图片
    output_path = Path(__file__).parent.parent / 'logs' / 'phase6_comparison.png'
    output_path.parent.mkdir(exist_ok=True)
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\n✓ 对比图已保存: {output_path}")

    # 显示图片
    try:
        plt.show(block=False)
        plt.pause(2)
    except:
        pass

    print("\n" + "="*70)
    print("✓ 测试完成!")
    print("="*70)


if __name__ == '__main__':
    compare_methods()
