"""
Phase 6 Step 5: MuJoCo闭环demo

使用Phase 1-3运动学MPC + Phase 6控制器 (插值+前馈PD)
测试完整闭环控制
"""

import sys
from pathlib import Path
import numpy as np
import time
import mujoco
import mujoco.viewer

# 添加aligator路径
_repo_root = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(_repo_root / "build" / "bindings" / "python")]

from wheeled_ur5e_aligator_mpc.aligator_problem import KinematicWheeledUR5eProblemBuilder
from wheeled_ur5e_aligator_mpc.phase6_controller import Phase6Controller, FeedforwardPDGains
from wheeled_ur5e_aligator_mpc.reference import generate_reference_trajectory
from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel


def run_phase6_demo(scenario='ee_circle', duration=10.0, render=True):
    """
    运行Phase 6完整demo

    Parameters
    ----------
    scenario : str
        场景: 'ee_circle', 'ee_line', 'stationary'
    duration : float
        运行时长(秒)
    render : bool
        是否可视化
    """
    print("="*60)
    print(f"Phase 6 MuJoCo闭环Demo")
    print(f"场景: {scenario}, 时长: {duration}s")
    print("="*60)

    # 1. 加载MuJoCo模型
    mjcf_path = Path(__file__).parent.parent / "assets" / "wheeled_ur5e.xml"
    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    data = mujoco.MjData(model)

    # 设置初始状态
    mujoco.mj_resetData(model, data)
    data.qpos[2] = 0.2  # base_z

    # 设置nominal姿态
    data.qpos[6] = np.pi  # shoulder_pan
    data.qpos[7] = np.pi / 3  # shoulder_lift
    data.qpos[8] = -np.pi / 2  # elbow

    mujoco.mj_forward(model, data)

    print(f"\n✓ MuJoCo模型加载")
    print(f"  nq = {model.nq}, nu = {model.nu}")

    # 2. 创建Phase 1-3运动学MPC
    robot = WheeledUR5eModel()
    builder = KinematicWheeledUR5eProblemBuilder(robot, horizon=15, dt=0.05)
    mpc = builder

    print(f"\n✓ MPC控制器创建")
    print(f"  Horizon: {mpc.horizon}")
    print(f"  MPC dt: {mpc.dt}s")

    # 3. 创建Phase 6控制器
    pd_gains = FeedforwardPDGains(
        Kp_base_xy=50.0, Kd_base_xy=10.0,
        Kp_base_z=500.0, Kd_base_z=100.0,
        Kp_arm=500.0, Kd_arm=50.0
    )

    phase6_controller = Phase6Controller(
        mpc_controller=mpc,
        mpc_dt=0.05,
        control_dt=model.opt.timestep,  # 使用MuJoCo的timestep
        pd_gains=pd_gains
    )

    print(f"\n✓ Phase 6控制器创建")
    print(f"  MPC频率: {1/phase6_controller.mpc_dt:.0f} Hz")
    print(f"  控制频率: {1/phase6_controller.control_dt:.0f} Hz")
    print(f"  插值比例: {phase6_controller.interpolator.ratio}:1")

    # 4. 创建参考轨迹
    ref_traj = generate_reference_trajectory(
        scenario=scenario,
        duration=duration,
        dt=mpc.dt,
        robot=robot
    )

    print(f"\n✓ 参考轨迹创建")
    print(f"  场景: {scenario}")
    print(f"  轨迹点数: {len(ref_traj['times'])}")

    # 5. 获取关节和执行器索引
    joint_ids = []
    joint_names = robot.q_names
    for name in joint_names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        joint_ids.append(jid)

    actuator_ids = []
    actuator_names = robot.u_names
    for name in actuator_names:
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        actuator_ids.append(aid)

    # EE site
    ee_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")

    # 6. 控制循环
    print(f"\n开始控制循环...")

    t = 0.0
    sim_dt = model.opt.timestep
    n_steps = int(duration / sim_dt)

    # 记录数据
    times = []
    ee_errors = []
    ee_positions = []
    controls = []

    # Viewer
    viewer = None
    if render:
        viewer = mujoco.viewer.launch_passive(model, data)

    for step in range(n_steps):
        t = step * sim_dt

        # 获取当前状态
        q_current = np.array([data.qpos[jid] for jid in joint_ids])
        x_current = q_current  # 运动学模式

        # Phase 6控制
        u_control, info = phase6_controller.control_step(x_current, ref_traj, t)

        # 应用控制
        for i, aid in enumerate(actuator_ids):
            data.ctrl[aid] = u_control[i]

        # 仿真步进
        mujoco.mj_step(model, data)

        # 记录数据
        ee_pos = data.site_xpos[ee_site_id].copy()

        # 计算EE误差
        ref_idx = min(int(t / mpc.dt), len(ref_traj['ee_pos']) - 1)
        ee_target = ref_traj['ee_pos'][ref_idx]
        ee_error = np.linalg.norm(ee_pos - ee_target)

        times.append(t)
        ee_errors.append(ee_error)
        ee_positions.append(ee_pos)
        controls.append(u_control.copy())

        # 更新viewer
        if viewer is not None:
            viewer.sync()

        # 每秒打印一次
        if step % int(1.0 / sim_dt) == 0:
            print(f"  t={t:.1f}s: EE误差={ee_error*100:.2f}cm, u_max={np.abs(u_control).max():.3f}")

    if viewer is not None:
        viewer.close()

    # 7. 统计结果
    print(f"\n" + "="*60)
    print("Phase 6测试完成")
    print("="*60)

    ee_errors = np.array(ee_errors)

    print(f"\nEE跟踪性能:")
    print(f"  RMS误差: {np.sqrt(np.mean(ee_errors**2))*100:.2f} cm")
    print(f"  最大误差: {np.max(ee_errors)*100:.2f} cm")
    print(f"  平均误差: {np.mean(ee_errors)*100:.2f} cm")

    stats = phase6_controller.get_statistics()
    print(f"\nMPC统计:")
    print(f"  求解次数: {stats['mpc_solves']}")
    print(f"  平均求解时间: {stats['mpc_solve_time_mean']*1000:.2f} ms")
    print(f"  最大求解时间: {stats['mpc_solve_time_max']*1000:.2f} ms")

    print(f"\n控制统计:")
    print(f"  控制步数: {stats['control_steps']}")
    print(f"  控制频率: {stats['control_frequency']:.0f} Hz")

    controls = np.array(controls)
    print(f"  控制范围: [{controls.min():.3f}, {controls.max():.3f}]")
    print(f"  控制平滑度: {np.std(np.diff(controls[:,0])):.4f} (std of diff)")

    print(f"\n" + "="*60)

    return {
        'times': times,
        'ee_errors': ee_errors,
        'ee_positions': ee_positions,
        'controls': controls,
        'stats': stats,
    }


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Phase 6 MuJoCo demo')
    parser.add_argument('--scenario', type=str, default='ee_circle',
                       choices=['ee_circle', 'ee_line', 'stationary'],
                       help='测试场景')
    parser.add_argument('--duration', type=float, default=10.0,
                       help='运行时长(秒)')
    parser.add_argument('--no-render', action='store_true',
                       help='不显示可视化')

    args = parser.parse_args()

    result = run_phase6_demo(
        scenario=args.scenario,
        duration=args.duration,
        render=not args.no_render
    )
