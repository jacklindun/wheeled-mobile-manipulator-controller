#!/usr/bin/env python3
"""
Phase 6-v2 双臂版本 - 优化版

关键改进：
1. 修正 nominal posture
2. 优化圆形轨迹参数
3. 增大 IK 迭代次数
4. 提高 PD 增益
"""

import sys
from pathlib import Path

_project_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_project_root))

_aligator_root = _project_root.parents[1]
sys.path.insert(0, str(_aligator_root / "build" / "bindings" / "python"))

import numpy as np
import time
from scipy.optimize import minimize

print("="*80)
print("Phase 6-v2 双臂版本 - 优化版")
print("="*80)

# Import modules
print("\n正在加载模块...")
import aligator
print("  ✓ ALIGATOR")

import mujoco
import mujoco.viewer
print("  ✓ MuJoCo")

from wheeled_ur5e_aligator_mpc.dual_arm_pinocchio_model import DualArmPinocchioModel
from wheeled_ur5e_aligator_mpc.trajectory_interpolator import TrajectoryInterpolator
from wheeled_ur5e_aligator_mpc.feedforward_pd_controller import (
    FeedforwardPDController, FeedforwardPDGains
)
print("  ✓ 项目模块\n")


class DualArmRobotModel:
    """双臂模型接口"""
    def __init__(self):
        self.pin_model = DualArmPinocchioModel()
        self.nq = 16
        self.nu = 16

        # 使用优化的 nominal 配置（两臂都指向前方）
        self.q_nominal = np.array([
            0.0, 0.0, 0.2, 0.0,  # base
            # Left arm (指向前方左侧)
            -2.5434, -0.6884,  1.6850,
             0.4209, -1.3484,  0.0000,
            # Right arm (指向前方右侧)
             1.4529, -0.7472,  2.3605,
             0.3727, -1.9646,  0.0000,
        ])

        # 验证配置
        ee_left = self.pin_model.fk_left_ee(self.q_nominal)
        ee_right = self.pin_model.fk_right_ee(self.q_nominal)

        print(f"Nominal 配置的 EE 位置:")
        print(f"  Left:  [{ee_left[0]:.3f}, {ee_left[1]:.3f}, {ee_left[2]:.3f}]")
        print(f"  Right: [{ee_right[0]:.3f}, {ee_right[1]:.3f}, {ee_right[2]:.3f}]")

        self.u_min = np.ones(16) * -2.0
        self.u_max = np.ones(16) * 2.0
        self.q_min = np.ones(16) * -6.28
        self.q_max = np.ones(16) * 6.28

    def fk_left(self, q):
        return self.pin_model.fk_left_ee(q)

    def fk_right(self, q):
        return self.pin_model.fk_right_ee(q)

    def dynamics_numpy(self, q, u, dt):
        return q + dt * u

    def linearize_dynamics(self, q, u, dt):
        A = np.eye(self.nq)
        B = np.eye(self.nq) * dt
        return A, B


class DualArmMuJoCoEnv:
    """双臂 MuJoCo 环境"""

    def __init__(self, xml_path, render=False, sim_dt=0.002, control_dt=0.002):
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.sim_dt = sim_dt
        self.control_dt = control_dt

        if render:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        else:
            self.viewer = None

    def reset(self, q0):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:16] = q0
        mujoco.mj_forward(self.model, self.data)

    def step(self, q_target):
        self.data.ctrl[:16] = q_target
        mujoco.mj_step(self.model, self.data)

        if self.viewer is not None:
            self.viewer.sync()

    def get_q(self):
        return self.data.qpos[:16].copy()

    def get_ee_left(self):
        left_ee_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "left_ee_site")
        return self.data.site_xpos[left_ee_id].copy()

    def get_ee_right(self):
        right_ee_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "right_ee_site")
        return self.data.site_xpos[right_ee_id].copy()

    def set_target_markers(self, left_target, right_target):
        try:
            left_mocap_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "left_target")
            if left_mocap_id >= 0:
                self.data.mocap_pos[left_mocap_id] = left_target
        except:
            pass

        try:
            right_mocap_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "right_target")
            if right_mocap_id >= 0:
                self.data.mocap_pos[right_mocap_id] = right_target
        except:
            pass

    def close(self):
        if self.viewer is not None:
            self.viewer.close()


def generate_dual_arm_circle_reference(t, ee_left_init, ee_right_init, duration=10.0, radius=0.05):
    """
    生成双臂小圆形轨迹

    使用初始 EE 位置作为圆心
    半径缩小到 5cm
    """
    theta = 2 * np.pi * (t / duration)

    # 左臂：在 YZ 平面逆时针
    left_target = ee_left_init + radius * np.array([0, np.cos(theta), np.sin(theta)])

    # 右臂：在 YZ 平面顺时针
    right_target = ee_right_init + radius * np.array([0, np.cos(-theta), np.sin(-theta)])

    return left_target, right_target


def solve_dual_arm_ik(robot, left_target, right_target, q_init, max_iters=200):
    """
    双臂 IK - 增加迭代次数
    """
    def cost_function(q):
        ee_left = robot.fk_left(q)
        ee_right = robot.fk_right(q)

        error_left = ee_left - left_target
        error_right = ee_right - right_target

        # 添加配置空间正则化
        q_diff = q - q_init
        reg = 0.01 * np.dot(q_diff, q_diff)

        return np.dot(error_left, error_left) + np.dot(error_right, error_right) + reg

    bounds = [(robot.q_min[i], robot.q_max[i]) for i in range(robot.nq)]

    result = minimize(
        cost_function,
        q_init,
        method='L-BFGS-B',
        bounds=bounds,
        options={'maxiter': max_iters, 'ftol': 1e-8}  # 更严格的收敛
    )

    return result.x, np.sqrt(result.fun)


def run_dual_arm_phase6_v2(duration=10.0, render=False):
    """双臂 Phase 6-v2 优化版"""

    print(f"测试配置: 双臂小圆形轨迹 (5cm), {duration}s")
    print("="*80)

    # ========================================
    # 1. 初始化
    # ========================================
    print("\n1. 初始化组件...")

    robot = DualArmRobotModel()
    print(f"  ✓ 双臂Robot: {robot.nq} DOF")

    xml_path = _project_root / "assets" / "wheeled_dual_ur5e_v2.xml"
    mpc_dt = 0.05
    control_dt = 0.002

    env = DualArmMuJoCoEnv(
        xml_path=str(xml_path),
        render=render,
        sim_dt=control_dt,
        control_dt=control_dt
    )
    print(f"  ✓ MuJoCo 双臂环境")

    interpolator = TrajectoryInterpolator(mpc_dt=mpc_dt, control_dt=control_dt)
    print(f"  ✓ 插值器: {interpolator.ratio}:1")

    # 提高 PD 增益
    pd_gains = FeedforwardPDGains(
        Kp_base_xy=100.0, Kd_base_xy=20.0,
        Kp_base_z=500.0, Kd_base_z=100.0,
        Kp_arm=1000.0, Kd_arm=100.0  # 提高到 1000
    )
    pd_controller = FeedforwardPDController(pd_gains)
    print(f"  ✓ 前馈PD: Kp={pd_gains.Kp_arm[0]:.0f}")

    # ========================================
    # 2. 初始化状态
    # ========================================
    print("\n2. 初始化状态...")
    env.reset(q0=robot.q_nominal)

    # 获取初始 EE 位置
    ee_left_init = env.get_ee_left()
    ee_right_init = env.get_ee_right()
    print(f"  初始 Left EE:  {ee_left_init}")
    print(f"  初始 Right EE: {ee_right_init}")
    print(f"  ✓ 环境已重置")

    # ========================================
    # 3. 数据记录
    # ========================================
    log = {
        't': [],
        'ee_left_error': [],
        'ee_right_error': [],
        'ik_error': [],
        'ik_time': [],
    }

    # ========================================
    # 4. 控制循环
    # ========================================
    print("\n3. 开始闭环控制...")
    print(f"  {'时间':>8s} | {'左臂误差':>10s} | {'右臂误差':>10s} | {'IK误差':>10s}")
    print(f"  {'-'*8}-+-{'-'*10}-+-{'-'*10}-+-{'-'*10}")

    n_steps = int(duration / control_dt)
    last_ik_time = -np.inf
    q_ik_current = robot.q_nominal.copy()

    for step in range(n_steps):
        t = step * control_dt

        q_current = env.get_q()

        # IK 更新
        if t - last_ik_time >= mpc_dt - 1e-9:
            left_target, right_target = generate_dual_arm_circle_reference(
                t, ee_left_init, ee_right_init, duration
            )

            t_ik_start = time.perf_counter()
            q_ik_current, ik_err = solve_dual_arm_ik(
                robot, left_target, right_target, q_ik_current, max_iters=200
            )
            t_ik = time.perf_counter() - t_ik_start

            log['ik_error'].append(ik_err)
            log['ik_time'].append(t_ik)

            last_ik_time = t

        # PD 控制
        q_des = q_ik_current
        v_des = np.zeros(robot.nu)
        v_current = np.zeros(robot.nu)

        u_control, _ = pd_controller.compute_control(
            q_current, v_current,
            q_des, v_des,
            u_feedforward=None
        )

        # 应用控制
        q_target = q_current + control_dt * u_control

        left_ref, right_ref = generate_dual_arm_circle_reference(
            t, ee_left_init, ee_right_init, duration
        )
        env.set_target_markers(left_ref, right_ref)

        env.step(q_target)

        # 测量误差
        ee_left = env.get_ee_left()
        ee_right = env.get_ee_right()

        error_left = np.linalg.norm(ee_left - left_ref)
        error_right = np.linalg.norm(ee_right - right_ref)

        log['t'].append(t)
        log['ee_left_error'].append(error_left)
        log['ee_right_error'].append(error_right)

        # 打印进度
        if step % int(1.0 / control_dt) == 0 and len(log['ik_error']) > 0:
            ik_err = log['ik_error'][-1] * 100
            print(f"  {t:>7.1f}s | {error_left*100:>8.2f} cm | {error_right*100:>8.2f} cm | {ik_err:>8.3f} cm")

    env.close()
    print(f"\n  ✓ 仿真完成")

    # ========================================
    # 5. 结果分析
    # ========================================
    print("\n" + "="*80)
    print("双臂 Phase 6-v2 优化版结果")
    print("="*80)

    ee_left_errors = np.array(log['ee_left_error'])
    ee_right_errors = np.array(log['ee_right_error'])

    left_rms = np.sqrt(np.mean(ee_left_errors**2)) * 100
    left_max = np.max(ee_left_errors) * 100

    right_rms = np.sqrt(np.mean(ee_right_errors**2)) * 100
    right_max = np.max(ee_right_errors) * 100

    avg_rms = (left_rms + right_rms) / 2

    print(f"\n✅ 跟踪性能:")
    print(f"   左臂 RMS:  {left_rms:.2f} cm (最大: {left_max:.2f} cm)")
    print(f"   右臂 RMS:  {right_rms:.2f} cm (最大: {right_max:.2f} cm)")
    print(f"   平均 RMS:  {avg_rms:.2f} cm")
    print(f"   目标:      < 5 cm")
    print(f"   状态:      {'✅ 优秀' if avg_rms <= 5.0 else '✓ 良好' if avg_rms <= 10.0 else '⚠️ 需改进'}")

    if len(log['ik_error']) > 0:
        avg_ik_error = np.mean(log['ik_error']) * 100
        avg_ik_time = np.mean(log['ik_time']) * 1000

        print(f"\n✅ IK 性能:")
        print(f"   平均IK残差: {avg_ik_error:.3f} cm")
        print(f"   平均IK时间: {avg_ik_time:.1f} ms")

    control_freq = len(log['t']) / duration
    print(f"\n✅ 控制质量:")
    print(f"   控制频率: {control_freq:.0f} Hz")

    print("\n" + "="*80)
    if avg_rms <= 5.0:
        print("🎉 ✅ 双臂 Phase 6-v2 验证通过！")
    else:
        print("✓ 双臂 Phase 6-v2 基本成功，可继续优化")
    print("="*80)

    return {
        'left_rms_cm': left_rms,
        'right_rms_cm': right_rms,
        'avg_rms_cm': avg_rms,
        'control_frequency': control_freq,
    }


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='双臂 Phase 6-v2 优化版')
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--render', action='store_true')

    args = parser.parse_args()

    try:
        result = run_dual_arm_phase6_v2(
            duration=args.duration,
            render=args.render
        )

        print(f"\n📊 最终结果:")
        print(f"   左臂RMS:  {result['left_rms_cm']:.2f} cm")
        print(f"   右臂RMS:  {result['right_rms_cm']:.2f} cm")
        print(f"   平均RMS:  {result['avg_rms_cm']:.2f} cm")
        print(f"   状态:     {'🎉 优秀!' if result['avg_rms_cm'] <= 5.0 else '✓ 良好' if result['avg_rms_cm'] <= 10.0 else '需优化'}")

        sys.exit(0)

    except KeyboardInterrupt:
        print(f"\n\n⚠️ 用户中断")
        sys.exit(2)

    except Exception as e:
        print(f"\n✗ 验证失败: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
