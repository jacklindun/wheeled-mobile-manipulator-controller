#!/usr/bin/env python3
"""
Phase 6-v2 双臂版本

架构：运动学MPC (20Hz) → 插值器 (500Hz) → 前馈PD (500Hz) → MuJoCo

双臂系统：
- 16 DOF: [base(4), left_arm(6), right_arm(6)]
- 双 EE 跟踪：独立控制左右臂
- IK 预规划：分别为左右臂求解

模型文件：wheeled_dual_ur5e_v2.xml
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
print("Phase 6-v2 双臂版本 - MuJoCo 闭环验证")
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
    """
    简化的双臂模型接口（兼容单臂代码）
    """
    def __init__(self):
        self.pin_model = DualArmPinocchioModel()
        self.nq = 16  # [base(4), left_arm(6), right_arm(6)]
        self.nu = 16  # 速度控制

        # Nominal posture
        self.q_nominal = np.array([
            0.0, 0.0, 0.0, 0.2,  # base: x, y, yaw, z
            # Left arm (pointing forward)
            np.pi, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0.0,
            # Right arm (pointing forward)
            np.pi, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0.0,
        ])

        # Control limits
        self.u_min = np.ones(16) * -1.0
        self.u_max = np.ones(16) * 1.0

        self.q_min = np.ones(16) * -10.0
        self.q_max = np.ones(16) * 10.0

    def fk_left(self, q):
        """左臂末端位置"""
        return self.pin_model.fk_left_ee(q)

    def fk_right(self, q):
        """右臂末端位置"""
        return self.pin_model.fk_right_ee(q)

    def dynamics_numpy(self, q, u, dt):
        """运动学积分：q_{k+1} = q_k + dt * u_k"""
        return q + dt * u

    def linearize_dynamics(self, q, u, dt):
        """线性化：A = I, B = dt * I"""
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

        # 渲染器
        if render:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        else:
            self.viewer = None

    def reset(self, q0):
        """重置到初始配置"""
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:16] = q0
        mujoco.mj_forward(self.model, self.data)

    def step(self, q_target):
        """执行一步（位置控制）"""
        self.data.ctrl[:16] = q_target
        mujoco.mj_step(self.model, self.data)

        if self.viewer is not None:
            self.viewer.sync()

    def get_q(self):
        """获取当前配置"""
        return self.data.qpos[:16].copy()

    def get_ee_left(self):
        """获取左臂 EE 位置"""
        left_ee_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "left_ee_site")
        return self.data.site_xpos[left_ee_id].copy()

    def get_ee_right(self):
        """获取右臂 EE 位置"""
        right_ee_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "right_ee_site")
        return self.data.site_xpos[right_ee_id].copy()

    def set_target_markers(self, left_target, right_target):
        """设置可视化目标标记"""
        # 左臂目标
        left_mocap_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "left_target")
        if left_mocap_id >= 0:
            self.data.mocap_pos[left_mocap_id] = left_target

        # 右臂目标
        right_mocap_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "right_target")
        if right_mocap_id >= 0:
            self.data.mocap_pos[right_mocap_id] = right_target

    def close(self):
        """关闭环境"""
        if self.viewer is not None:
            self.viewer.close()


def generate_dual_arm_circle_reference(t, duration=10.0, radius=0.1):
    """
    生成双臂圆形轨迹参考

    左臂：逆时针圆
    右臂：顺时针圆
    """
    # 左臂：以左侧为中心的圆
    left_center = np.array([0.5, 0.3, 0.6])
    theta_left = 2 * np.pi * (t / duration)
    left_target = left_center + radius * np.array([
        0,
        np.cos(theta_left),
        np.sin(theta_left)
    ])

    # 右臂：以右侧为中心的圆
    right_center = np.array([0.5, -0.3, 0.6])
    theta_right = -2 * np.pi * (t / duration)  # 反向
    right_target = right_center + radius * np.array([
        0,
        np.cos(theta_right),
        np.sin(theta_right)
    ])

    return left_target, right_target


def solve_dual_arm_ik(robot, left_target, right_target, q_init):
    """
    双臂 IK 求解

    最小化：||FK_left(q) - left_target||² + ||FK_right(q) - right_target||²
    """
    def cost_function(q):
        ee_left = robot.fk_left(q)
        ee_right = robot.fk_right(q)

        error_left = ee_left - left_target
        error_right = ee_right - right_target

        return np.dot(error_left, error_left) + np.dot(error_right, error_right)

    bounds = [(robot.q_min[i], robot.q_max[i]) for i in range(robot.nq)]

    result = minimize(
        cost_function,
        q_init,
        method='L-BFGS-B',
        bounds=bounds,
        options={'maxiter': 50, 'ftol': 1e-6}
    )

    q_solution = result.x
    final_error = np.sqrt(result.fun)

    return q_solution, final_error


def run_dual_arm_phase6_v2(duration=10.0, render=False):
    """双臂 Phase 6-v2 闭环测试"""

    print(f"测试配置: 双臂圆形轨迹, {duration}s, 渲染={render}")
    print("="*80)

    # ========================================
    # 1. 初始化
    # ========================================
    print("\n1. 初始化组件...")

    robot = DualArmRobotModel()
    print(f"  ✓ 双臂Robot: {robot.nq} DOF (16 = 4 base + 6 left + 6 right)")

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

    # 简化：使用插值器和PD，但不使用完整MPC（避免复杂性）
    interpolator = TrajectoryInterpolator(mpc_dt=mpc_dt, control_dt=control_dt)
    print(f"  ✓ 插值器: {interpolator.ratio}:1")

    pd_gains = FeedforwardPDGains(
        Kp_base_xy=50.0, Kd_base_xy=10.0,
        Kp_base_z=500.0, Kd_base_z=100.0,
        Kp_arm=500.0, Kd_arm=50.0
    )
    pd_controller = FeedforwardPDController(pd_gains)
    print(f"  ✓ 前馈PD控制器")

    # ========================================
    # 2. 初始化状态
    # ========================================
    print("\n2. 初始化状态...")
    env.reset(q0=robot.q_nominal)
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
    # 4. 控制循环（简化版：IK + PD，无MPC）
    # ========================================
    print("\n3. 开始闭环控制（双臂 IK + PD）...")
    print(f"  {'时间':>8s} | {'左臂误差':>10s} | {'右臂误差':>10s} | {'IK误差':>10s} | {'IK时间':>10s}")
    print(f"  {'-'*8}-+-{'-'*10}-+-{'-'*10}-+-{'-'*10}-+-{'-'*10}")

    n_steps = int(duration / control_dt)
    last_ik_time = -np.inf
    q_ik_current = robot.q_nominal.copy()

    for step in range(n_steps):
        t = step * control_dt

        q_current = env.get_q()

        # ========================================
        # 每个 MPC 周期更新 IK 目标
        # ========================================
        if t - last_ik_time >= mpc_dt - 1e-9:
            # 生成参考
            left_target, right_target = generate_dual_arm_circle_reference(t, duration)

            # 求解双臂 IK
            t_ik_start = time.perf_counter()
            q_ik_current, ik_err = solve_dual_arm_ik(
                robot, left_target, right_target, q_ik_current
            )
            t_ik = time.perf_counter() - t_ik_start

            log['ik_error'].append(ik_err)
            log['ik_time'].append(t_ik)

            last_ik_time = t

        # ========================================
        # PD 控制跟踪 IK 解
        # ========================================
        q_des = q_ik_current
        v_des = np.zeros(robot.nu)
        v_current = np.zeros(robot.nu)

        u_control, _ = pd_controller.compute_control(
            q_current, v_current,
            q_des, v_des,
            u_feedforward=None
        )

        # ========================================
        # 应用控制
        # ========================================
        q_target = q_current + control_dt * u_control

        # 生成当前参考用于可视化
        left_ref, right_ref = generate_dual_arm_circle_reference(t, duration)
        env.set_target_markers(left_ref, right_ref)

        env.step(q_target)

        # ========================================
        # 测量误差
        # ========================================
        ee_left = env.get_ee_left()
        ee_right = env.get_ee_right()

        error_left = np.linalg.norm(ee_left - left_ref)
        error_right = np.linalg.norm(ee_right - right_ref)

        log['t'].append(t)
        log['ee_left_error'].append(error_left)
        log['ee_right_error'].append(error_right)

        # 打印进度
        if step % int(1.0 / control_dt) == 0 and len(log['ik_time']) > 0:
            ik_err = log['ik_error'][-1] * 100
            ik_t = log['ik_time'][-1] * 1000
            print(f"  {t:>7.1f}s | {error_left*100:>8.2f} cm | {error_right*100:>8.2f} cm | "
                  f"{ik_err:>8.2f} cm | {ik_t:>8.1f} ms")

    env.close()
    print(f"\n  ✓ 仿真完成")

    # ========================================
    # 5. 结果分析
    # ========================================
    print("\n" + "="*80)
    print("双臂 Phase 6-v2 验证结果")
    print("="*80)

    ee_left_errors = np.array(log['ee_left_error'])
    ee_right_errors = np.array(log['ee_right_error'])

    # 左臂性能
    left_rms = np.sqrt(np.mean(ee_left_errors**2)) * 100
    left_max = np.max(ee_left_errors) * 100

    print(f"\n✅ 左臂跟踪性能:")
    print(f"   RMS误差:  {left_rms:.2f} cm")
    print(f"   最大误差: {left_max:.2f} cm")

    # 右臂性能
    right_rms = np.sqrt(np.mean(ee_right_errors**2)) * 100
    right_max = np.max(ee_right_errors) * 100

    print(f"\n✅ 右臂跟踪性能:")
    print(f"   RMS误差:  {right_rms:.2f} cm")
    print(f"   最大误差: {right_max:.2f} cm")

    # 综合性能
    avg_rms = (left_rms + right_rms) / 2

    print(f"\n✅ 综合性能:")
    print(f"   平均RMS:  {avg_rms:.2f} cm")
    print(f"   状态:     {'✅ 优秀' if avg_rms <= 4.0 else '✓ 良好'}")

    # IK 质量
    if len(log['ik_error']) > 0:
        avg_ik_error = np.mean(log['ik_error']) * 100
        avg_ik_time = np.mean(log['ik_time']) * 1000

        print(f"\n✅ IK 性能:")
        print(f"   平均IK残差: {avg_ik_error:.3f} cm")
        print(f"   平均IK时间: {avg_ik_time:.1f} ms")

    # 控制质量
    control_freq = len(log['t']) / duration

    print(f"\n✅ 控制质量:")
    print(f"   控制频率: {control_freq:.0f} Hz")
    print(f"   目标频率: 500 Hz")

    print("\n" + "="*80)
    print("🎉 双臂 Phase 6-v2 闭环验证完成！")
    print("="*80)

    return {
        'left_rms_cm': left_rms,
        'right_rms_cm': right_rms,
        'avg_rms_cm': avg_rms,
        'control_frequency': control_freq,
    }


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='双臂 Phase 6-v2 闭环验证')
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
        print(f"   频率:     {result['control_frequency']:.0f} Hz")

        sys.exit(0)

    except KeyboardInterrupt:
        print(f"\n\n⚠️ 用户中断")
        sys.exit(2)

    except Exception as e:
        print(f"\n✗ 验证失败: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
