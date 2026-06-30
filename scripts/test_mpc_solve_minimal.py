#!/usr/bin/env python3
"""
最小化测试MPC求解，定位崩溃点
"""

import sys
from pathlib import Path

_project_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_project_root))

_aligator_root = _project_root.parents[1]
sys.path[:0] = [str(_aligator_root / "build" / "bindings" / "python")]

import numpy as np
import traceback

print("="*80)
print("最小化MPC求解测试")
print("="*80)

try:
    print("\n[1] 导入模块...")
    from wheeled_ur5e_aligator_mpc.robot_model import WheeledUR5eModel
    from wheeled_ur5e_aligator_mpc.aligator_mpc_controller import AligatorWholeBodyMPC
    from wheeled_ur5e_aligator_mpc.reference import ReferenceGenerator
    print("✓ 模块导入成功")

    print("\n[2] 创建组件...")
    robot = WheeledUR5eModel()
    print(f"  ✓ Robot: {robot.nq}-DOF")

    mpc = AligatorWholeBodyMPC(robot, horizon=5, dt=0.05, max_iters=3)
    print(f"  ✓ MPC: horizon=5, dt=0.05")

    ee_start = robot.fk_numpy(robot.q_nominal)
    ref_gen = ReferenceGenerator(scenario='ee_circle', ee_start=ee_start)
    print(f"  ✓ ReferenceGenerator")

    print("\n[3] 准备状态...")
    q_current = robot.q_nominal.copy()
    print(f"  q_current: {q_current[:4]}...")

    print("\n[4] 生成参考轨迹...")
    ref_traj = ref_gen.get_reference(t=0.0, horizon=5, dt=0.05)
    print(f"  ✓ ref_traj keys: {list(ref_traj.keys())}")
    print(f"    ee_pos shape: {ref_traj['ee_pos'].shape}")
    print(f"    ee_rot shape: {ref_traj['ee_rot'].shape}")

    print("\n[5] 调用MPC.solve()...")
    print("  正在求解...")
    u0, q_pred, info = mpc.solve(q_current=q_current, ref_traj=ref_traj, u_prev=None)

    print(f"  ✓ 求解成功!")
    print(f"    收敛: {info['success']}")
    print(f"    状态: {info['status']}")
    print(f"    求解时间: {info['solve_time']*1000:.1f} ms")
    print(f"    u0: {u0[:3]}...")
    print(f"    q_pred shape: {q_pred.shape}")

    print("\n" + "="*80)
    print("✅ 测试通过！MPC求解正常工作")
    print("="*80)

except Exception as e:
    print(f"\n" + "="*80)
    print(f"❌ 错误发生")
    print("="*80)
    print(f"\n错误类型: {type(e).__name__}")
    print(f"错误信息: {e}")
    print("\n完整堆栈:")
    traceback.print_exc()
    sys.exit(1)
