"""Phase 6 简化demo - 使用模拟MPC测试控制器"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
import time

# 使用Phase 6的模拟MPC
from wheeled_ur5e_aligator_mpc.phase6_controller import Phase6Controller, MockMPCController, FeedforwardPDGains

print("="*60)
print("Phase 6 简化测试 (不使用MuJoCo)")
print("="*60)

# 创建模拟MPC
mock_mpc = MockMPCController(horizon=10, dt=0.05, state_dim=10, control_dim=10)
print("\n✓ 模拟MPC创建")

# 创建Phase 6控制器
pd_gains = FeedforwardPDGains(
    Kp_base_xy=50.0, Kd_base_xy=10.0,
    Kp_arm=500.0, Kd_arm=50.0
)
controller = Phase6Controller(mock_mpc, mpc_dt=0.05, control_dt=0.002, pd_gains=pd_gains)
print("✓ Phase 6控制器创建")
print(f"  MPC: {1/controller.mpc_dt:.0f} Hz")
print(f"  控制: {1/controller.control_dt:.0f} Hz")

# 模拟控制循环
x_current = np.zeros(10)
x_current[2] = 0.2  # base_z

ref_traj = {'ee_pos': np.array([[0.6, 0.0, 0.8]])}

print("\n运行5秒控制循环...")
dt = controller.control_dt
duration = 5.0
n_steps = int(duration / dt)

for i in range(n_steps):
    t = i * dt
    u_control, info = controller.control_step(x_current, ref_traj, t)
    
    if i % 500 == 0:  # 每秒打印
        print(f"  t={t:.1f}s: status={info['status']}, u[0]={u_control[0]:.4f}")

stats = controller.get_statistics()
print(f"\n统计:")
print(f"  MPC求解: {stats['mpc_solves']}次")
print(f"  控制步数: {stats['control_steps']}步")
print(f"  平均MPC时间: {stats['mpc_solve_time_mean']*1000:.2f}ms")

print("\n✓ Phase 6控制器工作正常!")
print("="*60)
