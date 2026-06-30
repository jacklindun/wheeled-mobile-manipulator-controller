#!/usr/bin/env python3
"""
分析已有的日志数据
"""

import numpy as np
from pathlib import Path

log_path = Path(__file__).parent.parent / "logs" / "latest.npz"

if not log_path.exists():
    print(f"日志文件不存在: {log_path}")
    exit(1)

print("="*70)
print("分析已有日志数据")
print("="*70)

data = np.load(log_path, allow_pickle=True)

print(f"\n日志文件: {log_path}")
print(f"\n可用字段:")
for key in data.keys():
    try:
        val = data[key]
        print(f"  {key}: shape={val.shape if hasattr(val, 'shape') else 'scalar'}")
    except:
        print(f"  {key}: (无法读取)")

# 提取关键数据
times = data['time']
ee_pos = data['ee_pos']
ee_ref = data['ee_ref']
solve_times = data['solve_time']

# 检查是否有mpc_success字段
if 'mpc_success' in data:
    mpc_success = data['mpc_success']
else:
    # 如果没有，假设全部成功
    mpc_success = np.ones(len(times), dtype=bool)

# 计算误差
ee_errors = np.linalg.norm(ee_pos - ee_ref, axis=1)

print(f"\n测试统计:")
print(f"  总时长: {times[-1]:.1f}s")
print(f"  总步数: {len(times)}")

print(f"\n跟踪误差 (EE):")
ee_rms = np.sqrt(np.mean(ee_errors**2)) * 100
ee_max = np.max(ee_errors) * 100
ee_mean = np.mean(ee_errors) * 100
print(f"  RMS误差:  {ee_rms:.2f} cm")
print(f"  最大误差: {ee_max:.2f} cm")
print(f"  平均误差: {ee_mean:.2f} cm")

print(f"\nMPC性能:")
convergence_rate = np.mean(mpc_success) * 100
avg_solve = np.mean(solve_times) * 1000
max_solve = np.max(solve_times) * 1000
print(f"  收敛率:       {convergence_rate:.1f}%")
print(f"  平均求解时间: {avg_solve:.1f} ms")
print(f"  最大求解时间: {max_solve:.1f} ms")

print(f"\n" + "="*70)
