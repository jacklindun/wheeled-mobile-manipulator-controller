#!/usr/bin/env python3
"""
测试Phase 4 Jacobian修复是否成功
"""
import sys
from pathlib import Path

# Add parent directory to path
_repo_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_repo_root))

import numpy as np
from wheeled_ur5e_aligator_mpc.hybrid_dynamics import HybridWheeledUR5eDynamics
from wheeled_ur5e_aligator_mpc.pinocchio_model import PinocchioWheeledUR5eModel

print("="*80)
print("Testing Phase 4 Hybrid Dynamics Jacobian Accuracy")
print("="*80)

# Create dynamics
pin_robot = PinocchioWheeledUR5eModel()
hybrid_dyn = HybridWheeledUR5eDynamics(pin_robot, dt=0.05)

# Random test state
rng = np.random.default_rng(13)
x = rng.uniform(-0.5, 0.5, 16)
x[4:10] = np.array([np.pi, np.pi/3, -np.pi/2, np.pi/6, 0, 0]) + rng.uniform(-0.2, 0.2, 6)
x[10:16] = rng.uniform(-0.5, 0.5, 6)
u = rng.uniform(-1.0, 1.0, 10)

print(f"\nTest configuration:")
print(f"  State x: 16-dim")
print(f"  Control u: 10-dim")
print(f"  Integration dt: 0.05s")

# Compute analytical Jacobian
data = hybrid_dyn.createData()
hybrid_dyn.dForward(x, u, data)
Jx_analytic = data.Jx.copy()
Ju_analytic = data.Ju.copy()

print(f"\n1. Computing analytical Jacobians...")
print(f"   Jx shape: {Jx_analytic.shape}")
print(f"   Ju shape: {Ju_analytic.shape}")

# Finite difference Jx
print(f"\n2. Computing finite-difference Jx...")
eps = 1e-7
Jx_fd = np.zeros((16, 16))
for i in range(16):
    x_p = x.copy(); x_p[i] += eps
    x_m = x.copy(); x_m[i] -= eps
    hybrid_dyn.forward(x_p, u, data); xnext_p = data.xnext.copy()
    hybrid_dyn.forward(x_m, u, data); xnext_m = data.xnext.copy()
    Jx_fd[:, i] = (xnext_p - xnext_m) / (2 * eps)

# Finite difference Ju
print(f"3. Computing finite-difference Ju...")
Ju_fd = np.zeros((16, 10))
for i in range(10):
    u_p = u.copy(); u_p[i] += eps
    u_m = u.copy(); u_m[i] -= eps
    hybrid_dyn.forward(x, u_p, data); xnext_p = data.xnext.copy()
    hybrid_dyn.forward(x, u_m, data); xnext_m = data.xnext.copy()
    Ju_fd[:, i] = (xnext_p - xnext_m) / (2 * eps)

# Compute errors
err_Jx = np.abs(Jx_analytic - Jx_fd)
err_Ju = np.abs(Ju_analytic - Ju_fd)

max_err_Jx = np.max(err_Jx)
max_err_Ju = np.max(err_Ju)

print("\n" + "="*80)
print("RESULTS")
print("="*80)

print(f"\nJx (state Jacobian):")
print(f"  Max error: {max_err_Jx:.6e}")
print(f"  Mean error: {np.mean(err_Jx):.6e}")
print(f"  Target: < 1.0e-04")
print(f"  Status: {'✅ PASS' if max_err_Jx < 1e-4 else '❌ FAIL'}")

print(f"\nJu (control Jacobian):")
print(f"  Max error: {max_err_Ju:.6e}")
print(f"  Mean error: {np.mean(err_Ju):.6e}")
print(f"  Target: < 1.0e-04")
print(f"  Status: {'✅ PASS' if max_err_Ju < 1e-4 else '❌ FAIL'}")

# Block-wise analysis
print(f"\nBlock-wise Jx errors:")
print(f"  Base kinematics [0:4, 0:4]: {np.max(err_Jx[0:4, 0:4]):.6e}")
print(f"  q_arm wrt q_arm [4:10, 4:10]: {np.max(err_Jx[4:10, 4:10]):.6e}")
print(f"  q_arm wrt v_arm [4:10, 10:16]: {np.max(err_Jx[4:10, 10:16]):.6e}")
print(f"  v_arm wrt q_arm [10:16, 4:10]: {np.max(err_Jx[10:16, 4:10]):.6e}")
print(f"  v_arm wrt v_arm [10:16, 10:16]: {np.max(err_Jx[10:16, 10:16]):.6e}")

if max_err_Jx >= 1e-4:
    print(f"\n⚠️  Worst Jx errors:")
    top5 = np.argsort(err_Jx.ravel())[-5:][::-1]
    for idx in top5:
        r, c = np.unravel_index(idx, err_Jx.shape)
        print(f"    [{r:2d},{c:2d}]: analytic={Jx_analytic[r,c]:.6e}, "
              f"fd={Jx_fd[r,c]:.6e}, err={err_Jx[r,c]:.6e}")

# Overall result
print("\n" + "="*80)
overall_pass = (max_err_Jx < 1e-4) and (max_err_Ju < 1e-4)
print(f"OVERALL: {'✅ ALL TESTS PASSED' if overall_pass else '❌ SOME TESTS FAILED'}")
print("="*80)

sys.exit(0 if overall_pass else 1)
