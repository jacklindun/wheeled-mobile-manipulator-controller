# Wheeled UR5e Aligator MPC - Documentation Index

**Complete guide to the wheeled mobile manipulator whole-body MPC project**

---

## 🚀 Quick Start

### New Users: Start Here
1. **Read**: [README.md](README.md) - Project overview
2. **Run**: Phase 6-v2 optimized demo (best single-arm)
   ```bash
   cd wheeled_ur5e_aligator_mpc
   export PYTHONPATH=.:../../build/bindings/python
   pixi run -e all python scripts/demo_phase6_v2_optimized.py --render
   ```
3. **Learn**: [PHASE6_V2_QUICKSTART.md](PHASE6_V2_QUICKSTART.md)

### Choosing a Version
- **Single-arm + best accuracy**: Phase 6-v2 (1.75 cm) ⭐⭐⭐⭐⭐
- **Dual-arm + torque control**: Phase 6-v3-Step1 (2.28 cm) ⭐⭐⭐⭐
- **Research on dynamics**: Phase 6-v3-Step2 (3.56 cm) ⭐⭐⭐

---

## 📚 Documentation Structure

### Core Documents

#### 1. Project Overview
- **[README.md](README.md)** - Main project documentation
  - System description
  - Requirements and setup
  - Quick demos
  - Available phases comparison

#### 2. Phase 6-v2 (Single-Arm, Recommended)
- **[PHASE6_V2_QUICKSTART.md](PHASE6_V2_QUICKSTART.md)** - Quick start guide
  - Fast setup (< 5 min)
  - Key configurations
  - Common scenarios
  - Troubleshooting
  
- **[PHASE6_V2_PERFORMANCE_REPORT.md](PHASE6_V2_PERFORMANCE_REPORT.md)** - Complete report (13KB)
  - System architecture
  - 3-round parameter tuning
  - 5-method startup optimization
  - Performance benchmarks
  - Computational analysis
  - Technical insights

#### 3. Phase 6 Complete Overview
- **[PHASE6_COMPLETE_COMPARISON.md](PHASE6_COMPLETE_COMPARISON.md)** - All versions compared
  - v1: MPC + WBC
  - v2: Kinematic MPC + Adaptive PD ⭐
  - v3-Step1: IK + Gravity FF (dual-arm) ⭐
  - v3-Step2: Dynamics MPC (research)
  - Decision guide
  - Technical insights

#### 4. Detailed Guides
- **[docs/phase6/README.md](docs/phase6/README.md)** - Phase 6 entry point
- **[docs/phase6/V2.md](docs/phase6/V2.md)** - v2 usage guide
- **[docs/phase6/V3.md](docs/phase6/V3.md)** - v3 usage guide
- **[docs/phase6/STATUS.md](docs/phase6/STATUS.md)** - Development status

---

## 🎯 Performance Summary

| Version | System | RMS Error | Status | Use Case |
|---------|--------|-----------|--------|----------|
| Phase 1-3 | Single-arm kinematic | 1.83 cm | ✅ Baseline | Stable reference |
| **Phase 6-v2** | **Single-arm optimized** | **1.75 cm** | ✅ **Best** | **Production single-arm** |
| Phase 6-v3-Step1 | Dual-arm IK+FF | 2.28 cm | ✅ Production | Dual-arm tasks |
| Phase 6-v3-Step2 | Dual-arm dynamics | 3.56 cm | 🔬 Research | Advanced research |

---

## 📂 Key Directories

### Source Code
```
wheeled_ur5e_aligator_mpc/
├── robot_model.py              # Robot kinematics
├── aligator_mpc_controller.py  # MPC controller
├── aligator_problem.py         # MPC problem builder
├── trajectory_interpolator.py  # 40Hz → 500Hz interpolation
├── feedforward_pd_controller.py # PD with adaptive gains
├── low_level_control.py        # Position command interface
├── mujoco_env.py               # MuJoCo simulation
├── reference.py                # Reference trajectory generators
├── dual_arm_*.py               # Dual-arm components (v3)
└── phase6_*.py                 # Phase 6 specific modules
```

### Scripts
```
scripts/
├── demo_phase6_v2_optimized.py     # ⭐ Best single-arm demo
├── test_phase6_v3_step1_simple.py  # ⭐ Best dual-arm demo
├── test_phase6_v3_step2_simple.py  # Research dual-arm MPC
├── run_demo.py                     # Phase 1-3 baseline
├── tune_phase6_v2*.py              # Tuning scripts (3 rounds)
├── test_adaptive_pd_gains.py       # Adaptive gains validation
└── test_combined_strategy.py       # Final combined test
```

### Results
```
results/
└── phase6_v2_tuning/
    ├── README.md                              # Results explanation
    ├── combined_strategy_comparison.png       # Final strategy
    ├── adaptive_pd_gains_comparison.png       # Adaptive PD test
    ├── phase6_v2_transient_analysis.png       # Transient analysis
    ├── startup_comparison.png                 # Startup methods
    └── combined_strategy_results.npz          # Raw data
```

### Assets
```
assets/
├── wheeled_ur5e.xml                 # Single-arm model (v2)
├── wheeled_dual_ur5e_v2_torque.xml  # Dual-arm model (v3)
└── ur5e/                            # UR5e meshes and kinematics
```

---

## 🔬 Technical Highlights

### Phase 6-v2 Innovations
1. **40 Hz MPC**: 2× frequency for faster tracking
2. **Adaptive PD Gains**: Startup boost (2× for 0-3s)
3. **Smooth Startup**: 2s cubic ease-in trajectory
4. **500 Hz Control**: High-frequency feedback loop
5. **Optimized Weights**: 3× EE position weights

**Achievement**: 69.5% error reduction from baseline (5.73 → 1.75 cm)

### Phase 6-v3 Contributions
1. **16-DOF Dual-Arm**: Independent arm control
2. **Torque Control**: Motor actuators with gravity compensation
3. **IK Solution**: Analytical IK with 0.004 cm residual
4. **Real-time**: Sub-millisecond IK computation

**Achievement**: 2.28 cm dual-arm tracking (first dual-arm MPC demo)

---

## 📊 Key Results

### Phase 6-v2 Final Performance
```
Steady-state RMS:  1.50 cm  (target: ≤ 2.5 cm)  ✅
Startup RMS:       2.20 cm  (0-3s)              ✅
Overall RMS:       1.75 cm                       ✅
Peak error:        3.29 cm  (reduced 48%)       ✅
Convergence rate:  99.7%   (target: ≥ 95%)     ✅
Control frequency: 500 Hz  (target: ≥ 450 Hz)  ✅
```

### Phase 6-v3-Step1 Performance
```
Left arm RMS:   2.19 cm
Right arm RMS:  2.37 cm
Average RMS:    2.28 cm
IK residual:    0.004 cm  (excellent)
Torque sat:     1.1%      (rare)
Real-time:      Yes       (0.39s wall for 10s sim)
```

---

## 🛠️ Development History

### Major Milestones
1. **Phase 1-3**: Kinematic MPC baseline (1.83 cm)
2. **Phase 4**: Hybrid dynamics attempt (failed, 2.5-5 cm)
3. **Phase 6-v1**: MPC + WBC exploration (2-4 cm, not optimal)
4. **Phase 6-v2**: Optimized kinematic approach (1.75 cm) ✅
5. **Phase 6-v3**: Dual-arm extension (2.28 cm) ✅

### Lessons Learned
1. Simple kinematic + high-gain PD > complex dynamics modeling
2. Adaptive gains beat sophisticated feedforward methods
3. MPC frequency matters more than prediction complexity
4. IK + gravity feedforward is excellent for dual-arm
5. Dynamics MPC still challenging in practice

---

## 🎓 For Researchers

### Extending This Work

#### Short-term (Ready to Use)
- Apply adaptive gains to Phase 6-v3 dual-arm
- Test on different trajectories (figure-8, lemniscate)
- Add obstacle avoidance constraints

#### Medium-term (Requires Development)
- Code-generated MPC (ACADOS) for <10ms solves
- Coordinated dual-arm manipulation
- Contact-rich tasks with force sensing
- Learning-based gain adaptation

#### Long-term (Research Topics)
- Implicit integrators for dynamics MPC
- Model learning from data
- Whole-body contact optimization
- Multi-contact locomotion

### Research Papers Enabled by This Work
- Kinematic MPC with adaptive gains
- High-frequency mobile manipulation control
- IK-informed dynamics MPC
- Dual-arm torque control strategies

---

## 📖 Reading Order

### For Practitioners
1. [README.md](README.md) - Overview
2. [PHASE6_V2_QUICKSTART.md](PHASE6_V2_QUICKSTART.md) - Setup
3. Run demo and verify performance
4. [PHASE6_COMPLETE_COMPARISON.md](PHASE6_COMPLETE_COMPARISON.md) - Choose version

### For Researchers
1. [PHASE6_COMPLETE_COMPARISON.md](PHASE6_COMPLETE_COMPARISON.md) - Technical overview
2. [PHASE6_V2_PERFORMANCE_REPORT.md](PHASE6_V2_PERFORMANCE_REPORT.md) - Detailed analysis
3. [docs/phase6/V3.md](docs/phase6/V3.md) - Dual-arm architecture
4. Source code and experiments

### For Students
1. [README.md](README.md) - Understand the system
2. Run Phase 1-3 baseline first
3. [PHASE6_V2_QUICKSTART.md](PHASE6_V2_QUICKSTART.md) - Modern approach
4. [PHASE6_COMPLETE_COMPARISON.md](PHASE6_COMPLETE_COMPARISON.md) - Design decisions

---

## 🔗 External References

### ALIGATOR
- Repository: https://github.com/Simple-Robotics/aligator
- Paper: "ALIGATOR: A Simple and Efficient DDP-based Solver"
- Version used: 0.19.0

### Pinocchio
- Repository: https://github.com/stack-of-tasks/pinocchio
- Documentation: https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/
- Version used: 4.0.0

### MuJoCo
- Website: https://mujoco.org
- Documentation: https://mujoco.readthedocs.io/
- Version used: 3.x

---

## ❓ FAQ

### Q: Which version should I use?
**A**: For single-arm, use Phase 6-v2 (best accuracy). For dual-arm, use Phase 6-v3-Step1 (production-ready).

### Q: Why is v2 better than v1 despite being simpler?
**A**: High-gain PD compensates for kinematic model simplicity, and avoids dynamics modeling errors. Simplicity + high-frequency feedback > complex modeling.

### Q: Can I use v2 with torque control?
**A**: Not directly tested. v2 is designed for position actuators. For torque control, use v3-Step1.

### Q: Why doesn't v3-Step2 (dynamics MPC) work better?
**A**: Dynamics modeling errors, short horizon (h=5), and integrator mismatch. IK + feedforward is already very accurate (0.004 cm residual).

### Q: How to reduce MPC solve time?
**A**: Use code-generated MPC (ACADOS), reduce horizon, or simplify costs. Current 85ms is acceptable for 40 Hz.

### Q: Can this scale to more DOFs?
**A**: v2 is optimized for 10-DOF. v3 handles 16-DOF dual-arm. Beyond that, consider hierarchical or distributed MPC.

---

## 📝 Changelog

### 2026-06-30: Phase 6 Complete
- ✅ Phase 6-v2 optimized and documented
- ✅ Phase 6-v3 tested and verified
- ✅ Complete comparison report
- ✅ Documentation overhaul

### Earlier
- Phase 1-3 baseline established (1.83 cm)
- Phase 4 hybrid approach (failed)
- Phase 6-v1 initial exploration

---

## 🙏 Acknowledgments

- **ALIGATOR Team**: Excellent DDP solver
- **Pinocchio Team**: Fast rigid body dynamics
- **MuJoCo Team**: High-quality simulator
- **Claude (Opus 4.8)**: AI pair programming assistant

---

## 📧 Contact & Contributing

For questions or contributions:
- Check documentation first
- Review existing issues
- Follow coding conventions in source
- Add tests for new features

---

**Last Updated**: 2026-06-30  
**Project Status**: ✅ Production-ready (v2 single-arm, v3-Step1 dual-arm)  
**Maintained by**: Active development
