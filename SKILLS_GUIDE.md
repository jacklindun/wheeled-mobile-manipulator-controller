# Skills Guide for Wheeled UR5e MPC Project

This guide shows you how to use the custom skills designed for this project. These skills help you work more efficiently with the mobile manipulator MPC system.

## 📚 Available Skills

### 1. `/quick-start` - Get Started Quickly
**What it does**: Complete setup verification and first demo run.

**When to use**: 
- First time working with the project
- After environment changes
- To verify everything works

**Example workflow**:
```bash
# The skill will:
# 1. Check pixi environment
# 2. Verify imports (MuJoCo, ALIGATOR, Pinocchio)
# 3. Run smoke tests
# 4. Execute a 10-second demo
# 5. Show you next steps
```

---

### 2. `/run-tests [module]` - Run Test Suite
**What it does**: Execute pytest with proper environment setup.

**Arguments**:
- `all` - Run all 27-28 tests
- `robot_model` - Test FK and dynamics
- `aligator` - Test MPC problem construction
- `mpc` - Test MPC solver
- `mujoco` - Test MuJoCo model loading
- `hybrid` - Test Phase 4 hybrid dynamics
- `dual_arm` - Test Phase 7 dual-arm

**Examples**:
```bash
/run-tests                    # All tests
/run-tests robot_model        # Just FK/dynamics tests
/run-tests dual_arm          # Dual-arm tests only
```

**Manual equivalent**:
```bash
cd study_example/wheeled_ur5e_aligator_mpc
env -i PATH="$PATH" HOME="$HOME" USER="$USER" \
  pixi run -e all python -m pytest tests/ -v
```

---

### 3. `/run-demo [scenario] [options]` - Run Simulation
**What it does**: Execute a MuJoCo demo with MPC control and analyze results.

**Scenarios**:
- `ee_circle` - EE traces a circle (default)
- `ee_line` - EE moves in a line
- `base_and_ee` - Base moves, EE holds world position
- `base_z_test` - Test vertical lift motion
- `dual_arm` - Dual-arm independent tracking
- `hybrid` - Phase 4 kinodynamic MPC

**Options**:
- `--render` - Open MuJoCo viewer
- `--duration <sec>` - Simulation time (default: 30s)
- `--headless` - No visualization
- `--analyze` - Detailed performance analysis

**Examples**:
```bash
/run-demo ee_circle --render              # Visual circle demo
/run-demo base_z_test --duration 20       # 20-second lift test
/run-demo dual_arm --analyze              # Dual-arm with analysis
```

**Manual equivalent**:
```bash
cd study_example/wheeled_ur5e_aligator_mpc
pixi run -e all python scripts/run_demo.py --scenario ee_circle --duration 30 --render
```

---

### 4. `/check-phase <number>` - Phase Status Check
**What it does**: Comprehensive health check for a development phase.

**Phases**:
- `1-3` - Kinematic MPC (✅ production-ready)
- `4` - Hybrid kinodynamic (⚠️ integrator mismatch)
- `5` - Wheeled dynamics (📋 design phase)
- `6` - MPC+WBC architecture (📋 design phase)
- `7` - Dual-arm (✅ completed)
- `8` - Dynamics matching (📋 future)

**Examples**:
```bash
/check-phase 4     # Check Phase 4 status and known issues
/check-phase 7     # Verify Phase 7 dual-arm completion
```

**What you get**:
- Implementation status
- Test results
- Performance metrics
- Known issues
- Next steps
- Related documentation

---

### 5. `/explain-module <name>` - Understand Code
**What it does**: Detailed explanation of a module's design and implementation.

**Modules**:
- `robot_model` - Basic kinematic model
- `pinocchio_model` - Pinocchio FK/Jacobian
- `aligator_problem` - MPC problem construction
- `mpc_controller` - MPC solver wrapper
- `hybrid_dynamics` - Phase 4 kinodynamic
- `dual_arm` - Dual-arm extensions
- `wbc` - Whole-body control (Phase 6)
- `mujoco_env` - MuJoCo interface

**Examples**:
```bash
/explain-module pinocchio_model    # Learn about FK implementation
/explain-module aligator_problem   # Understand MPC formulation
/explain-module hybrid_dynamics    # Phase 4 dynamics details
```

**What you get**:
- Purpose and scope
- Key classes and functions
- Design decisions
- Usage examples
- Implementation details
- Known limitations

---

### 6. `/analyze-dynamics [aspect]` - Diagnose Dynamics Issues
**What it does**: Deep analysis of dynamics model accuracy and issues.

**Aspects**:
- `all` - Complete analysis (default)
- `fk` - FK accuracy vs MuJoCo
- `jacobian` - Jacobian numerical verification
- `integrator` - Phase 4 integration error
- `mismatch` - ALIGATOR vs MuJoCo prediction gap

**Examples**:
```bash
/analyze-dynamics                # Full dynamics check
/analyze-dynamics fk            # Just FK accuracy
/analyze-dynamics integrator    # Phase 4 mismatch diagnosis
```

**What you get**:
- Error magnitudes and locations
- Root cause analysis
- Recommended fixes
- References to Phase 8 solutions

---

### 7. `/benchmark [target] [options]` - Performance Testing
**What it does**: Systematic performance benchmarking and comparison.

**Targets**:
- `all` - All scenarios and metrics (default)
- `mpc` - MPC solve time
- `tracking` - EE tracking accuracy
- `scenarios` - Compare all demos
- `phases` - Compare Phase 1-3 vs 4 vs 7

**Options**:
- `--quick` - 10s per scenario (fast)
- `--thorough` - 60s per scenario (statistical)
- `--compare` - Compare with historical data
- `--export` - Export to CSV/JSON

**Examples**:
```bash
/benchmark                          # Full benchmark
/benchmark mpc --quick              # Quick MPC timing test
/benchmark scenarios --thorough     # Thorough scenario comparison
```

**What you get**:
- Performance tables (solve time, tracking error, success rate)
- Comparison plots
- Bottleneck identification
- Tuning recommendations

---

## 🎯 Typical Workflows

### First-Time Setup
```bash
/quick-start                    # Verify environment and run first demo
/run-tests                      # Confirm all tests pass
/check-phase 7                  # Check latest phase status
```

### Daily Development
```bash
/run-tests robot_model          # Quick test after changes
/run-demo ee_circle --render    # Visual verification
/analyze-dynamics fk            # Check FK if issues
```

### Performance Tuning
```bash
/benchmark mpc --quick          # Measure solve time
/run-demo ee_circle --duration 60 --analyze  # Long run analysis
/benchmark scenarios --compare  # Check for regressions
```

### Learning the Codebase
```bash
/explain-module pinocchio_model  # Understand FK
/explain-module aligator_problem # Learn MPC formulation
/check-phase 4                   # Understand Phase 4 issues
```

### Debugging Issues
```bash
/analyze-dynamics all           # Full dynamics check
/check-phase 4                  # Check Phase 4 known issues
/run-tests hybrid               # Test specific module
```

---

## 🔧 Manual Command Reference

If skills don't work or you prefer manual commands:

### Environment Setup
```bash
cd /home/ldq/spirita-work/mobile-manipulator/aligator/study_example/wheeled_ur5e_aligator_mpc

# Activate environment once
eval "$(pixi shell-hook -e all)"
```

### Run Tests
```bash
# With ROS env stripped (required for pytest)
env -i PATH="$PATH" HOME="$HOME" USER="$USER" \
  pixi run -e all python -m pytest tests/ -v
```

### Run Demos
```bash
# Single-arm demos
pixi run -e all python scripts/run_demo.py --scenario ee_circle --render

# Dual-arm demo
pixi run -e all python scripts/demo_dual_arm_mpc.py

# Phase 4 hybrid
pixi run -e all python scripts/run_hybrid_demo.py
```

### Analyze Logs
```bash
pixi run -e all python scripts/plot_log.py
```

### Check Configuration
```bash
pixi run -e all python scripts/check_robot_configuration_text.py
```

---

## 📖 Documentation References

- **Project Overview**: [README.md](README.md)
- **Development Roadmap**: [ROADMAP.md](ROADMAP.md)
- **Overall Planning**: [项目整体规划.md](项目整体规划.md)
- **Phase Summaries**: `PHASE_X_SUMMARY.md`
- **Phase Designs**: `PHASE_X_DESIGN.md`
- **Test Summary**: `tests/PHASE6_COMPLETION_SUMMARY.md`
- **Model Migration**: [MODEL_V2_MIGRATION.md](MODEL_V2_MIGRATION.md)

---

## 💡 Tips

1. **Skills require session restart**: After creating new skills, restart Claude to load them.

2. **Environment matters**: Always use `pixi run -e all` for the correct environment.

3. **ROS conflicts**: Tests need ROS env vars stripped (`env -i ...`).

4. **Working directory**: Skills assume you're in the project root or repo root.

5. **Check phases first**: Use `/check-phase` to understand current status before making changes.

6. **Benchmark regularly**: Use `/benchmark --quick` to catch performance regressions early.

---

## 🚀 Quick Actions

**I want to...**

- ✅ Verify setup → `/quick-start`
- ✅ Run all tests → `/run-tests`
- ✅ See a demo → `/run-demo ee_circle --render`
- ✅ Check Phase 4 issues → `/check-phase 4`
- ✅ Learn MPC formulation → `/explain-module aligator_problem`
- ✅ Debug FK problems → `/analyze-dynamics fk`
- ✅ Measure performance → `/benchmark mpc --quick`

---

**Last updated**: 2026-06-25  
**Project**: Wheeled UR5e Aligator MPC  
**Status**: Phase 7 completed, Phase 6 in design
