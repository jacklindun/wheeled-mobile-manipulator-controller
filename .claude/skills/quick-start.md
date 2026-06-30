# Quick Start

Get started with the project quickly - setup, verification, and first demo.

## Usage

```
/quick-start
```

## What it does

Walks through the complete setup and verification process:

1. **Environment Check**
   - Verifies pixi environment 'all' is available
   - Checks ALIGATOR build exists
   - Tests Pinocchio, MuJoCo imports

2. **Quick Build** (if needed)
   - Builds ALIGATOR from source
   - Verifies Python bindings

3. **Smoke Tests**
   - Runs minimal test suite (fast tests only)
   - Verifies FK accuracy
   - Tests ALIGATOR problem construction

4. **First Demo**
   - Runs a short 10-second demo
   - Shows basic visualization
   - Explains the output

5. **Next Steps Guide**
   - Available scenarios
   - How to run tests
   - Where to find documentation
   - Common workflows

## Instructions

cd to study_example/wheeled_ur5e_aligator_mpc

### Step 1: Environment Check
```bash
# Check pixi
pixi --version

# Check environment
pixi list -e all | grep -E "(mujoco|numpy|scipy)"

# Check ALIGATOR build
ls -la ../../build/bindings/python/aligator/__init__.py
```

### Step 2: Import Verification
```bash
pixi run -e all python -c "
import mujoco
print(f'✓ MuJoCo {mujoco.__version__}')

import sys
sys.path.insert(0, '../../build/bindings/python')
import aligator
print(f'✓ ALIGATOR {aligator.__version__}')

import pinocchio
print(f'✓ Pinocchio {pinocchio.__version__}')
"
```

### Step 3: Run Smoke Tests
```bash
# Quick test subset
env -i PATH="$PATH" HOME="$HOME" USER="$USER" \
  pixi run -e all python -m pytest \
  tests/test_robot_model.py::test_fk_nominal \
  tests/test_mujoco_load.py::test_model_loads \
  -v
```

### Step 4: First Demo
```bash
# 10-second circle demo, headless
pixi run -e all python scripts/run_demo.py \
  --scenario ee_circle --duration 10
```

Check output:
- Should complete without errors
- Prints summary with tracking error
- Generates logs/latest.npz

### Step 5: Generate Guide
Provide formatted output:

```markdown
# 🎉 Setup Complete!

## ✓ Verified Components
- MuJoCo: [version]
- ALIGATOR: [version]
- Pinocchio: [version]
- Tests passing: X/X

## 📊 First Demo Results
- Scenario: ee_circle (10s)
- EE tracking error: X.X cm
- MPC solve time: XX ms
- Success rate: XX%

## 🚀 Next Steps

### Run More Demos
\`\`\`bash
# With visualization
pixi run -e all python scripts/run_demo.py --scenario ee_circle --duration 30 --render

# Different scenarios
pixi run -e all python scripts/run_demo.py --scenario base_z_test --render

# Dual-arm
pixi run -e all python scripts/demo_dual_arm_mpc.py
\`\`\`

### Run Full Test Suite
\`\`\`bash
pixi run -e all python -m pytest tests/ -v
\`\`\`

### Explore the Code
Key modules:
- `wheeled_ur5e_aligator_mpc/robot_model.py` - FK and dynamics
- `wheeled_ur5e_aligator_mpc/aligator_problem.py` - MPC problem
- `wheeled_ur5e_aligator_mpc/aligator_mpc_controller.py` - Solver wrapper

### Read Documentation
- README.md - Overview and usage
- ROADMAP.md - Development phases
- PHASE_X_*.md - Detailed phase docs

### Use Skills
Available /commands:
- `/run-tests [module]` - Run tests
- `/run-demo [scenario]` - Run demos
- `/explain-module <name>` - Understand code
- `/check-phase <num>` - Check phase status
- `/benchmark` - Performance testing

## 📚 Project Structure
\`\`\`
wheeled_ur5e_aligator_mpc/
├── README.md              # Start here
├── ROADMAP.md             # Development plan
├── assets/                # MJCF models
├── wheeled_ur5e_aligator_mpc/  # Source code
├── scripts/               # Demo scripts
├── tests/                 # Test suite
└── logs/                  # Results
\`\`\`

## ⚙️ Common Commands
\`\`\`bash
cd study_example/wheeled_ur5e_aligator_mpc

# Activate environment (optional, for multiple commands)
eval "$(pixi shell-hook -e all)"

# Then you can drop the 'pixi run -e all' prefix
python scripts/run_demo.py --scenario ee_circle --render
pytest tests/ -v
\`\`\`
```

If any step fails, provide diagnostic suggestions.
