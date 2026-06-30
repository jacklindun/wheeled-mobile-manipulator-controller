# Check Phase

Check the status and health of a specific development phase.

## Usage

```
/check-phase <phase_number>
```

## Arguments

- `phase_number` (required): Phase to check (1-8)
  - `1-3`: Kinematic MPC (production-ready)
  - `4`: Hybrid kinodynamic MPC (known issues)
  - `5`: Wheeled dynamics (design phase)
  - `6`: MPC+WBC architecture (design phase)
  - `7`: Dual-arm extension (completed)
  - `8`: Dynamics matching (future)

## What it does

For the specified phase, provides a comprehensive health check:

### Status Overview
- Current state (✅ Complete / ⚠️ Issues / 📋 Design / 🚧 In Progress)
- Completion percentage
- Time invested
- Key deliverables

### Implementation Check
- Required files present?
- Tests passing?
- Demo scripts working?
- Documentation complete?

### Performance Validation
- Meets target metrics?
- Known issues documented?
- Regression vs previous phase?

### Dependencies
- Prerequisites satisfied?
- Blocks next phase?
- Integration status with other phases

### Recommendations
- What to do next?
- Known issues to fix?
- Optimization opportunities?
- References to design docs

## Examples

```
/check-phase 4
/check-phase 7
```

## Instructions

cd to study_example/wheeled_ur5e_aligator_mpc

### 1. Read Phase Documentation
- ROADMAP.md for phase definition
- PHASE_X_*.md files (DESIGN, PROGRESS, SUMMARY)
- README.md for overall status

### 2. Check Implementation
Based on phase:

**Phase 1-3 (Kinematic MPC)**:
- Files: robot_model.py, aligator_problem.py, aligator_mpc_controller.py
- Tests: test_robot_model.py, test_aligator_problem.py
- Demo: run_demo.py --scenario ee_circle

**Phase 4 (Hybrid Kinodynamic)**:
- Files: hybrid_dynamics.py, hybrid_problem.py, kinodynamic_mpc_controller.py
- Tests: test_hybrid_dynamics.py, test_hybrid_problem.py
- Demo: run_hybrid_demo.py
- Known issue: integrator mismatch (0.034 error over 25 steps)

**Phase 7 (Dual-arm)**:
- Files: dual_arm_pinocchio_model.py, dual_arm_aligator_problem.py
- Tests: test_dual_arm_*.py
- Demo: demo_dual_arm_mpc.py

### 3. Run Health Check
```bash
# Run relevant tests
pixi run -e all python -m pytest tests/test_[phase]_*.py -v

# Try demo if exists
pixi run -e all python scripts/[phase_demo].py --duration 5
```

### 4. Generate Report
```markdown
# Phase X Health Check

## Status: [✅/⚠️/📋/🚧]

### Implementation
- [ ] Core files present
- [ ] Tests passing (X/Y)
- [ ] Demo working
- [ ] Documentation complete

### Performance
- Target: [metric goals from ROADMAP]
- Actual: [measured performance]
- Gap: [analysis]

### Known Issues
1. [Issue description]
   - Impact: [severity]
   - Workaround: [if any]
   - Fix planned: [Phase X]

### Dependencies
- Depends on: Phase X (status)
- Blocks: Phase Y (status)

### Next Steps
1. [Recommended action]
2. [Recommended action]

### References
- Design: PHASE_X_DESIGN.md
- Progress: PHASE_X_PROGRESS.md
- Tests: tests/test_X_*.py
```

### 5. Specific Checks per Phase

**Phase 4**: Check for known integrator issue
```python
# Run compare_dynamics.py and check error magnitude
# Should see ~7e-5 single-step, ~0.034 multi-step
```

**Phase 7**: Verify dual-arm independence
```python
# Check Jacobian cross-coupling is near zero
# J_left[:, 10:16] ≈ 0 and J_right[:, 4:10] ≈ 0
```
