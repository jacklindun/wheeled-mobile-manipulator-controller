# Explain Module

Provide detailed explanation of a specific module's implementation, design decisions, and usage.

## Usage

```
/explain-module <module_name>
```

## Arguments

- `module_name` (required): Module to explain
  - `robot_model`: Basic kinematic/dynamic model
  - `pinocchio_model`: Pinocchio-based FK/Jacobian
  - `aligator_problem`: MPC problem construction
  - `mpc_controller`: MPC solver wrapper
  - `hybrid_dynamics`: Phase 4 kinodynamic dynamics
  - `dual_arm`: Dual-arm extensions
  - `wbc`: Whole-body control (Phase 6)
  - `mujoco_env`: MuJoCo simulation interface

## What it does

For the specified module, provides:

1. **Purpose & Scope**
   - What problem does it solve?
   - Where does it fit in the architecture?

2. **Key Classes & Functions**
   - Main API entry points
   - Important methods and their signatures
   - Data structures

3. **Design Decisions**
   - Why this approach?
   - Trade-offs made
   - Alternatives considered (if documented)

4. **Usage Examples**
   - How to instantiate
   - Common usage patterns
   - Integration with other modules

5. **Implementation Details**
   - Key algorithms
   - Mathematical formulations
   - Performance considerations

6. **Known Issues & Limitations**
   - Current constraints
   - Future improvements (from ROADMAP.md)
   - Related Phase status

7. **Related Files**
   - Source code location
   - Tests
   - Design documents
   - Usage examples

## Examples

```
/explain-module pinocchio_model
/explain-module aligator_problem
/explain-module hybrid_dynamics
```

## Instructions

1. Locate the module file in `wheeled_ur5e_aligator_mpc/`
2. Read the source code
3. Find related test files in `tests/test_[module].py`
4. Check relevant Phase documents (PHASE_X_*.md)
5. Search for usage in demo scripts

Structure the explanation as:
```
# Module: [name]

## Purpose
[What it does, why it exists]

## Architecture
[How it fits in the system, dependencies]

## Key Components
### Class: [ClassName]
- `method1(args) -> return`: [description]
- `method2(args) -> return`: [description]

## Design Highlights
[Important design decisions, trade-offs]

## Usage Example
```python
[code example]
```

## Implementation Notes
[Key algorithms, math, performance]

## Limitations
[Current issues, future work]

## Related
- Tests: tests/test_*.py
- Design: PHASE_X_*.md
- Demos: scripts/*.py
```

For complex modules like hybrid_dynamics or aligator_problem, include:
- State space definition
- Dynamics equations
- Cost function formulation
- Constraint handling
