# Phase 6-v3 动力学MPC 状态报告

## 日期
2026-06-26

## 目标
实现全动力学MPC + 力矩控制，目标跟踪误差 < 1cm

## 当前状态

### ✅ 已完成
1. **架构实现**
   - 创建了 `DualArmDynamicsMPC` 类，使用 ALIGATOR 的 `MultibodyPhaseSpace`
   - 实现了完整的动力学模型：`MultibodyFreeFwdDynamics` + `IntegratorSemiImplEuler`
   - 集成了末端执行器跟踪代价（`FrameTranslationResidual`）
   - 实现了 MPC → 插值 → 前馈PD → MuJoCo 的完整控制链

2. **技术突破**
   - **解决了 ALIGATOR Python 绑定的多个 API 问题**：
     * 初始化列表：第一次调用使用空列表 `[]`，后续使用 warm start
     * Warm start 格式：`u_init` 作为 `(N, nu)` 数组传入，需要转换为 list
     * 问题构建：使用 `CostStack` 而非 `StageModel` 作为终端代价
   - **MPC 求解成功**：每次求解耗时约 36ms，满足 20Hz 实时要求

3. **测试结果**
   - **Step 1（纯 PD 控制）**: 1.79cm ✅ 优秀
   - **Step 2（MPC+PD 控制）**: 35.63cm ❌ 不理想

### ❌ 当前问题

**主要问题：MPC 跟踪误差远大于纯 PD 控制**

可能原因分析：

1. **模型不匹配**
   - ALIGATOR 使用 Pinocchio 模型（从 MJCF 加载）
   - MuJoCo 使用自己的动力学模型
   - 两者的动力学可能存在差异（惯性参数、摩擦、数值积分方法）

2. **Horizon 太短**
   - 当前：10 步 × 0.05s = 0.5 秒
   - 圆形轨迹周期：2π/0.5 ≈ 12.6 秒
   - MPC 只能看到 4% 的轨迹，预测能力受限

3. **优化未充分收敛**
   - 当前最大迭代次数：20
   - 可能需要更多迭代或更好的初始猜测

4. **权重调优不足**
   - 当前末端执行器权重：10000
   - 当前控制正则化：0.001
   - 可能需要进一步调整平衡

5. **插值问题**
   - MPC 输出的力矩轨迹在 500Hz 插值时可能引入误差
   - 线性插值可能不适合力矩信号

## 性能对比

| 方法 | 平均误差 | 最大误差 | 求解时间 | 实时性 |
|------|----------|----------|----------|--------|
| Phase 6-v2 (位置+IK) | 14.5 cm | - | - | ✅ |
| Phase 6-v3 Step 1 (力矩+PD) | 1.79 cm | 8.40 cm | - | ✅ |
| Phase 6-v3 Step 2 (MPC+PD) | 35.63 cm | 58.13 cm | 36.1 ms | ✅ |

## 下一步建议

### 优先级 1：诊断模型不匹配
1. **对比预测 vs 实际**
   - 记录 MPC 预测的状态轨迹 `xs`
   - 记录 MuJoCo 实际执行后的状态
   - 计算预测误差，识别模型不匹配的来源

2. **简化测试**
   - 测试单臂静态目标点跟踪
   - 验证 ALIGATOR 动力学模型是否准确

### 优先级 2：调优策略
1. **增加 Horizon**
   - 尝试 horizon=20 (1秒预测)
   - 权衡求解时间 vs 预测能力

2. **改进初始化**
   - 使用上一次的完整轨迹 `xs` 作为 warm start
   - 而不只是 `us`

3. **权重扫描**
   - 系统地测试不同的权重组合
   - 绘制误差-权重曲线

### 优先级 3：替代方案
如果动力学 MPC 性能无法改善，考虑：

1. **运动学 MPC + 逆动力学**
   - 使用运动学 MPC 优化关节轨迹（类似 Phase 6-v2）
   - 用逆动力学计算所需力矩
   - 比当前的动力学 MPC 简单，可能更稳定

2. **混合控制**
   - 保持纯 PD 控制（已验证性能优秀）
   - 将 MPC 用于高层规划（轨迹优化）
   - 而非低层力矩控制

## 代码位置

- **MPC 实现**: `wheeled_ur5e_aligator_mpc/dual_arm_dynamics_mpc.py`
- **测试脚本**: `scripts/test_phase6_v3_step2.py`
- **Step 1 测试**: `scripts/test_phase6_v3_step1.py` (基准：1.79cm)

## 关键发现

### ALIGATOR API 正确用法
```python
# 1. 创建 solver
solver = aligator.SolverProxDDP(tol, mu_init)
solver.max_iters = max_iters
solver.setup(problem)

# 2. 第一次调用：空列表
solver.run(problem, [], [])

# 3. 后续调用：使用上次结果作为 warm start
xs_init = [xs[i] for i in range(len(xs))]  # list of arrays
us_init = [us[i] for i in range(len(us))]
solver.run(problem, xs_init, us_init)
```

### Python 绑定陷阱
- ❌ `xs_init = [x0.copy() for _ in range(N)]` - 会导致类型错误
- ❌ `xs_init = [x0] * N` - 共享引用，但也会导致类型错误
- ✅ `xs_init = []` - 第一次调用时使用空列表
- ✅ `xs_init = [results.xs[i] for i in range(N)]` - warm start

## 结论

Phase 6-v3 的动力学 MPC 架构已经**技术上可行**（MPC 能够求解），但**性能上不理想**（误差比纯 PD 大 20 倍）。

建议优先诊断模型不匹配问题，如果无法解决，考虑回退到运动学 MPC 或保持纯 PD 控制方案。
