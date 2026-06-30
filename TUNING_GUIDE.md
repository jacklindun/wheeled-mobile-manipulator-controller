# 混合MPC系统性调参指南

## 📋 调参目标

将混合MPC从当前状态：
- ❌ 收敛率：0%
- ❌ EE误差：2.5-5.0 cm
- ❌ 初始"后退"现象

提升到目标状态：
- ✓ 收敛率：>80%
- ✓ EE误差：<2.0 cm
- ✓ 平滑跟踪，无异常行为

---

## 🎯 五阶段调参策略

### 阶段1：求解器参数（最关键！）⭐⭐⭐⭐⭐

**目标**：找到能让求解器收敛的 `mu_init` 和 `tolerance`

**参数范围**：
- `mu_init`: 1e-1（当前）→ 1e-2, 2e-2, 5e-2
- `tolerance`: 1e-2（当前）→ 1e-3, 5e-3

**期望结果**：收敛率从 0% 提升到 >20%

**为什么重要**：
- `mu_init` 是增广拉格朗日惩罚参数，太大会导致约束违反惩罚太弱
- 收敛是一切的基础，不收敛的话其他调参都无意义

---

### 阶段2：EE跟踪权重 ⭐⭐⭐⭐

**前提**：阶段1找到了能收敛的求解器参数

**目标**：在收敛的基础上，降低跟踪误差

**参数范围**：
- `ee_pos`: 100（当前）→ 150, 200, 300, 500
- `terminal_ee_pos`: 200（当前）→ 对应翻倍

**期望结果**：EE RMS误差降低 20-30%

**权衡**：
- 权重太大：可能导致扭矩过大、动作剧烈
- 权重太小：跟踪精度差

---

### 阶段3：扭矩正则化 ⭐⭐⭐

**目标**：平衡跟踪精度与控制平滑性

**参数范围**：
- `tau_arm`: 0.001（当前）→ 0.005, 0.01, 0.02, 0.05
- `dtau_arm`: 0.01（当前）→ 保持 5倍 tau_arm

**期望结果**：
- 消除初始"后退"现象
- 扭矩更平滑，机械臂运动更自然

**权衡**：
- 正则化太强：限制了控制能力，误差增大
- 正则化太弱：扭矩剧烈变化，不稳定

---

### 阶段4：Horizon长度 ⭐⭐

**目标**：优化预测视野，平衡精度与计算效率

**参数范围**：
- `horizon`: 20（当前）→ 10, 15, 20, 25

**期望结果**：
- 找到最佳的精度/速度平衡点
- 求解时间 <50ms（实时性）

**权衡**：
- Horizon太长：计算慢，可能更难收敛
- Horizon太短：预测不足，跟踪误差大

---

### 阶段5：基座/姿态约束 ⭐

**目标**：降低冗余约束，给系统更多自由度

**参数范围**：
- `base_xy`, `base_yaw`, `base_z`, `arm_posture`: 当前值 × (0.5, 0.3, 0.1)

**期望结果**：
- 系统更灵活，能找到更优的全身协调方案
- 可能进一步降低误差

**权衡**：
- 约束太松：可能偏离期望姿态
- 约束太紧：限制了解空间

---

## 🚀 执行步骤

### 快速测试（推荐先运行）

```bash
cd study_example/wheeled_ur5e_aligator_mpc

# 只测试阶段1（最关键，约10分钟）
pixi run -e all python scripts/tune_hybrid_systematic.py --phase 1

# 查看结果，如果收敛率>20%，继续阶段2
pixi run -e all python scripts/tune_hybrid_systematic.py --phase 2
```

### 完整调参（约1-2小时）

```bash
# 运行所有5个阶段
pixi run -e all python scripts/tune_hybrid_systematic.py --phase all

# 如果想看可视化（会慢很多）
pixi run -e all python scripts/tune_hybrid_systematic.py --phase all --render
```

### 单独调参某个阶段

```bash
# 阶段1: 求解器参数（6个配置 × 20秒 = 2分钟）
pixi run -e all python scripts/tune_hybrid_systematic.py --phase 1

# 阶段2: EE权重（4个配置 × 20秒 = 1.5分钟）
pixi run -e all python scripts/tune_hybrid_systematic.py --phase 2

# 阶段3: 扭矩正则化（4个配置）
pixi run -e all python scripts/tune_hybrid_systematic.py --phase 3

# 阶段4: Horizon（4个配置）
pixi run -e all python scripts/tune_hybrid_systematic.py --phase 4

# 阶段5: 姿态约束（3个配置）
pixi run -e all python scripts/tune_hybrid_systematic.py --phase 5
```

---

## 📊 如何解读结果

脚本会自动输出每个配置的：

```
配置名称                  收敛率      EE RMS    EE Max    求解时间    迭代数
-------------------------------------------------------------------------
Baseline                  0.0%       2.60cm    3.50cm     75.0ms     50.0
mu_1e-2                  35.5%       1.85cm    2.80cm     52.3ms     28.4  ← 最佳！
mu_5e-2                  18.2%       2.12cm    3.15cm     61.7ms     35.8
...
```

**评分标准**：
- **收敛率权重60%**：必须收敛才有意义
- **RMS误差权重40%**：在收敛基础上越小越好

脚本会自动选出最佳配置，作为下一阶段的起点。

---

## ✅ 成功标准

### 最小目标（可接受）
- 收敛率 >50%
- EE RMS <2.0 cm
- 无明显"后退"或异常行为

### 理想目标（优秀）
- 收敛率 >80%
- EE RMS <1.5 cm
- 求解时间 <50 ms
- 接近运动学MPC的性能

### 对比基线（运动学MPC Phase 1-3）
- 收敛率：100%
- EE RMS：1.5-2.1 cm
- 求解时间：15 ms

---

## 🔧 调参技巧

### 1. 优先级策略
按阶段顺序执行，不要跳过！每个阶段依赖前面的结果。

### 2. 增量调整
如果某个参数改动后效果变差，回退到上一步，尝试更小的步长。

### 3. 记录结果
脚本会自动输出结果，建议保存到文件：
```bash
pixi run -e all python scripts/tune_hybrid_systematic.py --phase all | tee tuning_results.log
```

### 4. 可视化验证
找到好的配置后，用渲染模式验证：
```bash
pixi run -e all python scripts/test_hybrid_scenarios.py \
    --scenario ee_circle --duration 30 --render
```
手动观察"后退"现象是否消失。

### 5. 多场景测试
最优参数在 ee_circle 找到后，在其他场景验证：
```bash
pixi run -e all python scripts/test_hybrid_scenarios.py --all --duration 30
```

---

## 🐛 预期问题与解决

### 问题1：阶段1所有配置收敛率都是0%

**可能原因**：
- 动力学模型与MuJoCo不匹配
- 关节阻尼参数错误
- ABA计算有bug

**解决方案**：
- 检查 `hybrid_dynamics.py` 中的阻尼参数
- 对比单步ABA预测 vs MuJoCo执行的差异
- 尝试更小的 mu_init（1e-3, 1e-4）

### 问题2：收敛了但误差反而更大

**可能原因**：
- 收敛到了局部最优（次优解）
- 权重不平衡

**解决方案**：
- 增加EE权重（阶段2）
- 降低姿态约束（阶段5）
- 改善warm-start（从运动学解初始化）

### 问题3：某些配置导致仿真崩溃

**可能原因**：
- 扭矩过大超出物理限制
- 数值不稳定

**解决方案**：
- 增强扭矩正则化
- 检查扭矩边界（-150~150 Nm）

---

## 📈 预期时间表

| 阶段 | 配置数 | 单次时长 | 总耗时 | 重要性 |
|------|--------|---------|--------|--------|
| 阶段1 | 6 | 20s | ~2分钟 | ⭐⭐⭐⭐⭐ |
| 阶段2 | 4 | 20s | ~1.5分钟 | ⭐⭐⭐⭐ |
| 阶段3 | 4 | 20s | ~1.5分钟 | ⭐⭐⭐ |
| 阶段4 | 4 | 20s | ~1.5分钟 | ⭐⭐ |
| 阶段5 | 3 | 20s | ~1分钟 | ⭐ |
| **总计** | **21** | - | **~8分钟** | - |

**实际耗时可能更长**（求解时间+环境重置开销），预留15-20分钟。

---

## 🎯 下一步

1. **先运行阶段1**（最关键！）
   ```bash
   cd study_example/wheeled_ur5e_aligator_mpc
   pixi run -e all python scripts/tune_hybrid_systematic.py --phase 1 | tee phase1_results.txt
   ```

2. **查看结果**，如果收敛率 >20%，说明有希望，继续后续阶段

3. **如果阶段1失败**（收敛率仍为0%），需要深入诊断动力学模型

4. **最终验证**：用最优配置测试全部4个场景
   ```bash
   pixi run -e all python scripts/test_hybrid_scenarios.py --all --duration 30
   ```

---

## 📝 结果记录模板

```
=== 调参结果记录 ===
日期：2026-06-XX
测试场景：ee_circle, 20秒

阶段1最佳：mu_init=___, tolerance=___, 收敛率=___%, RMS=___cm
阶段2最佳：ee_pos=___, 收敛率=___%, RMS=___cm
阶段3最佳：tau_arm=___, 收敛率=___%, RMS=___cm
阶段4最佳：horizon=___, 收敛率=___%, RMS=___cm
阶段5最佳：posture_scale=___, 收敛率=___%, RMS=___cm

最终配置：
{
    "mu_init": ___,
    "tolerance": ___,
    "horizon": ___,
    "weights": {
        "ee_pos": ___,
        "tau_arm": ___,
        ...
    }
}

最终性能：
- 收敛率：___%
- EE RMS：___cm
- EE Max：___cm
- 求解时间：___ms
- 是否有"后退"：是/否
```

---

好运！🚀
