# Phase 6-v3 修复进度报告

## 日期
2026-06-26

## 已完成的修复

### 1. ✅ 坐标顺序统一
- **问题**：Pinocchio/qpos 和 MuJoCo ctrl 的 base 顺序不一致
  - qpos: `[x, y, yaw, z, arms...]`
  - ctrl: `[x, y, z, yaw, arms...]`
  
- **修复**：
  - 创建 `coordinate_mapping.py`，定义 `q_to_ctrl()` 和 `ctrl_to_q()` 映射函数
  - 验证通过：映射正确，可逆

### 2. ✅ Base nominal 配置修正
- **问题**：多处将 base nominal 写成 `[0.0, 0.0, 0.2, 0.0]`（错误地将 z 设为 yaw）
- **修复**：
  - `dual_arm_dynamics_mpc.py`: 改为 `[0.0, 0.0, 0.0, 0.2]`
  - 其他文件待修复

### 3. ✅ 力矩限制修正
- **问题**：力矩限制顺序错误
- **修复**：
  - `dual_arm_dynamics_mpc.py`: 改为 `[200, 200, 100, 1000]` (x, y, yaw, z)

### 4. ✅ MPC 添加控制约束
- **问题**：MPC 没有力矩约束，输出超限力矩（-235~275 N·m）
- **修复**：
  - 在 `build_problem()` 中为每个 stage 添加 `BoxConstraint`
  - 效果：力矩范围降到 -200~150 N·m（基本在限制内）

### 5. ✅ 插值器逻辑修正
- **问题**：将整个 MPC horizon 当作 500Hz 段插值，时间尺度严重压缩
- **修复**：
  - 每个 MPC 周期只执行第一段 `[xs[0], xs[1]]`
  - 用 ratio=25 个 tick 插值
  - `tau_ff` 使用 `us[0]`，不插值

### 6. ✅ 控制输出添加坐标映射
- **修复**：
  - `test_phase6_v3_step2.py`: `tau_ctrl_order = q_to_ctrl(tau_control)`
  - `test_phase6_v3_step1.py`: 同样添加映射

## 测试结果

### Step 2 (MPC+PD) 误差变化
| 版本 | 平均误差 | 说明 |
|------|----------|------|
| 初始版本 | 35.63 cm | 无约束，插值错误 |
| +控制约束 | 86.71 cm | 添加约束后反而变差 |
| +坐标映射 | 86.71 cm | 无改善 |
| +插值修正 | **47.41 cm** | 有改善，但仍远差于纯PD |
| 目标 | 1.79 cm | Step 1 纯PD的性能 |

## 当前问题

### 1. ❌ 测试超时
- **现象**：`test_phase6_v3_step1.py` 和 `test_phase6_v3_step2.py` 都会超时
- **可能原因**：
  - MuJoCo viewer 在无 GUI 环境下阻塞
  - 某处存在死循环
  - 控制不稳定导致仿真发散

### 2. ❌ MPC 性能仍然很差
- **现象**：47.41cm vs 1.79cm（纯PD）
- **可能原因**（按你的分析）：
  - ✅ 插值逻辑：已修复
  - ✅ 坐标顺序：已修复
  - ⚠️  MPC 代价函数重复加权（未修复）
  - ⚠️  warm start 没有使用 xs（未修复）
  - ⚠️  没有 IK q_ref 轨迹（未修复）
  - ⚠️  没有检查收敛性（未修复）

### 3. ⚠️  MPC 输出仍然接近约束边界
- **现象**：第一个控制 `[-158, -200, 100, 105, 150, ...]`
- base_y 达到 -200（边界）
- 说明约束可能太紧或问题设置不合理

## 未完成的修复

### 优先级 1：解决测试超时
需要诊断为什么测试会超时。可能需要：
1. 添加 `--headless` 模式
2. 检查控制是否稳定
3. 添加超时保护

### 优先级 2：修复 MPC 代价函数
根据你的分析：
```python
# 当前（可能重复加权）：
cost_left = aligator.QuadraticResidualCost(self.space, res_left, np.eye(3) * w_left)
cost_stack.addCost("ee_left", cost_left, w_left)  # 再乘一次？

# 修正：
cost_left = aligator.QuadraticResidualCost(self.space, res_left, np.eye(3))
cost_stack.addCost("ee_left", cost_left, w_left)
```

### 优先级 3：改进 warm start
当前每次创建新 solver，无法利用上次的 xs。应该：
1. Solver 持久化，复用
2. 使用上次的完整 xs 轨迹作为初始化

### 优先级 4：添加收敛性检查
```python
if not results.conv:
    # Fallback: 不使用 MPC 输出
    tau_ff = np.zeros(16)
```

## 下一步建议

1. **立即**：诊断并修复测试超时问题
2. **然后**：修复 MPC 代价函数重复加权
3. **最后**：如果性能仍不理想，考虑简化方案（纯 PD 或运动学 MPC）

## 代码文件状态

| 文件 | 状态 |
|------|------|
| `coordinate_mapping.py` | ✅ 新建，已验证 |
| `dual_arm_dynamics_mpc.py` | ⚠️  部分修复（base顺序、约束） |
| `test_phase6_v3_step1.py` | ⚠️  添加映射，但超时 |
| `test_phase6_v3_step2.py` | ⚠️  添加映射+插值修复，但超时 |

## 关键发现

1. **坐标映射是必要的**：Pinocchio 和 MuJoCo 的 base 顺序确实不同
2. **插值修复有效**：误差从 87cm 降到 47cm
3. **但仍有根本问题**：47cm 仍然比纯 PD 的 1.79cm 差 26 倍

这说明即使修复了所有结构性错误，动力学 MPC 的性能仍然不理想。可能需要重新考虑整个架构。
