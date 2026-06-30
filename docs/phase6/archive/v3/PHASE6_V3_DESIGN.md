# Phase 6-v3: 全动力学MPC + 前馈PD控制

## 架构设计

```
┌─────────────────────────────────────────────────────────────────────┐
│ Phase 6-v3 架构                                                      │
│                                                                      │
│  动力学MPC (20Hz)     插值器 (25:1)      前馈PD (500Hz)   MuJoCo    │
│  ┌──────────────┐    ┌──────────┐      ┌────────────┐   ┌────────┐ │
│  │ ALIGATOR     │───>│ 轨迹插值  │─────>│ τ_mpc +    │──>│ Torque │ │
│  │ DDP Solver   │    │ q,v,τ    │      │ PD(q,v)    │   │ Ctrl   │ │
│  └──────────────┘    └──────────┘      └────────────┘   └────────┘ │
│       ↑                                       ↑                      │
│  目标轨迹 (EE)                            当前状态 (q,v)             │
└─────────────────────────────────────────────────────────────────────┘
```

## 与Phase 6-v2的区别

| 方面 | Phase 6-v2 (运动学) | Phase 6-v3 (动力学) |
|------|-------------------|-------------------|
| MPC类型 | 运动学IK | 动力学DDP |
| 求解器 | 手写IK | ALIGATOR |
| 状态空间 | q (16维) | (q, v) (32维) |
| 控制输出 | 位置 q_des | 力矩 τ |
| 执行器 | Position actuator | Torque actuator |
| 动力学 | ✗ | ✓ (完整刚体动力学) |
| 计算开销 | 低 (~1ms) | 高 (~20-50ms) |

## 关键组件

### 1. 动力学MPC (ALIGATOR)

**使用现有的 `aligator_problem_builtin.py`**

```python
# 状态: x = [q, v]  (32维)
# 控制: u = τ      (16维)

# 成本函数:
# - 状态跟踪: ||q - q_ref||²_W_q + ||v - v_ref||²_W_v
# - 控制正则化: ||τ||²_W_u
# - 末端执行器位置: ||p_ee - p_target||²_W_ee

# 约束:
# - 动力学约束: q̇ = v, M(q)v̇ + C(q,v) = τ
# - 关节限位: q_min ≤ q ≤ q_max
# - 速度限制: v_min ≤ v ≤ v_max
# - 力矩限制: τ_min ≤ τ ≤ τ_max
```

### 2. 轨迹插值器

**扩展现有的 `TrajectoryInterpolator`**

```python
# 输入: MPC轨迹
# - xs: (N+1, 32) 状态轨迹 [q, v]
# - us: (N, 16) 力矩轨迹 τ
# - ts: (N+1,) 时间点

# 输出: 插值后的参考
# - q_des(t), v_des(t), τ_feedforward(t)

# 插值方法: 线性插值
```

### 3. 前馈PD控制器

**修改 `FeedforwardPDController` 支持力矩输出**

```python
def compute_torque_control(
    q_current: np.ndarray,  # 当前关节位置
    v_current: np.ndarray,  # 当前关节速度
    q_des: np.ndarray,      # 期望关节位置
    v_des: np.ndarray,      # 期望关节速度
    τ_feedforward: np.ndarray,  # MPC输出的前馈力矩
) -> np.ndarray:
    """
    计算总力矩控制
    
    τ_total = τ_feedforward + Kp*(q_des - q_current) + Kd*(v_des - v_current)
    """
    q_error = q_des - q_current
    v_error = v_des - v_current
    
    τ_pd = Kp * q_error + Kd * v_error
    τ_total = τ_feedforward + τ_pd
    
    # 力矩限幅
    τ_total = np.clip(τ_total, τ_min, τ_max)
    
    return τ_total
```

### 4. MuJoCo模型

**创建力矩执行器版本**

```xml
<!-- 替换 position actuator 为 motor (torque) actuator -->
<actuator>
  <motor name="act_base_x" joint="base_x" gear="1" 
         ctrllimited="true" ctrlrange="-100 100"/>
  <motor name="act_left_shoulder_pan" joint="left_shoulder_pan_joint" 
         gear="1" ctrllimited="true" ctrlrange="-150 150"/>
  ...
</actuator>
```

## 实现步骤

### Step 1: 创建力矩执行器模型
- [x] 规划架构
- [ ] 复制 `wheeled_dual_ur5e_v2.xml`
- [ ] 替换所有 `<position>` 为 `<motor>`
- [ ] 设置合理的力矩限制

### Step 2: 修改ALIGATOR问题
- [ ] 使用 `aligator_problem_builtin.py` 作为基础
- [ ] 配置双臂末端执行器代价
- [ ] 调整权重矩阵

### Step 3: 扩展轨迹插值器
- [ ] 支持 (q, v, τ) 三元组插值
- [ ] 验证插值精度

### Step 4: 扩展前馈PD控制器
- [ ] 添加 `compute_torque_control()` 方法
- [ ] 支持力矩前馈

### Step 5: 集成测试
- [ ] 创建 `test_phase6_v3_simple.py`
- [ ] 圆形轨迹测试
- [ ] 性能对比

### Step 6: 优化调参
- [ ] MPC权重调优
- [ ] PD增益调优
- [ ] 求解器参数调优

## 预期性能

| 指标 | Phase 6-v2 | Phase 6-v3 (目标) |
|------|-----------|------------------|
| 平均跟踪误差 | 14.5 cm | < 5 cm |
| IK残差 | 0.004 cm | N/A (动力学) |
| 计算频率 | 20 Hz | 20 Hz |
| 控制频率 | 500 Hz | 500 Hz |
| 计算时间/步 | ~1 ms | ~20-50 ms |

## 潜在挑战

1. **计算性能**: ALIGATOR DDP求解可能超过50ms
   - 解决: 减少horizon长度
   - 解决: 使用warm start

2. **稳定性**: 力矩控制可能不稳定
   - 解决: 保守的PD增益
   - 解决: 力矩限幅

3. **碰撞**: 动力学考虑碰撞会复杂化
   - 解决: 先禁用碰撞，后期添加

4. **调参**: 更多超参数需要调优
   - 解决: 从Phase 5经验开始
   - 解决: 系统化调参流程

## 参考文件

- `wheeled_ur5e_aligator_mpc/aligator_problem_builtin.py` - ALIGATOR问题定义
- `wheeled_ur5e_aligator_mpc/trajectory_interpolator.py` - 轨迹插值
- `wheeled_ur5e_aligator_mpc/feedforward_pd_controller.py` - PD控制器
- `scripts/demo_phase6_v2.py` - Phase 6-v2参考实现

## 开始实现

准备好开始了吗？我们从Step 1开始：创建力矩执行器模型。
