# 移动机械臂全动力学MPC项目路线图

## 📋 项目目标

从当前的混合MPC（基座速度+臂扭矩）演进到完整的全动力学MPC系统，支持：
- 轮子真实动力学约束
- WBC（Whole-Body Control）QP框架
- 多臂+body多自由度
- 最终统一解决动力学匹配问题

---

## ✅ 已完成（Phase 1-4）

### Phase 1-3: 运动学MPC（生产就绪）
- ✅ Pinocchio后端（FK/Jacobian机器精度）
- ✅ 6D姿态控制（SO(3) log map）
- ✅ 硬状态箱约束
- ✅ 性能：收敛率100%，EE误差1.5-2.1cm

### Phase 4: 混合Kino-Dynamic MPC（技术验证）
- ✅ 16维状态空间 [q_base(4), q_arm(6), v_arm(6)]
- ✅ 混合控制 [v_base(4), tau_arm(6)]
- ✅ ABA动力学集成
- ✅ 闭环稳定运行（51/51测试）
- ⚠️ 已知问题：积分器不匹配（单步误差7e-5，25步累积0.034）
- ⚠️ 求解器收敛率0%（待后续统一解决）

---

## 🚀 项目路线（Phase 5-8）

### Phase 5: 轮子真实动力学约束 ⭐⭐⭐
**目标**：从虚拟基座运动学 → 轮式移动底盘动力学

**技术要点**：
- 差速驱动动力学模型（unicycle或differential drive）
- 非完整约束（non-holonomic constraints）
- 滑移/打滑建模
- 轮速→基座速度映射

**状态空间**（预计20-dim）：
```
x = [q_base(4), q_arm(6), v_base(3), v_arm(6), ω_wheel(2)]
  其中 v_base = [vx, vy, ω_yaw] （body-frame）
```

**控制空间**（预计8-dim）：
```
u = [τ_wheel_left, τ_wheel_right, τ_arm(6)]
```

**关键挑战**：
- 非完整约束的处理（ALIGATOR约束API）
- 轮地接触模型
- 基座动力学（质量、惯性）

**验证标准**：
- 轮速约束满足（无侧滑）
- 基座加速度合理
- 闭环稳定

---

### Phase 6: MPC+WBC双层架构 ⭐⭐⭐⭐⭐
**目标**：MPC规划轨迹 + WBC实时跟踪，分离规划与控制

**关键理念**：
- **MPC层**：高层规划，优化全身轨迹
- **WBC层**：低层控制，实时力矩求解
- **解耦优势**：MPC不需要精确动力学，WBC保证执行正确

**双层架构**：
```
┌─────────────────────────────────────────────────────┐
│  MPC层 (Outer Loop) - 轨迹规划                        │
│  ─────────────────────────────────────────────────  │
│  输入: x_current, 参考轨迹 ref                         │
│  优化: min Σ cost(x, u, ref)                         │
│        s.t. x_next = f(x, u)  (近似动力学！)           │
│             约束 (关节限位、速度限制)                    │
│  输出: 最优轨迹 x*[0:N], u*[0:N-1]                     │
│  频率: 10-20 Hz (可以较慢)                             │
└─────────────────────────────────────────────────────┘
                        ↓
              x_des[1], v_des[1], a_des[1]
                        ↓
┌─────────────────────────────────────────────────────┐
│  WBC层 (Inner Loop) - 力矩求解                        │
│  ─────────────────────────────────────────────────  │
│  输入: x_current, x_des, v_des, a_des                │
│  QP问题:                                              │
│    minimize   ||M(q)a + h(q,v) - S^T τ||²           │
│               + ||a - a_des||²_W                     │
│               + ||x - x_des||²_P (可选位置跟踪)        │
│                                                       │
│    subject to τ_min ≤ τ ≤ τ_max                      │
│               接触约束 (摩擦锥、接触力)                  │
│               非完整约束 (轮子)                         │
│                                                       │
│  输出: 最优力矩 τ*                                     │
│  频率: 100-500 Hz (实时控制频率)                       │
└─────────────────────────────────────────────────────┘
                        ↓
                  发送到机器人/仿真
```

**为什么这样设计？**

1. **MPC可以使用简化动力学**
   - 运动学MPC（Phase 1-3）已经证明有效
   - 不需要精确的扭矩计算
   - WBC负责将速度/加速度目标转换为正确的力矩

2. **WBC保证动力学一致性**
   - 使用精确的质量矩阵 M(q)
   - 考虑科氏力/重力 h(q,v)
   - 满足所有物理约束

3. **频率解耦**
   - MPC: 10-20 Hz（慢但全局优化）
   - WBC: 100-500 Hz（快速局部跟踪）

4. **鲁棒性**
   - MPC规划的轨迹即使有小误差，WBC也能跟踪
   - 解决Phase 4的动力学匹配问题！

**与Phase 4的区别**：

| 方面 | Phase 4（混合MPC） | Phase 6（MPC+WBC） |
|------|------------------|-------------------|
| MPC输出 | 直接输出力矩 | 输出轨迹（位置/速度/加速度） |
| 动力学精度要求 | 必须精确匹配 | 可以近似（运动学即可） |
| 实时性 | MPC频率=控制频率 | MPC可以更慢 |
| 动力学一致性 | MPC层负责 | WBC层保证 |
| 约束处理 | MPC中硬约束 | WBC中实时QP求解 |

**技术要点**：

1. **MPC层（可复用Phase 1-3的运动学MPC！）**
   ```python
   # 使用运动学MPC规划
   mpc_result = kinematic_mpc.solve(x_current, ref_traj)
   
   # 提取期望轨迹
   x_des = mpc_result.xs[1]  # 下一步期望状态
   u_des = mpc_result.us[0]  # 期望速度
   
   # 计算期望加速度（数值微分或从动力学）
   a_des = (u_des - u_current) / dt
   ```

2. **WBC层（新增）**
   ```python
   # 构建QP问题
   qp = build_wbc_qp(x_current, x_des, v_des, a_des)
   
   # 求解
   tau_opt = solve_qp(qp)  # ProxQP/OSQP
   
   # 发送到机器人
   robot.set_torque(tau_opt)
   ```

3. **QP问题详细形式**
   ```python
   # 决策变量: [a, f_contact, τ]
   # a: 全身加速度 (n-dim)
   # f_contact: 接触力 (可选，如果有地面接触)
   # τ: 关节力矩 (n-dim)
   
   # 目标函数
   cost = w_dynamics * ||M*a + h - S^T*τ - J^T*f||²  # 动力学一致性
        + w_track    * ||a - a_des||²                # 跟踪期望加速度
        + w_reg      * ||τ||²                        # 力矩正则化
        + w_smooth   * ||τ - τ_prev||²               # 力矩平滑
   
   # 约束
   τ_min ≤ τ ≤ τ_max              # 力矩限制
   q_min ≤ q + dt*v + 0.5*dt²*a ≤ q_max  # 关节限位预测
   ||f_contact|| ≤ μ*f_normal    # 摩擦锥（如果有接触）
   v_lateral = 0                  # 非完整约束（轮子）
   ```

**工具选择**：
- **QP求解器**：ProxQP（首选，ALIGATOR生态）/ OSQP / qpOASES
- **动力学**：Pinocchio（RNEA, CRBA, computeGeneralizedGravity）
- **MPC**：复用Phase 1-3的运动学MPC（已验证有效）

**实现步骤**：

1. **Week 1**：WBC QP构建
   - 实现动力学项（M, h）
   - 构建QP问题（ProxQP）
   - 单步测试（给定a_des，验证τ输出）

2. **Week 2**：MPC+WBC集成
   - MPC输出 → WBC输入的接口
   - 闭环测试
   - 性能调优

**验证标准**：
- [ ] WBC QP求解时间 <1ms
- [ ] 动力学残差 ||M*a + h - S^T*τ|| <1e-3
- [ ] MPC+WBC闭环收敛率 >80%
- [ ] EE跟踪误差 <2cm
- [ ] 力矩平滑（无突变）

**预期效果**：
- ✅ 解决Phase 4的动力学匹配问题（MPC不再需要精确动力学）
- ✅ 更好的鲁棒性（WBC实时补偿）
- ✅ 更灵活的架构（MPC和WBC可以独立优化）

---

### Phase 7: 扩展到双臂+Body多自由度 ⭐⭐⭐
**目标**：从单臂10-DOF → 双臂+torso 20-30 DOF

**系统配置**：
```
基座: 4-DOF (x, y, z, yaw)
Torso: 2-3 DOF (pitch, roll, yaw)
左臂: 6-7 DOF
右臂: 6-7 DOF
总计: 18-22 DOF
```

**状态空间**（预计40-50 dim）：
```
x = [q_full(20-22), v_full(20-22)]
```

**关键挑战**：
- 模型复杂度（质量矩阵40×40）
- 计算效率（求解时间可能>100ms）
- 双臂协调任务
- 冗余度利用

**协调任务示例**：
- 双臂搬运（保持物体姿态）
- 一臂操作+一臂支撑
- Torso补偿基座运动

**验证标准**：
- 求解时间可接受（<200ms for 20Hz MPC）
- 双臂任务精度
- 无自碰撞

---

### Phase 8: 统一解决动力学匹配问题 ⭐⭐⭐⭐⭐
**目标**：彻底解决ALIGATOR预测 vs MuJoCo执行的不匹配

**问题总结**：
- 单步误差：7e-5（可接受）
- 多步累积：0.034（480倍放大）
- 根本原因：积分器差异（semi-implicit Euler vs implicitfast）

**解决方案（按优先级）**：

#### 方案A：匹配MuJoCo积分器（推荐）⭐⭐⭐⭐⭐
实现MuJoCo的implicit积分器在ALIGATOR中：

```python
# Implicit Euler: solve for x_next, v_next simultaneously
# M(q_next) * (v_next - v) = dt * f(q_next, v_next, u)
# q_next = q + dt * v_next

# Newton迭代求解非线性方程组
for iter in range(max_newton_iters):
    residual = compute_residual(x_next, x, u, dt)
    jacobian = compute_jacobian(x_next, u, dt)
    delta = solve(jacobian, -residual)
    x_next += delta
    if norm(residual) < tol:
        break
```

**技术要点**：
- 实现implicit Euler的Newton迭代
- 计算残差和雅可比（Pinocchio支持）
- 可能需要ALIGATOR自定义动力学类

**优点**：
- 根本性解决匹配问题
- 数值稳定性更好
- 可以使用更大的dt

**缺点**：
- 实现复杂度高
- 计算量增加（Newton迭代）

---

#### 方案B：使用RK4积分器 ⭐⭐⭐
MuJoCo支持RK4，ALIGATOR也支持：

```python
# 在MJCF中设置
<option integrator="RK4"/>

# 在ALIGATOR中实现RK4动力学
class RK4Dynamics(aligator.dynamics.ExplicitDynamicsModel):
    def forward(self, x, u, data):
        k1 = self.f(x, u)
        k2 = self.f(x + dt/2 * k1, u)
        k3 = self.f(x + dt/2 * k2, u)
        k4 = self.f(x + dt * k3, u)
        data.xnext = x + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
```

**优点**：
- 实现相对简单
- 高阶精度（O(dt^4)）
- 两边都用标准积分器

**缺点**：
- MuJoCo的RK4可能对刚性系统不稳定
- 计算量增加4倍

---

#### 方案C：子步积分 ⭐⭐⭐
让ALIGATOR也做MuJoCo的25个子步：

```python
def forward_with_substeps(self, x, u, data):
    dt_sub = self.dt / self.substeps  # 0.05 / 25 = 0.002
    x_current = x.copy()
    
    for i in range(self.substeps):
        # Semi-implicit Euler子步
        a = self.compute_acceleration(x_current, u)
        v_next = x_current[10:] + dt_sub * a
        q_next = x_current[:10] + dt_sub * v_next
        x_current = np.concatenate([q_next, v_next])
    
    data.xnext = x_current
```

**优点**：
- 实现简单
- 减少累积误差

**缺点**：
- 计算量增加25倍
- 梯度计算复杂（需要链式法则）
- 可能仍有残差

---

#### 方案D：降低控制频率 ⭐⭐
使用dt = 0.002（MuJoCo原生步长），避免多步积分：

```python
# MPC频率: 500 Hz (dt=0.002)
# Horizon: 50-100 (保持预测视野0.1-0.2s)
```

**优点**：
- 完全消除累积误差
- 单步预测精确

**缺点**：
- 计算量巨大（horizon变大）
- 求解时间可能>1s
- 不现实

---

#### 方案E：模型学习校正 ⭐⭐
用神经网络学习残差：

```python
x_next_nominal = forward_dynamics(x, u)  # ALIGATOR预测
x_next_real = mujoco_execute(x, u)       # MuJoCo执行
residual = x_next_real - x_next_nominal

# 训练神经网络
nn_residual = NN(x, u)
x_next_corrected = x_next_nominal + nn_residual
```

**优点**：
- 可以补偿任何系统性偏差
- 数据驱动

**缺点**：
- 需要大量数据
- 泛化性问题
- 梯度计算困难（非解析）

---

**推荐方案优先级**：
1. **方案A（implicit积分器）**：根本解决，值得投入
2. **方案C（子步积分）**：短期可行，作为过渡
3. **方案B（RK4）**：如果方案A太难，备选

---

## 📅 时间估算

| 阶段 | 预计工作量 | 关键里程碑 |
|------|----------|----------|
| Phase 5（轮子动力学） | 3-5天 | 非完整约束MPC收敛 |
| Phase 6（WBC+QP） | 1-2周 | QP求解<1ms，动力学一致 |
| Phase 7（双臂扩展） | 5-7天 | 双臂协调任务演示 |
| Phase 8（动力学匹配） | 1-2周 | 收敛率>80%，误差<2cm |
| **总计** | **4-6周** | 完整系统演示 |

---

## 🎯 里程碑验证标准

### Phase 5完成标志
- [ ] 轮速约束满足（|v_lateral| < 0.01 m/s）
- [ ] MPC收敛率 >20%
- [ ] 闭环稳定30秒

### Phase 6完成标志
- [ ] QP求解时间 <1ms
- [ ] 动力学残差 <1e-3
- [ ] MPC+WBC集成稳定

### Phase 7完成标志
- [ ] 双臂模型加载成功
- [ ] 双臂任务（搬运）精度 <5cm
- [ ] 求解时间可接受

### Phase 8完成标志
- [ ] 动力学预测误差 <1e-3（单步）
- [ ] 累积误差 <0.001（25步）
- [ ] MPC收敛率 >80%
- [ ] EE跟踪误差 <2cm

---

## 📝 文档计划

每个阶段完成后更新：
- `PROGRESS.md`：技术细节、问题记录
- `PHASE_X_SUMMARY.md`：阶段总结
- 测试覆盖：每阶段新增10-15个测试

最终交付：
- 完整技术报告（50-80页）
- 演示视频（5-10分钟）
- 开源代码+文档

---

## 🔧 技术栈

- **动力学**：Pinocchio 3.9+
- **优化**：ALIGATOR 0.19+
- **QP求解**：ProxQP / OSQP
- **仿真**：MuJoCo 3.x
- **测试**：pytest (目标100+测试)

---

## 💡 关键设计决策

1. **先架构后优化**：Phase 5-7先完成功能，Phase 8统一优化性能
2. **模块化设计**：每个阶段独立，可以回退到上一阶段
3. **测试驱动**：每个新功能都有对应单元测试
4. **渐进式复杂度**：从单臂→双臂→完整系统

---

## 🎓 预期论文贡献

1. **系统集成**：完整的移动操作MPC+WBC框架
2. **动力学匹配**：解决数值积分器匹配问题的工程方案
3. **双臂协调**：层次化任务空间控制
4. **开源实现**：可复现的研究平台

---

**更新日期**：2026-06-23  
**当前阶段**：Phase 4完成，准备Phase 5

**下一步行动**：开始Phase 5（轮子真实动力学约束）
