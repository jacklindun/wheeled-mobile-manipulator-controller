# Phase 6-v1.1 实验计划

## 实验目标

Phase 4 (Hybrid Dynamics MPC) 失败诊断：

**核心问题**：
- Phase 4失败的根本原因是什么？
  1. MPC动力学预测本身不可靠（模型误差/积分器不匹配）？
  2. 20Hz torque直接执行太粗糙，缺少高频反馈层？

**验证方法**：
- 在Phase 4基础上添加插值 + 前馈PD层
- 对比三种配置的性能

---

## 实验设计

### 测试配置

**A. Phase 4原版（Baseline）**
```
Hybrid MPC (20Hz) → Torque直接执行 (20Hz)
```
- 预期性能: 2.5-5.0 cm RMS（已知失败）

**B. Phase 4 + 插值 + MPC前馈PD**
```
Hybrid MPC (20Hz) → 插值(20Hz→500Hz) → tau_mpc + PD (500Hz)
```
- 使用MPC输出的torque作为前馈
- 当MPC未收敛时，降级使用重力前馈

**C. Phase 4 + 插值 + 重力前馈PD**
```
Hybrid MPC (20Hz) → 插值(20Hz→500Hz) → tau_gravity + PD (500Hz)
```
- 忽略MPC torque输出
- 始终使用重力前馈

---

## 预期结果与结论

### 情况1: B > C > A
- **结论**: MPC torque有正贡献 + 高频执行层有价值
- **意义**: Phase 4部分可救，需要高频执行层

### 情况2: C ≥ B > A
- **结论**: MPC torque反而污染执行，高频执行层有价值
- **意义**: 问题在MPC动力学预测，不在执行频率

### 情况3: A ≈ B ≈ C（都差）
- **结论**: 根本问题是动力学rollout mismatch
- **意义**: Phase 4不可救，归档

### 情况4: C显著好（接近Phase 6-v3水平）
- **结论**: Phase 4的MPC完全没用，简单IK+前馈就够了
- **意义**: 验证了Phase 6-v3的设计哲学

---

## 实现细节

### 插值策略（借鉴Phase 6-v3）
```python
# MPC输出: xs[0:2], us[0]
# 插值到500Hz (25个样本)

for step in range(25):
    alpha = step / 25
    x_interp = (1 - alpha) * xs[0] + alpha * xs[1]
    q_des = x_interp[:nq]
    v_des = x_interp[nq:]
    tau_ff = us[0]  # 保持第一个torque
```

### PD控制器
```python
# 配置B: MPC前馈
if mpc_converged and torque_reasonable(us[0]):
    tau_ff = us[0]
else:
    tau_ff = compute_gravity_torque(q_des)

tau = tau_ff + Kp * (q_des - q) + Kd * (v_des - v)

# 配置C: 重力前馈
tau_ff = compute_gravity_torque(q_des)
tau = tau_ff + Kp * (q_des - q) + Kd * (v_des - v)
```

### MPC收敛判断
```python
def is_mpc_converged(solver_info):
    return solver_info['converged'] and solver_info['iters'] < max_iters

def torque_reasonable(tau, tau_max):
    return np.all(np.abs(tau) < tau_max * 0.95)
```

---

## 时间预算

**总时间**: 1-2天

- Day 1上午: 实现插值+前馈PD层（2-3小时）
- Day 1下午: 运行三组对比实验（2-3小时）
- Day 2: 如果有明显改善，调优参数；否则归档Phase 4

**中止条件**:
- 如果2天内看不到明显改善（<3.5 cm RMS）
- 立即归档Phase 4，不继续调参

---

## 成功标准

**最低目标**: B或C达到 <3.5 cm RMS
**理想目标**: 达到 <2.5 cm RMS

**归档标准**: 2天后仍 >4.0 cm RMS

---

## 文件组织

```
scripts/
├── phase4_original.py           # A: 原版Phase 4
├── phase4_with_mpc_feedforward.py   # B: MPC前馈版
├── phase4_with_gravity_feedforward.py  # C: 重力前馈版
└── compare_phase4_variants.py   # 对比测试脚本

docs/phase4/
└── PHASE4_V1.1_DIAGNOSIS.md     # 实验报告
```

---

## 报告结构

### PHASE4_V1.1_DIAGNOSIS.md

1. **背景**: Phase 4失败原因分析
2. **实验设计**: 三组对比配置
3. **结果**: 性能对比表格
4. **结论**: 
   - Phase 4失败的根本原因
   - 是否值得继续Phase 4
   - 对Phase 6的启示
5. **推荐**: 归档或继续

---

## 下一步

等待确认后开始实现：
1. 创建phase4_with_mpc_feedforward.py
2. 创建phase4_with_gravity_feedforward.py
3. 运行对比实验
4. 生成诊断报告
