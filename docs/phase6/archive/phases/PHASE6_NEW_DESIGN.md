# Phase 6 新设计：Full Dynamic MPC + 插值前馈PD控制

**日期**: 2026-06-25  
**状态**: 设计阶段  
**目标**: 解决Phase 4积分器不匹配问题

---

## 🎯 设计目标

### 问题回顾

**Phase 4的失败**:
```
问题: ALIGATOR (semi-implicit Euler) ≠ MuJoCo (implicitfast)
结果: 
  - 单步误差: 7e-5 (可接受)
  - 25步累积: 0.034 (放大480倍)
  - 收敛率: 0%
  - 直接输出力矩 → 预测与执行严重偏差
```

**新方案核心思想**:
> 不直接输出MPC力矩，而是通过**插值+前馈PD控制**消除积分器不匹配

---

## 🏗️ 新架构设计

### 整体控制流程

```
┌─────────────────────────────────────────────┐
│  Full Dynamic MPC (0.05s = 50ms)            │
├─────────────────────────────────────────────┤
│  输入: x_current (23-dim)                    │
│  模型: Phase 5 完整动力学                     │
│       - 基座虚拟关节(4)                       │
│       - 物理轮子(2) + 机械臂(6)               │
│  控制: u = [τ_wheels(2), τ_arm(6)]          │
│  输出: 轨迹 {xs, us, ts} for t∈[0, 1s]      │
└─────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────┐
│  插值器 (Interpolator)                       │
├─────────────────────────────────────────────┤
│  MPC输出: 20个时间步 × 0.05s = 1s轨迹        │
│  MuJoCo需要: 0.002s步长                      │
│  方法: 线性/样条插值                          │
│  输出: x_des(t), u_feedforward(t)           │
└─────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────┐
│  前馈PD控制器 (0.002s = 2ms)                 │
├─────────────────────────────────────────────┤
│  τ_final = τ_mpc                            │
│          + Kp * (q_des - q_current)         │
│          + Kd * (v_des - v_current)         │
│                                              │
│  轮子: Kp=500, Kd=50                         │
│  机械臂: Kp=1000, Kd=100                     │
└─────────────────────────────────────────────┘
                    ↓
                MuJoCo
```

---

## 📊 状态空间定义

### Phase 5的完整状态 (23-dim)

```python
x = [q(12), v(11)]

q = [base_x, base_y, base_z, base_yaw,      # 虚拟基座 (4)
     θ_left_wheel, θ_right_wheel,            # 轮子角度 (2)
     q_arm(6)]                               # 机械臂 (6)

v = [vx_base, vy_base, vz_base,             # 基座线速度 (3)
     ω_left, ω_right,                        # 轮速 (2)
     v_arm(6)]                               # 机械臂关节速度 (6)
```

### 控制输入 (8-dim)

```python
u = [τ_left, τ_right,    # 轮子电机扭矩 (2)
     τ_arm(6)]            # 机械臂关节扭矩 (6)
```

---

## 🔧 关键技术实现

### 1. Full Dynamic MPC (复用Phase 5动力学)

```python
class FullDynamicMPCController:
    """
    基于Phase 5完整动力学的MPC
    
    与Phase 4的区别:
    - Phase 4: 基座速度控制 + 机械臂扭矩控制 (混合)
    - Phase 6: 全部扭矩控制 (纯动力学)
    """
    
    def __init__(self, horizon=20, dt=0.05):
        from wheeled_ur5e_aligator_mpc.wheeled_dynamics import WheeledUR5eDynamics
        
        # Phase 5的完整动力学
        self.dynamics = WheeledUR5eDynamics()
        
        # 状态: 23-dim, 控制: 8-dim
        self.nx = 23
        self.nu = 8
        
        self.horizon = horizon
        self.dt = dt  # 0.05s, MPC求解周期
        
    def solve(self, x_current, ref_traj):
        """
        求解MPC优化问题
        
        Parameters
        ----------
        x_current : (23,) array
            当前状态
        ref_traj : dict
            参考轨迹 (基座位置、EE位置等)
        
        Returns
        -------
        trajectory : dict
            {
                'xs': (N+1, 23) 状态轨迹
                'us': (N, 8) 控制轨迹
                'ts': (N+1,) 时间点
            }
        """
        # 构建ALIGATOR OCP
        problem = self._build_problem(x_current, ref_traj)
        
        # 求解
        solver = aligator.SolverProxDDP(problem)
        solver.max_iters = 50
        solver.solve()
        
        # 提取轨迹
        xs = np.array(solver.results.xs)
        us = np.array(solver.results.us)
        ts = np.arange(len(xs)) * self.dt
        
        return {'xs': xs, 'us': us, 'ts': ts}
```

### 2. 轨迹插值器

```python
class TrajectoryInterpolator:
    """
    将MPC轨迹插值到MuJoCo控制频率
    
    MPC: 0.05s步长 (20 Hz)
    MuJoCo: 0.002s步长 (500 Hz)
    插值比例: 25:1
    """
    
    def __init__(self, mpc_dt=0.05, sim_dt=0.002):
        self.mpc_dt = mpc_dt
        self.sim_dt = sim_dt
        self.ratio = int(mpc_dt / sim_dt)  # 25
        
        # 当前MPC轨迹缓存
        self.trajectory = None
        self.trajectory_start_time = 0
        
    def update_trajectory(self, trajectory, current_time):
        """更新MPC轨迹"""
        self.trajectory = trajectory
        self.trajectory_start_time = current_time
        
    def interpolate(self, current_time):
        """
        插值获取当前时刻的期望状态和前馈控制
        
        Parameters
        ----------
        current_time : float
            当前仿真时间
        
        Returns
        -------
        x_des : (23,) array
            期望状态
        u_feedforward : (8,) array
            前馈扭矩
        """
        if self.trajectory is None:
            return None, None
        
        # 计算相对时间
        t_rel = current_time - self.trajectory_start_time
        
        # 如果超出轨迹范围，返回最后一个点
        if t_rel >= self.trajectory['ts'][-1]:
            return self.trajectory['xs'][-1], self.trajectory['us'][-1]
        
        # 线性插值
        xs = self.trajectory['xs']
        us = self.trajectory['us']
        ts = self.trajectory['ts']
        
        # 找到插值区间
        idx = np.searchsorted(ts, t_rel) - 1
        idx = max(0, min(idx, len(ts) - 2))
        
        # 插值权重
        t0, t1 = ts[idx], ts[idx + 1]
        alpha = (t_rel - t0) / (t1 - t0)
        
        # 状态插值
        x_des = (1 - alpha) * xs[idx] + alpha * xs[idx + 1]
        
        # 控制插值
        u_feedforward = (1 - alpha) * us[idx] + alpha * us[idx + 1]
        
        return x_des, u_feedforward
```

### 3. 前馈PD控制器

```python
class FeedforwardPDController:
    """
    前馈PD控制器
    
    核心思想:
    - 前馈项: MPC输出的扭矩 (基于动力学模型)
    - 反馈项: PD控制器补偿模型误差和扰动
    """
    
    def __init__(self, Kp_wheels=500.0, Kd_wheels=50.0,
                       Kp_arm=1000.0, Kd_arm=100.0):
        """
        Parameters
        ----------
        Kp_wheels : float
            轮子比例增益
        Kd_wheels : float
            轮子微分增益
        Kp_arm : float
            机械臂比例增益
        Kd_arm : float
            机械臂微分增益
        """
        self.Kp_wheels = Kp_wheels
        self.Kd_wheels = Kd_wheels
        self.Kp_arm = Kp_arm
        self.Kd_arm = Kd_arm
        
    def compute_control(self, x_current, x_des, u_feedforward):
        """
        计算最终控制输出
        
        Parameters
        ----------
        x_current : (23,) array
            当前状态 [q(12), v(11)]
        x_des : (23,) array
            期望状态
        u_feedforward : (8,) array
            MPC前馈扭矩
        
        Returns
        -------
        u_final : (8,) array
            最终扭矩 = 前馈 + PD反馈
        """
        # 提取状态
        q_current = x_current[:12]
        v_current = x_current[12:]
        
        q_des = x_des[:12]
        v_des = x_des[12:]
        
        # 位置误差
        q_error = q_des - q_current
        
        # 速度误差
        v_error = v_des - v_current
        
        # PD反馈扭矩
        tau_pd = np.zeros(8)
        
        # 轮子 (索引4:6对应的速度是9:11)
        tau_pd[0:2] = (self.Kp_wheels * q_error[4:6] + 
                       self.Kd_wheels * v_error[3:5])
        
        # 机械臂 (索引6:12对应的速度是5:11)
        tau_pd[2:8] = (self.Kp_arm * q_error[6:12] + 
                       self.Kd_arm * v_error[5:11])
        
        # 最终扭矩 = 前馈 + 反馈
        u_final = u_feedforward + tau_pd
        
        return u_final
```

### 4. 完整集成

```python
class Phase6Controller:
    """
    Phase 6完整控制器: Full Dynamic MPC + 插值 + 前馈PD
    """
    
    def __init__(self):
        # MPC: 20 Hz (0.05s)
        self.mpc = FullDynamicMPCController(horizon=20, dt=0.05)
        
        # 插值器
        self.interpolator = TrajectoryInterpolator(mpc_dt=0.05, sim_dt=0.002)
        
        # 前馈PD控制器
        self.pd_controller = FeedforwardPDController(
            Kp_wheels=500.0, Kd_wheels=50.0,
            Kp_arm=1000.0, Kd_arm=100.0
        )
        
        # 上次MPC更新时间
        self.last_mpc_time = 0
        self.mpc_dt = 0.05
        
    def control_step(self, x_current, ref_traj, current_time):
        """
        单步控制
        
        Parameters
        ----------
        x_current : (23,) array
            当前状态
        ref_traj : dict
            参考轨迹
        current_time : float
            当前时间
        
        Returns
        -------
        u_final : (8,) array
            最终控制扭矩
        """
        # 1. 检查是否需要更新MPC
        if current_time - self.last_mpc_time >= self.mpc_dt:
            # 求解MPC
            trajectory = self.mpc.solve(x_current, ref_traj)
            
            # 更新插值器
            self.interpolator.update_trajectory(trajectory, current_time)
            
            self.last_mpc_time = current_time
        
        # 2. 插值获取当前时刻的期望
        x_des, u_feedforward = self.interpolator.interpolate(current_time)
        
        if x_des is None:
            # 第一步，MPC还未求解
            return np.zeros(8)
        
        # 3. 前馈PD控制
        u_final = self.pd_controller.compute_control(
            x_current, x_des, u_feedforward
        )
        
        return u_final
```

---

## 🎯 关键优势

### 1. 解决积分器不匹配

**Phase 4的问题**:
```
MPC预测25步 (1.25s) → 累积误差0.034 → 预测完全错误
```

**Phase 6的解决**:
```
MPC预测1步 (0.05s) → 立即更新 → 累积误差最多单步7e-5
插值保证平滑 (25个子步)
PD反馈消除残差
```

### 2. 前馈+反馈结合

**前馈项 (u_feedforward)**:
- 来自MPC的动力学模型
- 包含重力补偿、科氏力等
- 提供主要控制力

**反馈项 (PD)**:
- 补偿模型误差
- 消除扰动
- 保证鲁棒性

### 3. 频率分离

**MPC层**: 20 Hz
- 计算密集 (~50-100ms求解时间)
- 全局轨迹优化
- 处理约束和代价

**控制层**: 500 Hz
- 轻量级 (<1ms)
- 实时反馈
- 平滑控制

---

## 📊 预期性能

### 对比Phase 4

| 指标 | Phase 4 (直接输出) | Phase 6 (插值+PD) |
|------|-------------------|------------------|
| 积分器匹配 | ❌ 不匹配 | ✅ 通过插值+反馈补偿 |
| 收敛率 | 0% | 预期 >60% |
| EE误差 | 2.5-5.0 cm | 预期 <3 cm |
| 求解时间 | 75 ms | 预期 50-100 ms |
| 控制平滑度 | ⚠️ 跳变 | ✅ 插值保证平滑 |

### 对比Phase 1-3

| 指标 | Phase 1-3 (运动学) | Phase 6 (动力学) |
|------|-------------------|-----------------|
| 模型 | 运动学近似 | 完整动力学 |
| EE误差 | 1.5-2.1 cm | 预期 2-3 cm |
| 控制输出 | 速度命令 | 力矩命令 |
| 真实硬件适用性 | 低 (需要底层控制器) | 高 (直接力矩控制) |

---

## 🔧 实施步骤

### Step 1: 实现Full Dynamic MPC (2-3天)

**任务**:
1. 创建 `full_dynamic_mpc_controller.py`
2. 使用Phase 5的 `WheeledUR5eDynamics`
3. 构建ALIGATOR OCP (代价函数 + 约束)
4. 单元测试: 验证MPC能求解

**验证**:
```bash
python -m pytest tests/test_full_dynamic_mpc.py -v
```

### Step 2: 实现插值器 (1天)

**任务**:
1. 创建 `trajectory_interpolator.py`
2. 实现线性插值 (可选: 样条插值)
3. 处理边界情况 (轨迹结束、MPC未更新)
4. 单元测试: 验证插值精度

**验证**:
```python
# 测试插值精度
trajectory = {'xs': [...], 'us': [...], 'ts': [...]}
interpolator.update_trajectory(trajectory, t=0)

# 在中间时刻插值
x_des, u_ff = interpolator.interpolate(t=0.025)  # 两个MPC步之间

# 验证: x_des应该接近线性插值结果
```

### Step 3: 实现前馈PD控制器 (1天)

**任务**:
1. 创建 `feedforward_pd_controller.py`
2. 实现前馈+PD逻辑
3. 调参: 初始Kp, Kd值
4. 单元测试: 验证控制输出合理

**验证**:
```python
# 测试无误差情况 (前馈主导)
u = pd_controller.compute_control(x_current, x_des=x_current, u_ff)
assert np.allclose(u, u_ff)

# 测试有误差情况 (反馈补偿)
x_des_offset = x_current.copy()
x_des_offset[0] += 0.1  # 10cm位置误差
u = pd_controller.compute_control(x_current, x_des_offset, u_ff)
assert not np.allclose(u, u_ff)  # 应该有额外的反馈项
```

### Step 4: 集成测试 (1-2天)

**任务**:
1. 创建 `phase6_controller.py` (集成)
2. 创建 `demo_phase6.py` (演示脚本)
3. 在MuJoCo中闭环测试
4. 调优增益参数

**验证场景**:
```bash
# 场景1: 静止目标 (5秒)
python scripts/demo_phase6.py --scenario stationary --duration 5

# 场景2: EE画圆 (10秒)
python scripts/demo_phase6.py --scenario ee_circle --duration 10

# 场景3: 基座移动 (15秒)
python scripts/demo_phase6.py --scenario base_and_ee --duration 15
```

### Step 5: 性能评估和调优 (1-2天)

**任务**:
1. 记录关键指标 (EE误差、收敛率、求解时间)
2. 调优MPC权重
3. 调优PD增益
4. 对比Phase 1-3和Phase 4

**关键指标**:
- EE RMS误差 < 3 cm
- MPC收敛率 > 60%
- 系统稳定运行 ≥ 30秒
- 控制平滑 (无明显跳变)

---

## 🎛️ 参数调优指南

### MPC权重

```python
# 代价函数权重
weights = {
    'ee_pos': 1000.0,      # EE位置跟踪 (主要目标)
    'ee_ori': 0.0,         # EE姿态 (可选)
    'base_pos': 10.0,      # 基座位置
    'base_yaw': 10.0,      # 基座朝向
    'posture': 1.0,        # 姿态正则化
    'control': 0.01,       # 扭矩正则化
    'control_smooth': 1.0, # 扭矩平滑
}
```

### PD增益

**初始值** (基于经验):
```python
# 轮子 (质量小，惯性小)
Kp_wheels = 500.0
Kd_wheels = 50.0

# 机械臂 (质量大，惯性大)
Kp_arm = 1000.0
Kd_arm = 100.0
```

**调优策略**:
1. Kp太小 → 跟踪慢，误差大
2. Kp太大 → 震荡，不稳定
3. Kd太小 → 超调
4. Kd太大 → 噪声敏感

**推荐调优顺序**:
1. 先调Kp (固定Kd=0)，找到临界增益
2. 设置 Kp = 0.6 * Kp_critical
3. 再调Kd，消除震荡

---

## 🧪 测试计划

### 单元测试 (15个)

```
tests/
  ├─ test_full_dynamic_mpc.py        # MPC模块 (5个测试)
  │  ├─ test_mpc_initialization
  │  ├─ test_mpc_solve_stationary
  │  ├─ test_mpc_trajectory_shape
  │  ├─ test_mpc_cost_decrease
  │  └─ test_mpc_constraints_satisfied
  │
  ├─ test_interpolator.py             # 插值器 (5个测试)
  │  ├─ test_linear_interpolation
  │  ├─ test_boundary_extrapolation
  │  ├─ test_update_trajectory
  │  ├─ test_time_out_of_range
  │  └─ test_interpolation_accuracy
  │
  └─ test_feedforward_pd.py           # PD控制器 (5个测试)
     ├─ test_zero_error_feedforward_only
     ├─ test_position_error_feedback
     ├─ test_velocity_error_feedback
     ├─ test_control_saturation
     └─ test_different_gains
```

### 集成测试 (3个场景)

```
scripts/
  └─ demo_phase6.py
     ├─ stationary       # 静止目标
     ├─ ee_circle        # EE画圆
     └─ base_and_ee      # 基座移动
```

---

## 📈 成功标准

### 必须满足 (Minimum Viable Product)
- [x] MPC能正常求解 (不崩溃)
- [x] 控制输出平滑 (插值有效)
- [x] 系统稳定运行 ≥ 10秒
- [x] EE误差 < 5 cm

### 期望达到 (Target Performance)
- [ ] MPC收敛率 > 60%
- [ ] EE RMS误差 < 3 cm
- [ ] 求解时间 < 100 ms
- [ ] 系统稳定运行 ≥ 30秒

### 理想目标 (Stretch Goals)
- [ ] EE误差接近Phase 1-3 (2 cm)
- [ ] 收敛率 > 80%
- [ ] 真实硬件部署准备

---

## 🔍 与旧Phase 6的关系

### 旧Phase 6 (MPC+WBC)

**保留但不推荐**:
- 代码位置: `mpc_wbc_*.py`, `wbc_controller.py`
- 状态: 架构完整但未验证
- 问题: 欠驱动系统的动力学残差大
- 用途: 作为技术探索的参考

### 新Phase 6 (Full Dynamic MPC + 插值PD)

**主推方案**:
- 代码位置: `full_dynamic_mpc_controller.py`, `phase6_controller.py`
- 优势: 
  - 直接解决积分器不匹配
  - 无需WBC的复杂QP
  - 前馈+反馈结合
- 风险: 
  - 仍然基于Phase 5动力学 (可能有误差)
  - PD增益需要仔细调优

---

## 💡 技术创新点

1. **插值消除频率不匹配**
   - MPC慢频率 (20 Hz) 规划
   - 控制快频率 (500 Hz) 执行
   - 插值桥接两者

2. **前馈+反馈分工**
   - 前馈: 动力学补偿 (主要)
   - 反馈: 误差纠正 (辅助)

3. **避免WBC复杂度**
   - 不需要实时QP求解
   - PD控制简单高效
   - 易于调参和理解

---

## 📅 时间规划

**总计**: 5-7天

| 阶段 | 任务 | 时间 |
|------|------|------|
| Step 1 | Full Dynamic MPC | 2-3天 |
| Step 2 | 插值器 | 1天 |
| Step 3 | 前馈PD | 1天 |
| Step 4 | 集成测试 | 1-2天 |
| Step 5 | 调优评估 | 1-2天 |

**里程碑**:
- Day 3: MPC能求解
- Day 5: 闭环运行5秒
- Day 7: 性能达标或明确失败原因

---

## 🚀 下一步行动

你希望我：
1. **立即开始实现Step 1** (Full Dynamic MPC)？
2. 先创建详细的API设计文档？
3. 还是先运行一些Phase 5的测试验证动力学模型？

---

**设计完成日期**: 2026-06-25  
**设计者**: Claude + User  
**预期开始时间**: 待确认
