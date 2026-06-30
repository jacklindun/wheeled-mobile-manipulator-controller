# Phase 6: 升级MPC为Kino-Dynamic模型

**日期**: 2024-06-24  
**状态**: 进行中  
**目标**: 让Phase 6的MPC使用混合动力学模型，提供准确的加速度给WBC

---

## 🎯 问题分析

### 当前架构的缺陷

**Phase 6 现状**:
```
MPC (运动学模型):
  q_next = q + dt * u_vel
  输出: u_vel (速度命令)
  ❌ 无法提供准确的加速度信息

WBC (动力学QP):
  从速度差分估计加速度: a_des = (v_des - v_current) / dt
  ❌ 缺少重力、科氏力、惯性等动力学信息
  
  结果: 动力学残差 = 83.25 >> 目标 0.1 (失败)
```

**根本原因**: 运动学模型无法捕捉动力学效应，WBC需要"猜测"大量动力学信息。

---

## ✅ 解决方案

### 使用Phase 4的Kino-Dynamic MPC

**Phase 4 混合动力学**:
```python
状态: x = [q_base(4), q_arm(6), v_arm(6)] = 16-dim
控制: u = [v_base(4), tau_arm(6)] = 10-dim

动力学:
  - 基座: 运动学积分 (速度控制)
  - 机械臂: ABA + semi-implicit Euler (力矩控制)
  
MPC输出:
  - 状态轨迹: xs = [x_0, ..., x_N]
  - 控制轨迹: us = [u_0, ..., u_{N-1}]
  - 加速度: a_arm 可从ABA直接计算 ✅
```

**优势**:
1. MPC直接优化力矩，理解动力学约束
2. WBC从MPC轨迹中提取准确的加速度
3. 动力学一致性自然满足

---

## 🔧 实施步骤

### Step 1: 修复Phase 4 Jacobian精度 ✅ (进行中)

**问题**: `test_hybrid_dynamics.py::test_dforward_jacobians_vs_finite_difference`
- Jacobian误差: 0.0064 vs 阈值 1e-4
- 位置: `v_arm wrt q_arm [10:16, 4:10]` 块

**根因**: 
- `computeABADerivatives()` 在 `crba()` 之后调用，导致数据污染
- Armature correction的导数计算不准确

**修复** (已完成):
```python
# 修改前: CRBA在ABA derivatives之后
pin.computeABADerivatives(...)
pin.crba(...)  # ❌ 污染了data

# 修改后: CRBA在前，立即复制
pin.crba(...)
M = self._arm_data.M.copy()  # ✅ 避免污染
pin.computeABADerivatives(...)
```

**验证**: 
- [ ] 运行测试确认误差 < 1e-4
- [ ] 检查所有相关测试通过

---

### Step 2: 创建Kino-Dynamic MPC接口 (待实施)

**新文件**: `wheeled_ur5e_aligator_mpc/kinodynamic_mpc_controller.py`

```python
class KinoDynamicMPCController:
    """
    基于Phase 4混合动力学的MPC控制器
    
    输入: x_current (16-dim), ref_traj
    输出: 
      - u_opt (10-dim): [v_base(4), tau_arm(6)]
      - trajectory: xs, us, accelerations
    """
    
    def __init__(self, pin_robot, horizon=20, dt=0.05):
        self.problem_builder = HybridWheeledUR5eProblemBuilder(
            robot, pin_robot, horizon, dt
        )
        
    def solve(self, x_current, ref_traj):
        """
        求解kino-dynamic MPC
        
        Returns
        -------
        u_opt : (10,) array
            最优控制 [v_base, tau_arm]
        traj : dict
            {'xs': (N+1, 16), 'us': (N, 10), 'as': (N, 11)}
            包含状态、控制、加速度轨迹
        """
        # 构建ALIGATOR问题
        problem = self.problem_builder.build_problem(
            x_current, ref_traj
        )
        
        # 求解
        solver = aligator.SolverProxDDP(...)
        solver.solve(problem, ...)
        
        # 提取轨迹
        xs = solver.results.xs
        us = solver.results.us
        
        # 计算加速度 (从ABA)
        accelerations = self._compute_accelerations(xs, us)
        
        return us[0], {
            'xs': xs, 
            'us': us, 
            'as': accelerations
        }
    
    def _compute_accelerations(self, xs, us):
        """从状态和控制计算加速度"""
        accelerations = []
        for x, u in zip(xs[:-1], us):
            q_arm = x[4:10]
            v_arm = x[10:16]
            tau_arm = u[4:10]
            
            # 使用ABA计算准确的加速度
            a_arm = pin.aba(arm_model, arm_data, q_arm, v_arm, tau_arm)
            
            # 基座加速度 (从速度差分)
            # 或者从运动学约束推导
            a_base = ...
            
            accelerations.append(
                np.concatenate([a_base, a_arm])
            )
        
        return np.array(accelerations)
```

---

### Step 3: 更新MPC-WBC接口 (待实施)

**修改文件**: `wheeled_ur5e_aligator_mpc/mpc_wbc_interface.py`

```python
class MPCWBCInterface:
    """更新为支持kino-dynamic MPC"""
    
    def __init__(self, kinodynamic_mpc, wbc_dt=0.01):
        self.mpc = kinodynamic_mpc  # 使用kino-dynamic MPC
        self.mpc_dt = kinodynamic_mpc.dt
        self.wbc_dt = wbc_dt
        
        # MPC轨迹缓存 (包含加速度)
        self.mpc_trajectory = None
        
    def get_desired_acceleration_from_mpc(self, x_wbc, t):
        """
        从MPC轨迹直接获取期望加速度
        
        ✅ 不再使用差分估计！
        """
        if self.mpc_trajectory is None:
            return np.zeros(11)
        
        # 找到对应时刻的加速度
        t_mpc = t - self.last_mpc_time
        idx = int(t_mpc / self.mpc_dt)
        
        if idx >= len(self.mpc_trajectory['as']):
            idx = -1
        
        # 直接返回MPC计算的准确加速度
        a_des = self.mpc_trajectory['as'][idx]
        
        return a_des  # ✅ 包含完整动力学信息
```

---

### Step 4: 集成测试 (待实施)

**测试1: Kino-Dynamic MPC单独运行**
```bash
pixi run -e all python scripts/test_kinodynamic_mpc.py
```
验证:
- MPC收敛率 > 80%
- EE跟踪误差 < 3cm
- 求解时间 < 100ms

**测试2: MPC+WBC集成**
```bash
pixi run -e all python scripts/test_mpc_wbc_kinodynamic.py
```
验证:
- 动力学残差 < 0.1 ✅ (关键指标)
- WBC求解时间 < 1ms
- 系统稳定运行30秒

**测试3: 对比Phase 1-3运动学MPC**
```bash
pixi run -e all python scripts/compare_kinematic_vs_kinodynamic.py
```
对比指标:
- EE跟踪精度
- 力矩平滑度
- 动力学一致性
- 计算效率

---

## 📊 预期性能

| 指标 | Phase 6 (当前) | Phase 6 (升级后) | 改进 |
|------|---------------|-----------------|------|
| MPC模型 | 运动学 | Kino-dynamic | ✅ |
| 加速度精度 | 差分估计 | ABA计算 | ✅ |
| 动力学残差 | 83.25 | < 0.1 | **830×** |
| WBC收敛率 | ~0% | > 95% | ✅ |
| MPC求解时间 | ~15ms | ~50-80ms | 3-5× |
| 总控制频率 | 20 Hz | 20 Hz (MPC) + 100 Hz (WBC) | ✅ |

**关键改进**: 动力学残差从83.25降到<0.1，提升**830倍**！

---

## 🔄 状态转换矩阵

### 从16-dim kino-dynamic到23-dim WBC

**MPC状态** (16-dim):
```
x_mpc = [q_base(4), q_arm(6), v_arm(6)]
```

**WBC状态** (23-dim):
```
x_wbc = [q_base(4), θ_wheels(2), q_arm(6), 
         v_base(3), ω_wheels(2), v_arm(6)]
```

**转换逻辑**:
```python
def mpc_to_wbc_state(x_mpc, x_wbc_prev):
    """将MPC状态扩展为WBC状态"""
    x_wbc = np.zeros(23)
    
    # 位置部分
    x_wbc[0:4] = x_mpc[0:4]      # q_base (直接复制)
    x_wbc[4:6] = x_wbc_prev[4:6]  # θ_wheels (保持或从v_base积分)
    x_wbc[6:12] = x_mpc[4:10]    # q_arm (直接复制)
    
    # 速度部分
    v_base_from_mpc = ...  # 从MPC控制u[0:4]获取
    x_wbc[12:15] = v_base_from_mpc  # v_base
    x_wbc[15:17] = ...              # ω_wheels (从v_base计算)
    x_wbc[17:23] = x_mpc[10:16]     # v_arm (直接复制)
    
    return x_wbc
```

---

## 📝 待办清单

- [x] **Step 1a**: 修复Jacobian计算顺序 (CRBA before ABA derivatives)
- [ ] **Step 1b**: 验证Jacobian精度 < 1e-4
- [ ] **Step 2a**: 创建 `KinoDynamicMPCController` 类
- [ ] **Step 2b**: 实现加速度提取 `_compute_accelerations()`
- [ ] **Step 3a**: 更新 `MPCWBCInterface` 支持kino-dynamic
- [ ] **Step 3b**: 修改 `MPCWBCController` 使用新接口
- [ ] **Step 4a**: 单元测试kino-dynamic MPC
- [ ] **Step 4b**: 集成测试MPC+WBC
- [ ] **Step 4c**: 性能对比和基准测试

---

## 🚀 下一步行动

1. **立即**: 验证Jacobian修复是否成功
2. **今天**: 实现 `KinoDynamicMPCController`
3. **明天**: 集成测试MPC+WBC，确认动力学残差<0.1

---

**作者**: Claude & User  
**更新**: 2024-06-24
