# Phase 6: MPC+WBC 双层架构

**目标**：分离规划与控制，MPC负责轨迹优化，WBC负责力矩求解

**状态**：设计中  
**预计工作量**：1-2周  
**前置条件**：Phase 1-5完成

---

## 🎯 核心理念

### 为什么需要双层架构？

**Phase 4的问题**：
- 混合MPC直接输出扭矩
- 需要精确的动力学模型
- 积分器不匹配导致收敛率0%

**Phase 6的解决方案**：
- **解耦规划与控制**
- MPC只需近似动力学（运动学即可）
- WBC保证动力学一致性

---

## 🏗️ 系统架构

```
┌─────────────────────────────────────────────────────┐
│  上层：MPC - 轨迹规划 (10-20 Hz)                      │
├─────────────────────────────────────────────────────┤
│  输入: x_current, ref_trajectory                     │
│  优化: 运动学MPC（复用Phase 1-3）                      │
│  输出: x_des[k], v_des[k], a_des[k]                  │
│        期望位置、速度、加速度                           │
└─────────────────────────────────────────────────────┘
                        ↓
              MPC → WBC 接口
                        ↓
┌─────────────────────────────────────────────────────┐
│  下层：WBC - 力矩求解 (100-500 Hz)                    │
├─────────────────────────────────────────────────────┤
│  输入: x_current, x_des, v_des, a_des                │
│                                                       │
│  QP问题:                                              │
│    minimize   ||M(q)a + h(q,v) - S^T τ||²           │
│               + w_track * ||a - a_des||²             │
│               + w_reg * ||τ||²                       │
│               + w_smooth * ||τ - τ_prev||²           │
│                                                       │
│    subject to τ_min ≤ τ ≤ τ_max                      │
│               vy_body = 0 (非完整约束)                 │
│               接触约束（可选）                          │
│                                                       │
│  输出: τ_opt = [τ_wheels(2), τ_arm(6)]               │
└─────────────────────────────────────────────────────┘
                        ↓
                  机器人/仿真
```

---

## 📐 数学公式

### MPC层（运动学）

**状态空间**（Phase 1-3）：
```
q = [base_x, base_y, base_z, base_yaw, arm(6)]  # 10-dim
u = [vx_body, vy_body, vz, ω_yaw, v_arm(6)]     # 10-dim
```

**动力学**（简化运动学）：
```
q_next = q + dt * u
```

**代价函数**：
```
cost = Σ_k (
    w_ee * ||p_ee(q_k) - p_ee_ref||²
    + w_posture * ||q_k - q_nom||²
    + w_vel * ||u_k||²
)
```

**约束**：
```
q_min ≤ q_k ≤ q_max
u_min ≤ u_k ≤ u_max
vy_body = 0 (Phase 5非完整约束)
```

### WBC层（动力学）

**状态空间**（完整动力学）：
```
x = [q(12), v(11)] = 23-dim  # Phase 5定义
q = [base(4), θ_wheels(2), arm(6)]
v = [v_base(3), ω_wheels(2), v_arm(6)]
```

**QP决策变量**：
```
z = [a, τ]
  a: 广义加速度 (11-dim) = [a_base(3), α_wheels(2), a_arm(6)]
  τ: 控制扭矩 (8-dim) = [τ_wheels(2), τ_arm(6)]
```

**动力学方程**（欧拉-拉格朗日）：
```
M(q) * a + h(q, v) = S^T * τ

其中:
  M(q): 质量矩阵 (11×11)
  h(q,v): 科氏力 + 重力 + 阻尼
  S: 选择矩阵 (11×8)
```

**QP目标函数**：
```
minimize:
  w_dyn * ||M*a + h - S^T*τ||²       # 动力学一致性
  + w_track * ||a - a_des||²          # 跟踪期望加速度
  + w_reg * ||τ||²                    # 扭矩正则化
  + w_smooth * ||τ - τ_prev||²        # 扭矩平滑

subject to:
  τ_min ≤ τ ≤ τ_max                   # 扭矩限制
  vy_body(q, v) = 0                   # 非完整约束
  v_min ≤ v + dt*a ≤ v_max            # 速度限制（可选）
```

**标准QP形式**：
```
minimize   0.5 * z^T * P * z + q^T * z
subject to A_eq * z = b_eq     (等式约束)
           A_ineq * z ≤ b_ineq (不等式约束)
```

---

## 🔧 实现步骤

### Step 1: WBC核心类（2-3天）

创建 `wbc_controller.py`：

```python
class WholeBodyController:
    """
    全身控制器（WBC）- QP求解器

    输入：当前状态 x, 期望加速度 a_des
    输出：最优扭矩 τ_opt

    使用ProxQP求解器
    """

    def __init__(self, robot_model, wheel_params):
        self.robot = robot_model
        self.wheel = wheel_params

        # QP求解器
        self.qp_solver = None  # ProxQP

        # 权重
        self.w_dynamics = 1000.0   # 动力学一致性（高权重！）
        self.w_tracking = 100.0    # 跟踪
        self.w_reg = 0.01          # 正则化
        self.w_smooth = 1.0        # 平滑

    def compute_control(self, x_current, x_des, v_des, a_des, τ_prev):
        """
        计算最优扭矩

        Parameters
        ----------
        x_current : (23,) array
            当前状态
        x_des : (12,) array
            期望位置
        v_des : (11,) array
            期望速度
        a_des : (11,) array
            期望加速度
        τ_prev : (8,) array
            上一步扭矩

        Returns
        -------
        τ_opt : (8,) array
            最优扭矩
        """
        # 1. 提取当前状态
        q = x_current[:12]
        v = x_current[12:]

        # 2. 计算动力学项
        M, h = self._compute_dynamics(q, v)

        # 3. 构建QP问题
        P, q_vec, A_eq, b_eq, A_ineq, b_ineq = self._build_qp(
            M, h, a_des, τ_prev, v
        )

        # 4. 求解QP
        z_opt = self._solve_qp(P, q_vec, A_eq, b_eq, A_ineq, b_ineq)

        # 5. 提取扭矩
        a_opt = z_opt[:11]
        τ_opt = z_opt[11:]

        return τ_opt, a_opt

    def _compute_dynamics(self, q, v):
        """计算M(q)和h(q,v)"""
        # 使用Pinocchio计算
        # M: CRBA
        # h: RNEA(q, v, 0) - gravity + coriolis + friction
        pass

    def _build_qp(self, M, h, a_des, τ_prev, v):
        """构建QP矩阵"""
        # 决策变量: z = [a(11), τ(8)] = 19-dim

        # 目标函数
        # P = diag([w_track*I, w_reg*I]) + w_dyn*...
        # q = -w_track*a_des - w_smooth*τ_prev
        pass

    def _solve_qp(self, P, q, A_eq, b_eq, A_ineq, b_ineq):
        """求解QP（使用ProxQP）"""
        pass
```

**关键点**：
- 动力学项使用Pinocchio计算
- QP矩阵构建需要仔细推导
- ProxQP接口需要正定矩阵P

---

### Step 2: MPC-WBC接口（1天）

创建 `mpc_wbc_interface.py`：

```python
class MPCWBCInterface:
    """
    MPC和WBC之间的数据接口

    负责：
    - 状态空间转换（10-dim MPC → 23-dim WBC）
    - 加速度估计（从MPC速度轨迹）
    - 频率同步（MPC 20Hz → WBC 100Hz）
    """

    def __init__(self, mpc_controller, wbc_controller):
        self.mpc = mpc_controller
        self.wbc = wbc_controller

        # MPC输出缓存
        self.mpc_trajectory = None  # xs, us from MPC
        self.last_update_time = 0

    def compute_mpc_trajectory(self, x_current, ref_traj, t):
        """
        调用MPC获取轨迹

        只在需要时更新（10-20 Hz）
        """
        if t - self.last_update_time >= self.mpc_dt:
            # 转换状态：23-dim → 10-dim
            x_mpc = self._wbc_state_to_mpc(x_current)

            # 运行MPC
            self.mpc_trajectory = self.mpc.solve(x_mpc, ref_traj)

            self.last_update_time = t

    def get_desired_acceleration(self, x_current, t):
        """
        从MPC轨迹插值/估计期望加速度
        """
        # 从 xs, us 计算 a_des
        # a_des = (u[k+1] - u[k]) / dt
        pass

    def _wbc_state_to_mpc(self, x_wbc):
        """23-dim WBC状态 → 10-dim MPC状态"""
        # x_wbc = [q_base(4), θ_wheels(2), q_arm(6), v_base(3), ω_wheels(2), v_arm(6)]
        # x_mpc = [q_base(4), q_arm(6)]

        q_mpc = np.concatenate([x_wbc[:4], x_wbc[6:12]])
        return q_mpc

    def _mpc_state_to_wbc(self, x_mpc, x_wbc_prev):
        """10-dim MPC状态 → 23-dim WBC状态"""
        # 保留轮子状态，更新base和arm
        pass
```

---

### Step 3: 闭环集成（2-3天）

创建 `mpc_wbc_controller.py`：

```python
class MPCWBCController:
    """
    MPC+WBC完整控制器

    双层控制循环：
    - MPC: 10-20 Hz
    - WBC: 100-500 Hz
    """

    def __init__(self, mpc, wbc, interface):
        self.mpc = mpc
        self.wbc = wbc
        self.interface = interface

        self.mpc_freq = 20  # Hz
        self.wbc_freq = 100  # Hz

    def control_loop(self, env, ref_traj, duration):
        """
        运行闭环控制

        Parameters
        ----------
        env : MujocoEnv
            仿真环境
        ref_traj : dict
            参考轨迹
        duration : float
            控制时长
        """
        dt_wbc = 1.0 / self.wbc_freq
        num_steps = int(duration / dt_wbc)

        τ_prev = np.zeros(8)

        for step in range(num_steps):
            t = step * dt_wbc

            # 1. 获取当前状态（23-dim WBC）
            x_current = env.get_state()

            # 2. 更新MPC轨迹（如果需要）
            self.interface.compute_mpc_trajectory(x_current, ref_traj, t)

            # 3. 获取期望加速度
            x_des, v_des, a_des = self.interface.get_desired_acceleration(
                x_current, t
            )

            # 4. WBC求解扭矩
            τ_opt, a_opt = self.wbc.compute_control(
                x_current, x_des, v_des, a_des, τ_prev
            )

            # 5. 应用控制
            env.set_control(τ_opt)
            env.step(substeps=int(dt_wbc / env.model.opt.timestep))

            # 6. 记录
            τ_prev = τ_opt

            # 7. 输出（每秒一次）
            if step % self.wbc_freq == 0:
                ee_pos = env.get_ee_pos()
                ee_ref = ref_traj["ee_pos"][int(t / 0.05)]
                ee_err = np.linalg.norm(ee_pos - ee_ref)
                print(f"t={t:5.2f}s: EE_err={ee_err*100:5.2f}cm")
```

---

### Step 4: 测试和验证（2-3天）

**测试1：WBC单独测试**
```python
def test_wbc_tracking():
    """测试WBC跟踪给定的加速度"""
    # 给定a_des，验证τ_opt能实现
```

**测试2：MPC-WBC集成**
```python
def test_mpc_wbc_integration():
    """测试MPC+WBC闭环"""
    # EE画圆任务
    # 验证收敛率、跟踪误差
```

**测试3：性能基准**
```python
def test_performance():
    """对比Phase 1-3 vs Phase 6"""
    # 收敛率、误差、求解时间
```

---

## 📊 预期性能

| 指标 | Phase 1-3（运动学MPC） | Phase 6（MPC+WBC） |
|------|----------------------|-------------------|
| 收敛率 | 100% | >80% (目标) |
| EE RMS误差 | 1.5-2.1 cm | <2.0 cm (目标) |
| MPC求解时间 | 15 ms | 15 ms (复用) |
| WBC求解时间 | N/A | <1 ms (目标) |
| 总控制频率 | 20 Hz | 100 Hz |

---

## 🎯 验证标准

Phase 6完成的标志：

- [ ] WBC QP求解时间 <1 ms
- [ ] 动力学残差 ||M*a + h - S^T*τ|| <1e-3
- [ ] MPC收敛率 >80%
- [ ] EE跟踪误差 <2 cm (RMS)
- [ ] 闭环稳定运行30秒
- [ ] 力矩平滑（无突变）
- [ ] 非完整约束满足 |vy_body| <0.01 m/s

---

## 🔗 技术参考

### ProxQP使用
```python
import proxsuite

# 创建QP问题
qp = proxsuite.proxqp.dense.QP(n, n_eq, n_in)

# 初始化
qp.init(P, q, A_eq, b_eq, A_ineq, b_ineq)

# 求解
qp.solve()

# 获取结果
z_opt = qp.results.x
```

### Pinocchio动力学
```python
import pinocchio as pin

# 质量矩阵
pin.crba(model, data, q)
M = data.M

# 非线性项
pin.rnea(model, data, q, v, np.zeros(nv))
h = data.tau  # 包含重力、科氏力等
```

---

## 📝 下一步行动

准备好开始Phase 6了吗？我可以帮你：

1. **创建WBC控制器框架**
2. **实现QP问题构建**
3. **设置ProxQP求解器**
4. **创建测试用例**

你想从哪里开始？🚀
