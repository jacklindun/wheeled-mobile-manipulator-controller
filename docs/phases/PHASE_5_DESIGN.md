# Phase 5: 轮子真实动力学约束

**目标**：从虚拟基座运动学 → 差速驱动移动底盘动力学

**状态**：设计中  
**预计工作量**：3-5天  
**前置条件**：Phase 1-4完成

---

## 🎯 技术目标

从当前的10-DOF虚拟基座系统：
```
q = [base_x, base_y, base_z, base_yaw, arm(6)]
u = [vx_body, vy_body, vz, ω_yaw, v_arm(6)]
```

升级到包含轮子动力学的系统：
```
q = [base_x, base_y, base_z, base_yaw, θ_left, θ_right, arm(6)]  # 12-DOF
u = [τ_left, τ_right, v_arm(6)]  # 8-dim控制
```

---

## 🏗️ 系统建模

### 1. 差速驱动运动学

**基本参数**：
```python
wheelbase = 0.5  # m, 两轮间距
wheel_radius = 0.1  # m, 轮子半径
```

**轮速 → 基座速度映射**：
```python
# 前向运动学
v_linear = r * (ω_right + ω_left) / 2        # 线速度
ω_angular = r * (ω_right - ω_left) / wheelbase  # 角速度

# 在body frame
vx_body = v_linear
vy_body = 0  # 非完整约束！
ω_yaw = ω_angular
```

**逆向运动学**：
```python
# 给定期望的 (v_linear, ω_angular)
ω_left = (v_linear - wheelbase/2 * ω_angular) / r
ω_right = (v_linear + wheelbase/2 * ω_angular) / r
```

### 2. 轮子动力学

**单个轮子的动态方程**：
```
I_wheel * dω/dt = τ_motor - τ_friction - F_ground * r

其中:
  I_wheel: 轮子转动惯量 ≈ 0.01 kg·m²
  τ_motor: 电机扭矩（控制输入）
  τ_friction: 摩擦阻力 = b * ω
  F_ground: 地面反力（来自基座动力学）
```

### 3. 基座动力学

**平移动力学**：
```
m_base * a_base = F_left + F_right + F_external

其中:
  m_base: 基座质量 ≈ 50 kg (包括机械臂)
  F_left, F_right: 左右轮的地面推力
  F_external: 外力（如机械臂反作用力）
```

**旋转动力学**：
```
I_base * α_yaw = (F_right - F_left) * wheelbase/2 + τ_external
```

### 4. 非完整约束

**关键约束**：轮子不能侧向滑动
```
vy_body = 0  # 在body frame中，y方向速度必须为0

或者更精确地:
vx_body = v_linear
vy_body = 0
ω_yaw = ω_angular
```

**在ALIGATOR中实现**：
```python
# 方法1: 作为等式约束
constraint = aligator.constraints.EqualityConstraint(
    residual=vy_body_residual,  # vy_body(q, v) = 0
    space=manifold
)

# 方法2: 作为Baumgarte稳定化
# vy_body + k_d * vy_body_dot = 0
```

---

## 🔧 实现步骤

### Step 1: 更新MJCF模型（0.5天）

创建 `wheeled_ur5e_with_wheels.xml`：

```xml
<mujoco>
  <worldbody>
    <!-- 基座（保持虚拟关节用于定位） -->
    <body name="base_x_body">
      <joint name="base_x" type="slide" axis="1 0 0"/>
      <body name="base_y_body">
        <joint name="base_y" type="slide" axis="0 1 0"/>
        <body name="base_yaw_body">
          <joint name="base_yaw" type="hinge" axis="0 0 1"/>
          <body name="base_z_body">
            <joint name="base_z" type="slide" axis="0 0 1"/>
            
            <!-- 底盘 -->
            <body name="chassis">
              <geom type="box" size="0.35 0.25 0.05" mass="30"/>
              
              <!-- 左轮 -->
              <body name="left_wheel" pos="0 0.25 0">
                <joint name="left_wheel_joint" type="hinge" axis="0 1 0"/>
                <geom type="cylinder" size="0.1 0.04" euler="1.57 0 0" mass="2"/>
              </body>
              
              <!-- 右轮 -->
              <body name="right_wheel" pos="0 -0.25 0">
                <joint name="right_wheel_joint" type="hinge" axis="0 1 0"/>
                <geom type="cylinder" size="0.1 0.04" euler="1.57 0 0" mass="2"/>
              </body>
              
              <!-- UR5e机械臂 -->
              <include file="ur5e/ur5e_kinematics.xml"/>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  
  <actuator>
    <!-- 保留虚拟基座执行器（用于验证） -->
    <position name="act_base_x" joint="base_x" kp="2000"/>
    <position name="act_base_y" joint="base_y" kp="2000"/>
    <position name="act_base_z" joint="base_z" kp="35000"/>
    <position name="act_base_yaw" joint="base_yaw" kp="2000"/>
    
    <!-- 轮子电机 -->
    <motor name="act_left_wheel" joint="left_wheel_joint" 
           gear="50" ctrlrange="-10 10"/>
    <motor name="act_right_wheel" joint="right_wheel_joint" 
           gear="50" ctrlrange="-10 10"/>
    
    <!-- 机械臂（保持不变） -->
    <motor name="act_shoulder_pan" joint="shoulder_pan_joint" 
           ctrlrange="-150 150"/>
    <!-- ... 其他关节 ... -->
  </actuator>
</mujoco>
```

**关键点**：
- 保留虚拟基座关节（用于测试和对比）
- 添加物理轮子（有转动惯量）
- 轮子电机使用 `gear` 参数模拟减速器

---

### Step 2: 实现轮子动力学模型（1天）

创建 `wheeled_dynamics.py`：

```python
class WheeledUR5eDynamics(aligator.dynamics.ExplicitDynamicsModel):
    """
    带轮子动力学的移动机械臂
    
    状态: x = [q_base(4), θ_wheels(2), q_arm(6), v_base(3), ω_wheels(2), v_arm(6)]
           = 12-dim q + 11-dim v = 23-dim
    
    控制: u = [τ_left, τ_right, tau_arm(6)] = 8-dim
    """
    
    def __init__(self, pin_robot, dt, wheel_params):
        nx = 23  # 状态维度
        nu = 8   # 控制维度
        space = aligator.manifolds.VectorSpace(nx)
        super().__init__(space, nu)
        
        self.pin_robot = pin_robot
        self.dt = dt
        
        # 轮子参数
        self.wheel_radius = wheel_params['radius']  # 0.1 m
        self.wheelbase = wheel_params['base']      # 0.5 m
        self.I_wheel = wheel_params['inertia']     # 0.01 kg·m²
        self.friction_coeff = wheel_params['friction']  # 0.5
    
    def forward(self, x, u, data):
        """前向动力学积分"""
        
        # 提取状态
        q_base = x[0:4]      # [x, y, z, yaw]
        θ_wheels = x[4:6]    # [θ_left, θ_right]
        q_arm = x[6:12]      # 机械臂
        v_base = x[12:15]    # [vx, vy, ω_yaw] in world frame
        ω_wheels = x[15:17]  # 轮速
        v_arm = x[17:23]     # 机械臂速度
        
        # 提取控制
        τ_left, τ_right = u[0], u[1]
        τ_arm = u[2:8]
        
        # 1. 轮子动力学
        # 计算地面反力（从基座加速度推导）
        F_left, F_right = self._compute_ground_forces(x, u)
        
        # 轮子加速度
        α_left = (τ_left - self.friction_coeff * ω_wheels[0] 
                  - F_left * self.wheel_radius) / self.I_wheel
        α_right = (τ_right - self.friction_coeff * ω_wheels[1] 
                   - F_right * self.wheel_radius) / self.I_wheel
        
        # 2. 基座动力学（从轮速计算）
        v_linear = self.wheel_radius * (ω_wheels[0] + ω_wheels[1]) / 2
        ω_angular = (self.wheel_radius * (ω_wheels[1] - ω_wheels[0]) 
                     / self.wheelbase)
        
        # 非完整约束：body frame速度
        yaw = q_base[3]
        vx_world = v_linear * np.cos(yaw)
        vy_world = v_linear * np.sin(yaw)
        
        # 基座加速度（简化：忽略机械臂反作用力）
        a_base_x = 0  # 由轮速决定，非直接加速
        a_base_y = 0
        α_yaw = 0  # 由轮速差决定
        
        # 3. 机械臂动力学（ABA，同Phase 4）
        a_arm = self._compute_arm_acceleration(q_arm, v_arm, τ_arm)
        
        # 4. 积分（semi-implicit Euler）
        # 轮子
        ω_wheels_next = ω_wheels + self.dt * np.array([α_left, α_right])
        θ_wheels_next = θ_wheels + self.dt * ω_wheels_next
        
        # 基座（从轮速积分）
        q_base_next = self._integrate_base(q_base, ω_wheels_next)
        v_base_next = np.array([vx_world, vy_world, ω_angular])
        
        # 机械臂
        v_arm_next = v_arm + self.dt * a_arm
        q_arm_next = q_arm + self.dt * v_arm_next
        
        # 组装下一状态
        data.xnext = np.concatenate([
            q_base_next, θ_wheels_next, q_arm_next,
            v_base_next, ω_wheels_next, v_arm_next
        ])
    
    def _compute_ground_forces(self, x, u):
        """计算轮地接触力（简化模型）"""
        # 简化：假设力均分
        total_weight = 50 * 9.81  # N
        F_left = total_weight / 2
        F_right = total_weight / 2
        return F_left, F_right
    
    def _integrate_base(self, q_base, ω_wheels):
        """从轮速积分基座位置"""
        x, y, z, yaw = q_base
        
        v_linear = self.wheel_radius * (ω_wheels[0] + ω_wheels[1]) / 2
        ω_angular = (self.wheel_radius * (ω_wheels[1] - ω_wheels[0]) 
                     / self.wheelbase)
        
        # 积分
        x_next = x + self.dt * v_linear * np.cos(yaw)
        y_next = y + self.dt * v_linear * np.sin(yaw)
        z_next = z  # 假设平地
        yaw_next = yaw + self.dt * ω_angular
        
        return np.array([x_next, y_next, z_next, yaw_next])
```

---

### Step 3: 添加非完整约束（1天）

在MPC问题中添加约束：

```python
class WheeledMPCProblem:
    def build_problem(self, x0, ref_traj):
        # ... 标准MPC设置 ...
        
        # 添加非完整约束到每个阶段
        for i in range(self.horizon):
            # 约束：vy_body = 0（不能侧滑）
            vy_constraint = NonholonomicConstraint(
                wheelbase=0.5,
                wheel_radius=0.1
            )
            stage.addConstraint(vy_constraint, constraint_set)
        
        return problem

class NonholonomicConstraint(aligator.constraints.EqualityConstraint):
    """非完整约束：vy_body = 0"""
    
    def compute(self, x, u, data):
        q_base = x[0:4]
        v_base = x[12:15]  # [vx_world, vy_world, ω_yaw]
        
        yaw = q_base[3]
        vx_world, vy_world = v_base[0], v_base[1]
        
        # 转换到body frame
        vx_body = vx_world * np.cos(yaw) + vy_world * np.sin(yaw)
        vy_body = -vx_world * np.sin(yaw) + vy_world * np.cos(yaw)
        
        # 约束残差
        data.value = vy_body  # 应该等于0
```

---

### Step 4: 测试和验证（1-2天）

**测试1：轮子前进**
```python
def test_straight_motion():
    """测试直线前进"""
    τ_left = τ_right = 5.0  # 相等扭矩
    # 预期：机器人直线前进，vy_body ≈ 0
```

**测试2：原地旋转**
```python
def test_spin():
    """测试原地旋转"""
    τ_left = -5.0
    τ_right = 5.0  # 反向扭矩
    # 预期：原地旋转，vx_body ≈ 0
```

**测试3：圆弧运动**
```python
def test_arc():
    """测试圆弧轨迹"""
    τ_left = 3.0
    τ_right = 5.0  # 不等扭矩
    # 预期：圆弧轨迹
```

**测试4：MPC收敛性**
```python
def test_mpc_with_wheels():
    """测试带轮子约束的MPC"""
    # 目标：EE画圆，基座配合移动
    # 验证：非完整约束满足，MPC收敛
```

---

## 🎯 验证标准

Phase 5完成的标志：

- [ ] MJCF模型加载成功，轮子可见
- [ ] 轮子动力学计算正确（单元测试通过）
- [ ] 非完整约束满足：|vy_body| < 0.01 m/s
- [ ] MPC收敛率 >20%（允许比Phase 1-3低）
- [ ] 闭环稳定运行30秒
- [ ] EE跟踪误差 <5cm（允许比Phase 1-3差）

**注意**：Phase 5的目标是**功能完成**，不是性能优化。性能优化在Phase 6（MPC+WBC）中解决。

---

## 📊 预期性能

| 指标 | Phase 1-3（虚拟基座） | Phase 5（轮子） | 说明 |
|------|---------------------|----------------|------|
| 收敛率 | 100% | 20-50% | 约束更复杂 |
| EE误差 | 1.5-2.1 cm | 3-5 cm | 轮子约束限制 |
| 求解时间 | 15 ms | 30-50 ms | 状态增加 |

**为什么性能会下降？**
- 非完整约束增加了优化难度
- 轮子动力学引入了新的耦合
- 状态空间从10维→23维

**不用担心**：Phase 6的MPC+WBC架构会解决这些问题！

---

## 🔗 下一步

Phase 5完成后，进入Phase 6：
- MPC层：可以继续用运动学MPC（复用Phase 1-3）
- WBC层：将MPC的速度目标转换为轮子扭矩
- 非完整约束：在WBC的QP中处理

---

**准备好开始Phase 5了吗？** 我可以帮你：
1. 创建初始的MJCF模型
2. 实现轮子动力学类的框架
3. 设计单元测试

你想从哪里开始？🚀
