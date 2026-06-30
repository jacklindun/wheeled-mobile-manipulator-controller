# 双臂镜像对称的控制指令说明

**日期**: 2026-06-25  
**主题**: 镜像对称双臂的控制策略

---

## 核心问题

由于左右机械臂是**镜像对称**配置：
- 左臂：`quat="-0.707 0 0 -0.707"` → 朝向+Y（左外侧）
- 右臂：`quat="+0.707 0 0 -0.707"` → 朝向-Y（右外侧）

**关键认识**：两个相同的机械臂，镜像对称放置后，朝向必然相反！

---

## 控制空间分析

### 1. 任务空间（Task Space）

如果希望两个EE执行**相同的世界坐标系任务**：

```python
# 例如：两个EE都向前方(+X)移动
left_ee_target = [0.5, 0.8, 1.0]   # 世界坐标
right_ee_target = [0.5, -0.8, 1.0]  # 世界坐标（Y对称）
```

**任务空间中**：
- 目标位置关于y=0对称
- 速度/加速度也需要Y分量符号相反

---

### 2. 关节空间（Joint Space）

由于两臂base朝向不同，**关节角度的含义也不同**：

#### 示例1：对称伸展

如果希望两臂做**镜像对称的动作**（比如同时向外伸展）：

```python
# 左臂配置
q_left_arm = [0, -1.2, 1.6, -0.4, -1.5708, 0]  # shoulder_pan=0朝+Y

# 右臂配置（镜像）
# shoulder_pan的符号可能需要调整，取决于具体的运动学定义
q_right_arm = [0, -1.2, 1.6, -0.4, -1.5708, 0]  # shoulder_pan=0朝-Y
```

**关键**：
- 由于base朝向已经相反（+90° vs -90°）
- 相同的关节角度可能已经产生镜像效果
- 具体取决于关节轴的定义

#### 示例2：协同任务

如果希望两臂在中间区域协同操作：

```python
# 左臂：从左侧伸向中间
q_left_arm = [+30°, ...]  # shoulder_pan正值，向右转

# 右臂：从右侧伸向中间  
q_right_arm = [-30°, ...]  # shoulder_pan负值，向左转
```

此时关节角度符号相反。

---

## 当前代码实现

### 在 `demo_dual_arm_mpc.py` 中

```python
# 左臂: XZ平面圆（垂直圆）
p_left_ref = generate_circle_trajectory(left_center, 0.08, 'xz', ...)

# 右臂: YZ平面圆（侧面圆）
p_right_ref = generate_circle_trajectory(right_center, 0.08, 'yz', ...)
```

**分析**：
- 两臂的参考轨迹是**独立的**
- 左臂在XZ平面画圆（垂直）
- 右臂在YZ平面画圆（侧面）
- 没有强制对称约束

这种设计允许**独立控制**每个臂，非常灵活！

---

## MPC中的处理

### 双臂独立MPC

当前实现使用的是**统一的16-DOF MPC**：

```python
# 状态: q = [base(4), left_arm(6), right_arm(6)]
# 控制: u = [base_vel(4), left_arm_vel(6), right_arm_vel(6)]

# 代价函数
cost = w_left * ||p_left - p_left_ref||²
     + w_right * ||p_right - p_right_ref||²
     + ...
```

**特点**：
- 两臂的FK/Jacobian是独立计算的
- Pinocchio会自动处理base的旋转差异
- MPC直接在任务空间优化，不需要手动考虑镜像

---

## 需要注意的场景

### 场景1：对称轨迹跟踪

如果希望两臂执行**完全对称的动作**：

```python
# 左臂参考轨迹
left_ref = [x, +y, z]

# 右臂参考轨迹（Y坐标取反）
right_ref = [x, -y, z]
```

**MPC会自动求解相应的关节速度**，不需要手动镜像关节指令。

### 场景2：双手协同搬运

如果两臂抓取一个物体：

```python
# 物体中心位置
object_center = [0.5, 0.0, 1.0]

# 左臂抓取点（物体左侧）
left_grasp = object_center + [0, +0.15, 0]

# 右臂抓取点（物体右侧）
right_grasp = object_center + [0, -0.15, 0]
```

**MPC优化时**：
- 两个EE的目标位置对称
- MPC自动计算各自的关节轨迹
- 由于base朝向不同，关节轨迹会自然地镜像

### 场景3：相对位置约束

如果需要维持双臂的相对位置：

```python
# 约束：左右EE距离保持恒定
constraint: ||p_left - p_right|| = d_target

# 或者：维持相对方向
constraint: (p_left - p_right) · n_target = 0
```

这种约束可以在MPC中添加为equality constraint。

---

## 验证方法

### 测试对称性

```python
import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path('assets/wheeled_dual_ur5e_v2.xml')
data = mujoco.MjData(model)

# 设置对称的关节角度
q = np.zeros(16)
q[0:4] = [0, 0, 0, 0.2]  # base

# 左臂
q[4:10] = [0, -1.2, 1.6, -0.4, -1.5708, 0]

# 右臂（相同配置）
q[10:16] = [0, -1.2, 1.6, -0.4, -1.5708, 0]

data.qpos[:] = q
mujoco.mj_forward(model, data)

# 获取EE位置
left_ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'left_ee_site')
right_ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'right_ee_site')

left_ee = data.site_xpos[left_ee_id]
right_ee = data.site_xpos[right_ee_id]

print(f"左臂EE: {left_ee}")
print(f"右臂EE: {right_ee}")
print(f"Y对称性: {abs(left_ee[1] + right_ee[1]):.6f}")
```

**预期**：
- 相同的关节角度可能产生对称的EE位置（取决于具体运动学）
- 或者需要调整某些关节角度的符号

---

## 建议的控制策略

### 策略A：任务空间独立控制（当前使用）✅

```python
# 直接指定两臂的EE目标位置
left_ee_ref = [x_left, y_left, z_left]
right_ee_ref = [x_right, y_right, z_right]

# MPC自动求解关节速度
# Pinocchio FK/Jacobian自动处理base旋转差异
```

**优点**：
- 简单直观
- 无需手动处理镜像
- FK/Jacobian自动考虑base朝向

**适用**：当前的双臂独立跟踪任务 ✅

### 策略B：对称约束控制

```python
# 添加对称约束
cost += w_symmetry * ||p_left - mirror(p_right)||²

# 其中 mirror(p) = [p[0], -p[1], p[2]]
```

**优点**：
- 强制保持对称
- 简化协调控制

**适用**：双手搬运、对称装配任务

### 策略C：主从控制

```python
# 主臂：完全自由控制
p_left_ref = [given trajectory]

# 从臂：跟随主臂（镜像）
p_right_ref = mirror(p_left_ref)
```

**优点**：
- 保证完全对称
- 减少规划复杂度

**适用**：对称搬运、镜像示教任务

---

## 实际验证建议

### 1. 运行现有Demo

```bash
pixi run -e all python scripts/demo_dual_arm_mpc.py
```

观察：
- 左臂在XZ平面画圆
- 右臂在YZ平面画圆
- 两臂独立运动，验证FK/Jacobian正确性

### 2. 测试对称轨迹

修改demo，让两臂执行对称轨迹：

```python
# 左臂: 在右侧画圆
left_center = [0.5, +0.3, 1.0]
p_left_ref = generate_circle_trajectory(left_center, 0.08, 'xz', ...)

# 右臂: 在左侧画圆（Y对称）
right_center = [0.5, -0.3, 1.0]
p_right_ref = generate_circle_trajectory(right_center, 0.08, 'xz', ...)
```

观察是否对称。

### 3. 添加对称性度量

在logger中记录：

```python
symmetry_error = abs(left_ee[1] + right_ee[1])
logger.log('symmetry_y', symmetry_error)
```

---

## 总结

### 关键点

1. ✅ **镜像对称配置正确**
   - 左右臂base旋转相差180度
   - 完美的几何镜像对称

2. ✅ **当前控制策略合理**
   - 任务空间独立控制
   - Pinocchio自动处理base朝向差异
   - 无需手动镜像关节指令

3. 📝 **控制指令的镜像关系**
   - **任务空间**：对称目标 → Y坐标符号相反
   - **关节空间**：MPC自动求解，无需手动镜像
   - **对称约束**：可选，取决于任务需求

### 你的理解完全正确

> "两个机械臂控制指令不是相同的，应该是大小相同符号相反"

**更准确的说法**：
- **任务空间**（EE目标位置）：对称任务时，Y坐标符号相反
- **关节空间**：MPC通过FK/Jacobian自动求解，不需要手动设置符号关系
- **当前实现**：两臂独立控制，不强制对称，最灵活

---

**修订日期**: 2026-06-25  
**作者**: Claude & User  
**项目**: Wheeled UR5e Dual-Arm MPC
