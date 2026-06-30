# Phase 7: 双臂扩展

**目标**：从单臂10-DOF系统扩展到双臂16-DOF系统

**状态**：设计中  
**预计工作量**：5-7天  
**前置条件**：Phase 1-6完成

---

## 🎯 系统配置

### 从单臂到双臂

**当前系统（Phase 1-6）**：
```
基座: 4-DOF (x, y, z, yaw)
右臂: 6-DOF (UR5e)
────────────────────────
总计: 10-DOF
```

**目标系统（Phase 7）**：
```
基座:  4-DOF (x, y, z, yaw)
左臂:  6-DOF (UR5e)
右臂:  6-DOF (UR5e)
────────────────────────
总计: 16-DOF
```

---

## 🏗️ 实现步骤

### Step 1: 创建双臂MJCF模型（1天）

创建 `wheeled_dual_ur5e.xml`：

```xml
<mujoco model="wheeled_dual_ur5e">
  <worldbody>
    <!-- 基座 -->
    <body name="base">
      
      <!-- 底盘 -->
      <body name="chassis">
        
        <!-- 左臂：挂载在左侧 -->
        <body name="left_shoulder_link" pos="0 0.3 0.27">
          <!-- UR5e左臂，关节名加 left_ 前缀 -->
          <joint name="left_shoulder_pan_joint" .../>
          <!-- ... 完整UR5e链 -->
          <site name="left_ee_site" .../>
        </body>
        
        <!-- 右臂：挂载在右侧 -->
        <body name="right_shoulder_link" pos="0 -0.3 0.27">
          <!-- UR5e右臂，关节名加 right_ 前缀 -->
          <joint name="right_shoulder_pan_joint" .../>
          <!-- ... 完整UR5e链 -->
          <site name="right_ee_site" .../>
        </body>
        
      </body>
    </body>
  </worldbody>
  
  <actuator>
    <!-- 基座执行器 -->
    <velocity name="act_base_x" .../>
    ...
    
    <!-- 左臂执行器 -->
    <motor name="act_left_shoulder_pan" .../>
    <motor name="act_left_shoulder_lift" .../>
    <motor name="act_left_elbow" .../>
    <motor name="act_left_wrist_1" .../>
    <motor name="act_left_wrist_2" .../>
    <motor name="act_left_wrist_3" .../>
    
    <!-- 右臂执行器 -->
    <motor name="act_right_shoulder_pan" .../>
    <motor name="act_right_shoulder_lift" .../>
    <motor name="act_right_elbow" .../>
    <motor name="act_right_wrist_1" .../>
    <motor name="act_right_wrist_2" .../>
    <motor name="act_right_wrist_3" .../>
  </actuator>
</mujoco>
```

**关键点**：
- 左臂和右臂对称挂载（y轴±0.3m）
- 关节名使用 `left_` 和 `right_` 前缀区分
- 两个EE site：`left_ee_site`, `right_ee_site`

---

### Step 2: 扩展Pinocchio模型（1天）

创建 `DualArmPinocchioModel`：

```python
class DualArmPinocchioModel:
    """
    双臂Pinocchio模型
    
    状态空间: 16-DOF
      q = [base(4), left_arm(6), right_arm(6)]
    """
    
    def __init__(self):
        # 加载双臂URDF/MJCF
        self.model = pin.buildModelFromMJCF(mjcf_path)
        self.data = self.model.createData()
        
        # EE frames
        self.left_ee_frame_id = self.model.getFrameId("left_ee_site")
        self.right_ee_frame_id = self.model.getFrameId("right_ee_site")
    
    def fk_left_ee(self, q):
        """左臂正向运动学"""
        pin.framesForwardKinematics(self.model, self.data, q)
        return self.data.oMf[self.left_ee_frame_id]
    
    def fk_right_ee(self, q):
        """右臂正向运动学"""
        pin.framesForwardKinematics(self.model, self.data, q)
        return self.data.oMf[self.right_ee_frame_id]
    
    def jacobian_left_ee(self, q):
        """左臂雅可比"""
        J = pin.computeFrameJacobian(
            self.model, self.data, q, 
            self.left_ee_frame_id, 
            pin.LOCAL_WORLD_ALIGNED
        )
        return J
    
    def jacobian_right_ee(self, q):
        """右臂雅可比"""
        J = pin.computeFrameJacobian(
            self.model, self.data, q, 
            self.right_ee_frame_id, 
            pin.LOCAL_WORLD_ALIGNED
        )
        return J
```

---

### Step 3: 双臂MPC问题（2天）

扩展状态空间和代价函数：

```python
class DualArmMPCProblem:
    """
    双臂MPC问题构建器
    
    状态: q(16), u(16)
    代价: 双EE跟踪 + 双臂姿态正则化
    """
    
    def build_problem(self, x0, ref_traj_left, ref_traj_right):
        """
        构建双臂MPC问题
        
        Parameters
        ----------
        x0 : (16,) array
            初始状态
        ref_traj_left : dict
            左臂参考轨迹 {"ee_pos": (N,3), ...}
        ref_traj_right : dict
            右臂参考轨迹 {"ee_pos": (N,3), ...}
        """
        
        for k in range(self.horizon):
            # 左臂EE跟踪
            left_ee_cost = self._build_ee_cost(
                k, ref_traj_left["ee_pos"][k], 
                self.left_ee_frame_id,
                weight=self.weights["left_ee_pos"]
            )
            
            # 右臂EE跟踪
            right_ee_cost = self._build_ee_cost(
                k, ref_traj_right["ee_pos"][k], 
                self.right_ee_frame_id,
                weight=self.weights["right_ee_pos"]
            )
            
            # 双臂协调代价（可选）
            if self.enable_coordination:
                coord_cost = self._build_coordination_cost(k, ref_traj)
            
            stage_cost = left_ee_cost + right_ee_cost + coord_cost
            problem.addCost(stage_cost)
```

---

### Step 4: 双臂任务场景（2-3天）

#### 场景1: 独立运动
```python
def scenario_independent():
    """
    两臂独立运动
    
    左臂: EE画圆（Y-Z平面）
    右臂: EE直线（X方向）
    """
    ref_left = generate_circle_trajectory(
        center=[0.5, 0.3, 0.8],
        radius=0.1,
        normal=[1, 0, 0],  # X-Y平面
    )
    
    ref_right = generate_line_trajectory(
        start=[0.5, -0.3, 0.8],
        end=[0.7, -0.3, 0.8],
    )
    
    return ref_left, ref_right
```

#### 场景2: 协同搬运
```python
def scenario_carry_object():
    """
    双臂协同搬运物体
    
    约束: 保持相对姿态
    """
    # 物体尺寸
    object_width = 0.3  # m
    
    ref_left = generate_trajectory(...)
    ref_right = generate_trajectory(...)
    
    # 添加相对位置约束
    for k in range(N):
        # 保持固定间距
        constraint = ||p_left[k] - p_right[k]|| == object_width
    
    return ref_left, ref_right
```

#### 场景3: 主从协作
```python
def scenario_primary_secondary():
    """
    主臂操作，从臂支撑
    
    左臂(主): 执行精细操作
    右臂(从): 保持固定姿态支撑
    """
    ref_left = generate_manipulation_trajectory(...)
    
    # 右臂保持不动
    ref_right = {
        "ee_pos": np.tile(right_ee_start, (N, 1)),
        "ee_rot": np.tile(right_rot_start, (N, 1, 1)),
    }
    
    return ref_left, ref_right
```

---

## 📊 状态空间对比

| 系统 | 状态维度 | 控制维度 | 质量矩阵 | 计算复杂度 |
|------|---------|---------|---------|-----------|
| 单臂 | 10 | 10 | 10×10 | 基准 |
| 双臂 | 16 | 16 | 16×16 | 2.56× |
| 双臂+轮子 | 29 | 18 | 17×17 | 2.89× |

**预期性能**：
- MPC求解时间：15ms → 30-50ms
- WBC求解时间：0.15ms → 0.3-0.5ms
- 仍可达到20Hz MPC + 100Hz WBC

---

## 🎯 验证标准

Phase 7完成的标志：

- [ ] 双臂MJCF模型加载成功
- [ ] 双EE FK计算正确
- [ ] 双臂独立运动任务：EE误差 <5cm
- [ ] 双臂协同任务：相对位置误差 <2cm
- [ ] MPC收敛率 >80%
- [ ] 闭环稳定运行30秒
- [ ] 无自碰撞

---

## 🔧 技术挑战

### 1. 自碰撞检测
**问题**：两臂可能相互碰撞

**解决方案**：
- 在MPC中添加碰撞约束
- 使用碰撞检测库（HPP-FCL）
- 或简单的球近似

### 2. 工作空间冲突
**问题**：两臂工作空间重叠

**解决方案**：
- 任务规划时避免重叠
- 软约束（cost penalty）
- 优先级（左臂主，右臂从）

### 3. 计算复杂度
**问题**：16-DOF的优化问题更大

**解决方案**：
- 利用稀疏性（左右臂解耦）
- 并行计算左右臂雅可比
- 分层优化（基座→左臂→右臂）

---

## 📝 文件结构

```
Phase 7新增文件:
assets/
  └─ wheeled_dual_ur5e.xml          # 双臂MJCF

wheeled_ur5e_aligator_mpc/
  ├─ dual_arm_model.py               # 双臂Pinocchio模型
  ├─ dual_arm_problem.py             # 双臂MPC问题
  └─ dual_arm_scenarios.py           # 双臂任务场景

tests/
  ├─ test_dual_arm_model.py          # 双臂模型测试
  └─ test_dual_arm_mpc.py            # 双臂MPC测试

scripts/
  ├─ demo_dual_arm_independent.py    # 独立运动演示
  ├─ demo_dual_arm_coordination.py   # 协同演示
  └─ visualize_dual_arm.py           # 可视化工具

docs/
  ├─ PHASE_7_DESIGN.md               # 本文档
  └─ PHASE_7_SUMMARY.md              # 完成总结（待创建）
```

---

## 🚀 开始Phase 7

准备好了吗？我可以帮你：

1. **创建双臂MJCF模型**（从单臂复制+修改）
2. **实现双臂Pinocchio接口**
3. **扩展MPC问题到双EE**
4. **创建第一个双臂演示**

你想从哪里开始？🎉

**推荐顺序**：
1. 先创建双臂MJCF模型（看到两个机械臂）
2. 然后实现双FK（验证两个EE位置正确）
3. 再做双EE跟踪MPC
4. 最后实现协同任务

从第1步开始吗？
