# Phase 6 模型文件说明

**日期**: 2024-06-24

---

## 📁 Assets文件夹中的模型文件

### 单臂模型

| 文件名 | 大小 | DOF | 描述 | 用途 |
|--------|------|-----|------|------|
| **wheeled_ur5e.xml** | 6.7K | 10 | 虚拟基座 + UR5e机械臂 | Pinocchio原默认，Phase 1-3 |
| **wheeled_ur5e_wheels.xml** | 8.8K | 12 | 差速驱动轮子 + UR5e | Phase 4-6，带物理轮子 |
| **wheeled_ur5e_hybrid.xml** | 5.7K | ? | 混合模型 | Phase 4混合动力学？ |
| **wheeled_ur5e_pin.xml** | 6.6K | 10 | Pinocchio处理后的版本 | 自动生成，缓存 |

### 双臂模型

| 文件名 | 大小 | DOF | 描述 |
|--------|------|-----|------|
| **wheeled_dual_ur5e.xml** | 12K | ? | 双UR5e机械臂 |
| **wheeled_dual_ur5e_pin.xml** | 11K | ? | Pinocchio版本 |

### 机械臂单独模型

| 文件名 | 大小 | DOF | 描述 |
|--------|------|-----|------|
| **ur5e_arm_6dof.xml** | 3.4K | 6 | 仅UR5e机械臂（无基座） |

---

## 🔍 当前Phase 6的模型不一致问题

### 问题描述

**MuJoCo测试使用**: `wheeled_ur5e_wheels.xml` (12 DOF)
- base_x, base_y, base_yaw, base_z (4)
- left_wheel, right_wheel (2)
- arm joints (6)

**Pinocchio原本加载**: `wheeled_ur5e.xml` (10 DOF)
- base_x, base_y, base_yaw, base_z (4)
- arm joints (6)
- **没有轮子关节**

**结果**: FK差异 137.9mm，导致闭环测试失败

---

## 🎯 推荐的模型使用方案

### 方案1: 统一使用wheels模型（推荐Phase 6）

**适用场景**: Phase 4-6，需要物理轮子动力学

**修改**:
1. Pinocchio加载 `wheeled_ur5e_wheels.xml`
2. 更新 `nq=12`, `nu=12`
3. 处理轮子关节状态

**优点**:
- 与Phase 4混合动力学一致
- 支持真实的差速驱动控制
- FK完全一致

**缺点**:
- 需要更新Pinocchio模型代码
- 轮子关节增加状态维度

### 方案2: 统一使用虚拟基座模型

**适用场景**: Phase 1-3，纯运动学MPC

**修改**:
1. MuJoCo测试改用 `wheeled_ur5e.xml`
2. 保持 `nq=10`, `nu=10`
3. 基座使用速度控制

**优点**:
- 简单，无需修改Pinocchio
- 状态维度更小

**缺点**:
- 失去轮子动力学特性
- 不适合Phase 4-6的设计目标

### 方案3: 为不同Phase使用不同模型

**设计原则**:
- **Phase 1-3**: `wheeled_ur5e.xml` (运动学MPC)
- **Phase 4-6**: `wheeled_ur5e_wheels.xml` (动力学MPC)
- **Phase 7+**: 根据需求选择

**实现**:
- Pinocchio模型构造函数接受 `mjcf_path` 参数
- 测试脚本明确指定模型文件
- 添加模型验证工具

---

## ✅ 立即行动：修复Phase 6

### Step 1: 统一Pinocchio模型到wheels (已完成)

```python
# pinocchio_model.py:76
mjcf_path = str(
    Path(__file__).resolve().parents[1] / "assets" / "wheeled_ur5e_wheels.xml"
)
```

### Step 2: 更新DOF数量

```python
# pinocchio_model.py:57-58
nq: int = 12  # base(4) + wheels(2) + arm(6)
nu: int = 12
```

### Step 3: 更新关节名称列表

```python
_BASE_JOINTS = ["base_x", "base_y", "base_yaw", "base_z"]
_WHEEL_JOINTS = ["left_wheel_joint", "right_wheel_joint"]  # 新增
_ARM_JOINTS = [...]
```

### Step 4: 更新状态转换逻辑

需要处理轮子关节在状态向量中的位置。

### Step 5: 验证FK一致性

运行 `scripts/check_robot_configuration_text.py`，确认FK误差 < 1mm

### Step 6: 重新运行闭环测试

预期EE误差从117cm降至 < 10cm

---

## 📝 建议：添加模型验证工具

创建一个自动化工具验证MuJoCo和Pinocchio使用相同模型：

```python
def verify_model_consistency(mujoco_path, pinocchio_model):
    """验证MuJoCo和Pinocchio的FK一致性"""
    # 加载MuJoCo模型
    m = mujoco.MjModel.from_xml_path(mujoco_path)
    d = mujoco.MjData(m)
    
    # 测试多个配置
    for q in test_configs:
        p_mj = get_ee_pos_mujoco(m, d, q)
        p_pin = pinocchio_model.fk_pose(q)[0]
        
        error = np.linalg.norm(p_mj - p_pin)
        assert error < 0.001, f"FK不一致: {error*1000:.2f}mm"
```

在测试开始时自动调用，确保模型一致。

---

## 🎓 经验教训

1. **明确文档化每个模型文件的用途**
2. **在代码中添加模型验证检查**
3. **测试脚本应明确指定使用的模型**
4. **不同Phase可能需要不同的模型文件**
5. **FK一致性是控制器正常工作的前提**

---

**下一步**: 是否继续完成Step 2-6的修复工作？

