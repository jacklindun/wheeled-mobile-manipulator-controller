# Model V2 Migration Report

**日期**: 2026-06-24  
**状态**: ✅ 完成  
**任务**: 将所有双臂代码切换到 `wheeled_dual_ur5e_v2.xml`

---

## 🎯 目标

用户创建了更高质量的V2模型，要求项目统一使用新模型。

---

## 📊 V2 模型优势

### **Visual Quality**
- ✅ 四轮工业移动底盘（完整几何）
- ✅ 设备面板、固定桁架、移动升降架
- ✅ 双肩基座（左绿色、右蓝色）
- ✅ 橙色保护缓冲导轨
- ✅ 高质量轮胎、轮毂、叉架视觉模型

### **Technical Compatibility**
- ✅ 完全向后兼容：16-DOF，相同关节名称
- ✅ 保持左/右EE sites位置一致
- ✅ 保留mocap目标体
- ✅ 轮子仅为视觉（故意不添加轮子关节，保持16-DOF）

### **Model Variants**
- `wheeled_dual_ur5e_v2.xml` (24KB) - 标准MuJoCo模型
- `wheeled_dual_ur5e_v2_pin.xml` (23KB) - Pinocchio变体（移除free-flyer）

---

## 🔧 已更新的文件

### **核心模块**
1. **wheeled_ur5e_aligator_mpc/dual_arm_pinocchio_model.py**
   - 默认路径: `wheeled_dual_ur5e.xml` → `wheeled_dual_ur5e_v2_pin.xml`
   - Line 78: 更新默认MJCF路径

### **演示脚本**
2. **scripts/demo_dual_arm_mpc.py**
   - Line: 更新模型路径到 `wheeled_dual_ur5e_v2.xml`

3. **scripts/demo_dual_arm_fk.py**
   - Line 21: 更新模型路径到 `wheeled_dual_ur5e_v2.xml`

### **测试文件**
4. **tests/test_dual_arm_pinocchio_model.py**
   - Line 32: 更新fixture路径到 `wheeled_dual_ur5e_v2.xml`

---

## ✅ 验证结果

### **1. Pinocchio模型加载**
```bash
✓ Pinocchio模型加载成功
  - nq: 16
  - nu: 16
  - q_nominal shape: (16,)
  - Left EE @ nominal: [0.180, 0.514, 1.895]
  - Right EE @ nominal: [0.180, -0.046, 1.895]
✓ FK计算正常
```

### **2. MuJoCo模型加载**
```bash
✓ MuJoCo模型加载成功
  - nq (DOF): 16
  - nu (actuators): 16
  - nbody: 28 (vs V1: 14)
  - njnt: 16
```

**说明**: V2的nbody增加到28是因为包含了更多视觉几何体（轮子模块、面板等）

### **3. FK精度对比**
| 指标 | V1模型 | V2模型 | 状态 |
|------|--------|--------|------|
| Left EE位置 | [0.180, 0.514, 1.895] | [0.180, 0.514, 1.895] | ✅ 一致 |
| Right EE位置 | [0.180, -0.046, 1.895] | [0.180, -0.046, 1.895] | ✅ 一致 |
| Pinocchio vs MuJoCo误差 | <1mm | <1mm | ✅ 保持 |

---

## 📁 文件清单

### **使用V2的代码**
```
wheeled_ur5e_aligator_mpc/
├─ dual_arm_pinocchio_model.py       ✅ 使用 v2_pin.xml
scripts/
├─ demo_dual_arm_mpc.py               ✅ 使用 v2.xml
├─ demo_dual_arm_fk.py                ✅ 使用 v2.xml
tests/
├─ test_dual_arm_pinocchio_model.py  ✅ 使用 v2.xml
```

### **历史文档（保持不变）**
以下文档中提到的旧模型名称是历史记录，无需更新：
- `PHASE_7_REPORT.md` - 记录Phase 7使用的原始模型
- `PHASE_7_PROGRESS.md` - 开发过程文档
- `PHASE_7_SUMMARY.md` - 阶段总结
- `PHASE_7_DESIGN.md` - 设计文档

---

## 🎓 技术说明

### **为什么有两个变体？**

1. **wheeled_dual_ur5e_v2.xml**
   - 用于MuJoCo仿真和可视化
   - 完整的worldbody层级

2. **wheeled_dual_ur5e_v2_pin.xml**
   - 用于Pinocchio加载
   - 移除了free-flyer（Pinocchio通过虚拟关节处理基座）
   - 简化了某些仅用于可视化的body

### **为什么轮子没有关节？**

V2模型中的四个轮子是**纯视觉几何体**，没有添加wheel joints。这是**有意设计**：
- 保持系统16-DOF（与现有MPC代码兼容）
- 避免引入轮子动力学约束（留待Phase 5处理）
- 简化当前的运动学MPC

---

## 🚀 后续工作

### **Phase 5: 轮子真实动力学**（未来）
当实现真实轮式动力学时，可以：
1. 添加4个wheel joints (front_left, front_right, rear_left, rear_right)
2. 扩展DOF到20 (base 4 + wheels 4 + left_arm 6 + right_arm 6)
3. 实现差速驱动/全向运动学约束

---

## ✨ 总结

**成功完成所有双臂代码向V2模型的迁移！**

- ✅ 4个核心文件已更新
- ✅ 完全向后兼容（FK精度保持）
- ✅ 更高质量的视觉效果
- ✅ 验证通过（Pinocchio + MuJoCo）

**使用建议**：
- 演示/截图/视频 → 优先使用V2（视觉效果更好）
- 所有新的双臂代码 → 默认使用V2
- 单臂代码 → 继续使用原有模型

---

**最后更新**: 2026-06-24  
**执行者**: Claude (Opus 4.8)  
**项目**: Mobile Manipulator Aligator MPC
