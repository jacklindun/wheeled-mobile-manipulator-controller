# Phase 6 导航指南

**创建日期**: 2026-06-25  
**目的**: 快速导航Phase 6所有相关文档和代码

---

## 📚 核心文档 (必读)

### 1. 工作总结 ⭐ 最重要
**文件**: `PHASE6_WORK_SUMMARY.md`  
**内容**: 
- 今天7小时工作总结
- 遇到的技术障碍
- 三个下一步方案
- 我的推荐：方案B (先用运动学MPC)

**什么时候读**: 现在立即阅读

---

### 2. 新架构设计
**文件**: `PHASE6_NEW_DESIGN.md`  
**内容**:
- Full Dynamic MPC + 插值 + 前馈PD架构
- 5步实施计划
- 详细API设计
- 预期性能分析

**什么时候读**: 开始实施前

---

### 3. 诊断报告
**文件**: `PHASE6_DIAGNOSIS_REPORT.md`  
**内容**:
- Phase 5验证 (9/9测试通过)
- 5个具体问题诊断
- 修复方案和时间估算

**什么时候读**: 遇到问题时查阅

---

## 🗂️ 历史文档 (参考)

### Phase 6 演进历史

| 版本 | 文档 | 状态 | 说明 |
|------|------|------|------|
| **Version 1** | `PHASE_6_DESIGN.md` | 设计 | 初始MPC+WBC双层架构 |
| **Version 2** | `PHASE_6_PROGRESS.md` | 失败 | 61cm误差，动力学残差10^5 |
| **Version 3** | `PHASE6_KINODYNAMIC_UPGRADE.md` | 未验证 | Kino-Dynamic升级方案 |
| **Version 3** | `PHASE6_FINAL_SUMMARY.md` | 72%完成 | 代码完整但未测试 |
| **Version 4** | `PHASE6_NEW_DESIGN.md` | 🌟 当前 | 插值+前馈PD新方案 |

---

## 💻 代码文件

### 主要代码
```
wheeled_ur5e_aligator_mpc/
├── full_dynamic_mpc_controller.py    # Step 1: Full Dynamic MPC
│   状态: 90%完成，C++崩溃未解决
│   大小: ~550行
│
├── trajectory_interpolator.py         # Step 2: 插值器 (待创建)
├── feedforward_pd_controller.py       # Step 3: 前馈PD (待创建)
└── phase6_controller.py               # Step 4: 集成 (待创建)
```

### 依赖的Phase 5代码
```
wheeled_ur5e_aligator_mpc/
├── wheeled_dynamics.py                # Phase 5动力学
│   测试: 9/9通过 ✅
│   状态空间: 23-dim
│   控制空间: 8-dim
│
└── pinocchio_model.py                 # Pinocchio模型
    包含: arm_model (6-DOF)
```

---

## 🎯 快速决策树

```
你想要什么？
│
├─ 快速了解Phase 6现状
│  → 读 PHASE6_WORK_SUMMARY.md
│
├─ 理解新架构设计
│  → 读 PHASE6_NEW_DESIGN.md
│
├─ 继续Phase 6开发
│  │
│  ├─ 方案A: 简化动力学MPC
│  │  → 读 PHASE6_STEP1_DEBUG.md
│  │  → 修改 full_dynamic_mpc_controller.py
│  │
│  ├─ 方案B: 先用运动学MPC ⭐ 推荐
│  │  → 复用 aligator_problem.py (Phase 1-3)
│  │  → 创建 trajectory_interpolator.py
│  │  → 创建 feedforward_pd_controller.py
│  │
│  └─ 方案C: 深度调试Phase 5
│     → 读 wheeled_dynamics.py
│     → 添加详细日志
│     → 单步调试C++
│
└─ 了解旧Phase 6为什么失败
   → 读 PHASE_4_CONCLUSION.md (积分器不匹配)
   → 读 PHASE6_COMPLETE_ASSESSMENT.md (欠驱动问题)
```

---

## 📊 Phase 6 状态速查

### 完成度

| 步骤 | 状态 | 完成度 | 阻塞原因 |
|------|------|--------|---------|
| **Step 1**: Full Dynamic MPC | 🔄 调试中 | 90% | C++崩溃 |
| **Step 2**: 插值器 | ⏳ 待开始 | 0% | 依赖Step 1 |
| **Step 3**: 前馈PD | ⏳ 待开始 | 0% | 依赖Step 1 |
| **Step 4**: 集成测试 | ⏳ 待开始 | 0% | 依赖Step 2-3 |
| **Step 5**: 调优评估 | ⏳ 待开始 | 0% | 依赖Step 4 |

**总体进度**: 18% (Step 1的90%)

### 时间投入

| 阶段 | 已用 | 预计剩余 | 总计 |
|------|------|---------|------|
| Step 1 | 0.7天 | 1-2天 | 2-3天 |
| Step 2-5 | 0天 | 3-4天 | 3-4天 |
| **总计** | 0.7天 | 4-6天 | 5-7天 |

---

## 🔧 常见问题

### Q1: Phase 6有几个版本？
A: 4个版本
- V1: MPC+WBC设计 (未实现)
- V2: 简化MPC+WBC (失败，61cm误差)
- V3: Kino-Dynamic MPC+WBC (未验证)
- V4: Full Dynamic MPC+插值+前馈PD (当前)

### Q2: 为什么Phase 4失败？
A: 积分器不匹配
- ALIGATOR: semi-implicit Euler
- MuJoCo: implicitfast
- 25步累积误差放大480倍

### Q3: 旧Phase 6 (MPC+WBC)为什么未完成？
A: 欠驱动系统问题
- 11个加速度 vs 8个控制输入
- 基座动力学无法完全满足
- 动力学残差≈83 (目标<0.1)

### Q4: 新Phase 6有什么不同？
A: 插值+前馈PD绕过积分器问题
- MPC 20Hz规划
- 插值0.05s→0.002s
- PD 500Hz补偿模型误差

### Q5: 现在应该做什么？
A: 三个选项 (详见PHASE6_WORK_SUMMARY.md)
- 方案A: 简化继续 (4-6天)
- 方案B: 运动学MPC先行 (5-6天) ⭐ 推荐
- 方案C: 深度调试 (6-9天)

---

## 📁 文件清单

### 主目录 (`study_example/wheeled_ur5e_aligator_mpc/`)

**Phase 6新文档** (2026-06-25创建):
```
PHASE6_WORK_SUMMARY.md                 # 工作总结 ⭐
PHASE6_NEW_DESIGN.md                   # 新架构设计
PHASE6_DIAGNOSIS_REPORT.md             # 诊断报告
PHASE6_COMPLETE_ASSESSMENT.md          # 完整评估
PHASE6_STATUS_AND_NEXT_STEPS.md        # 状态和下一步
PHASE6_IMPLEMENTATION_PROGRESS.md      # 实施进度
PHASE6_STEP1_DEBUG.md                  # Step 1调试记录
PHASE6_NAVIGATION_GUIDE.md             # 本文档
```

**Phase 6旧文档** (历史记录):
```
PHASE_6_DESIGN.md                      # V1设计
PHASE_6_PROGRESS.md                    # V2进度
PHASE6_KINODYNAMIC_UPGRADE.md          # V3升级方案
PHASE6_FINAL_SUMMARY.md                # V3总结
```

**Phase 6代码**:
```
wheeled_ur5e_aligator_mpc/
└── full_dynamic_mpc_controller.py     # Step 1代码 (90%)
```

---

## 🚀 快速开始

### 如果你是新来的
1. 读 `PHASE6_WORK_SUMMARY.md`
2. 读 `PHASE6_NEW_DESIGN.md`
3. 决定采用哪个方案 (A/B/C)

### 如果你要继续开发
1. 检查当前在哪个步骤 (目前在Step 1)
2. 读对应的文档
3. 查看代码状态
4. 根据方案执行

### 如果遇到问题
1. 查看 `PHASE6_STEP1_DEBUG.md`
2. 查看 `PHASE6_DIAGNOSIS_REPORT.md`
3. 参考历史文档了解已知问题

---

## 💡 推荐的阅读顺序

**第一次接触Phase 6**:
1. `PHASE6_WORK_SUMMARY.md` (15分钟)
2. `PHASE6_NEW_DESIGN.md` (30分钟)
3. `PHASE_4_CONCLUSION.md` (了解历史，15分钟)

**准备继续开发**:
1. `PHASE6_WORK_SUMMARY.md` → 决定方案
2. 如果选方案B: `aligator_problem.py` (Phase 1-3)
3. 如果选方案A/C: `PHASE6_STEP1_DEBUG.md`

**调试问题**:
1. `PHASE6_STEP1_DEBUG.md` → 具体修复步骤
2. `PHASE6_DIAGNOSIS_REPORT.md` → 问题诊断
3. Phase 5测试: `test_wheeled_dynamics.py`

---

## 📞 关键联系人

**项目位置**: `/home/ldq/spirita-work/mobile-manipulator/aligator/study_example/wheeled_ur5e_aligator_mpc`

**环境**: pixi环境 `all`

**依赖**:
- ALIGATOR 0.19.0 (从源码构建)
- Pinocchio 3.9+
- MuJoCo 3.x

**测试命令**:
```bash
cd /home/ldq/spirita-work/mobile-manipulator/aligator/study_example/wheeled_ur5e_aligator_mpc
eval "$(pixi shell-hook -e all)"

# Phase 5测试
python -m pytest tests/test_wheeled_dynamics.py -v

# Phase 6测试 (目前崩溃)
PYTHONPATH=/home/ldq/spirita-work/mobile-manipulator/aligator/build/bindings/python:. \
python wheeled_ur5e_aligator_mpc/full_dynamic_mpc_controller.py
```

---

**最后更新**: 2026-06-25 20:20  
**维护者**: Claude  
**版本**: 1.0
