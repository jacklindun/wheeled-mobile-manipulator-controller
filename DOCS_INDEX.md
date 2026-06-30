# 文档索引

**更新日期**: 2026-06-26

本项目有34个markdown文档，按功能分类整理如下：

---

## 📖 核心文档（必读）

### 1. 项目概览
- **README.md** - 项目介绍和快速开始
- **ROADMAP.md** - 技术路线图

### 2. 最新进展（2026-06-25）
- **FINAL_SUMMARY.md** - 最终总结报告 ⭐推荐阅读
  - Phase 6两版本对比
  - Phase 4修复方案
  - 性能对比和推荐方案

### 3. 技术指南
- **SKILLS_GUIDE.md** - 技能使用指南
- **TUNING_GUIDE.md** - 参数调优指南

---

## 🔬 Phase开发文档

### Phase 1-3: 运动学MPC（已完成✅）
- **PHASE_1-4_SUMMARY.md** - Phase 1-4总结
- 性能: 1.83cm误差，100%收敛率

### Phase 4: 混合动力学MPC（有问题❌）
- **PHASE_4_CONCLUSION.md** - Phase 4总结和问题分析
- 问题: 积分器不匹配，0%收敛率，2.5-5.0cm误差
- 修复: 见FINAL_SUMMARY.md

### Phase 5: 物理轮子动力学（已完成✅）
- **PHASE_5_SUMMARY.md** - Phase 5完成总结
- **PHASE_5_DESIGN.md** - Phase 5设计文档

### Phase 6: 高频全身控制（v2/v3）⭐
- **[PHASE6.md](PHASE6.md)** - 根目录简短指针
- **[docs/phase6/README.md](docs/phase6/README.md)** - **Phase 6 文档中心（主入口）**
- **[docs/phase6/STATUS.md](docs/phase6/STATUS.md)** - 当前状态与版本对比
- **[docs/phase6/V2.md](docs/phase6/V2.md)** - v2 运动学 MPC + 前馈 PD
- **[docs/phase6/V3.md](docs/phase6/V3.md)** - v3 双臂力矩控制
- **[docs/phase6/ARCHIVE_INDEX.md](docs/phase6/ARCHIVE_INDEX.md)** - 历史文档归档索引

### Phase 7: 双臂控制（规划中）
- **PHASE_7_*.md** - Phase 7相关文档

---

## 📊 测试报告

### 最新测试（2026-06-25）
- **FINAL_SUMMARY.md** - 最终总结 ⭐
- **PHASE6_COMPARISON_REPORT.md** - Phase 6对比
- **FINAL_TEST_REPORT.md** - 完整测试报告
- **EXECUTIVE_SUMMARY.md** - 执行摘要
- **QUICK_SUMMARY.md** - 快速总结

### 历史测试
- **TEST_SUMMARY_REPORT.md** - 测试总结
- **EXECUTION_SUMMARY.md** - 执行总结

---

## 🔧 模型相关

- **MODEL_V2_*.md** - 模型V2迁移和修复相关
- **MIRROR_SYMMETRY_CONTROL_GUIDE.md** - 镜像对称控制指南

---

## 📝 开发记录（归档）

以下文档记录了开发过程，可归档：
- PHASE_6_DESIGN.md
- PHASE_6_PROGRESS.md
- PHASE6_KINODYNAMIC_UPGRADE.md
- PHASE6_MODEL_FILES_GUIDE.md
- PHASE6_IMPLEMENTATION_PROGRESS.md
- PROGRESS.md
- 各种 *_REPORT.md, *_PROGRESS.md 文件

---

## 🎯 快速导航

**我想了解...**

1. **项目整体情况** → README.md + FINAL_SUMMARY.md
2. **最佳方案是什么** → docs/phase6/STATUS.md（v2 单臂 / v3 Step1 双臂）
3. **Phase 6有几个版本** → docs/phase6/STATUS.md
4. **Phase 4为什么失败** → PHASE_4_CONCLUSION.md
5. **如何调参** → TUNING_GUIDE.md
6. **项目路线图** → ROADMAP.md

---

## 📦 建议的文档整理方案

### 保留在根目录（核心文档）
```
README.md                          - 项目介绍
PHASE6.md                          - Phase 6 指针 → docs/phase6/
FINAL_SUMMARY.md                   - 项目级总结
PHASE_4_CONCLUSION.md              - Phase 4问题分析
PHASE_5_SUMMARY.md                 - Phase 5总结
ROADMAP.md                         - 路线图
TUNING_GUIDE.md                    - 调优指南
SKILLS_GUIDE.md                    - 技能指南
DOCS_INDEX.md                      - 本文档索引
```

### Phase 6 专题（已集中到 docs/phase6/）
```
docs/phase6/README.md              - 主入口
docs/phase6/STATUS.md              - 当前状态
docs/phase6/V2.md, V3.md           - 版本指南
docs/phase6/archive/               - 历史过程文档
```

### docs/phases/（其他 Phase 开发文档）
```
PHASE_1-4_SUMMARY.md
PHASE_5_DESIGN.md
PHASE_6_*.md (除PHASE6_COMPARISON_REPORT.md外)
PHASE_7_*.md
PHASE6_*.md
```

### 移动到 docs/reports/（测试报告）
```
FINAL_TEST_REPORT.md
EXECUTIVE_SUMMARY.md
QUICK_SUMMARY.md
TEST_SUMMARY_REPORT.md
EXECUTION_SUMMARY.md
```

### 移动到 docs/archive/（历史文档）
```
PROGRESS.md
MODEL_V2_*.md
MIRROR_SYMMETRY_CONTROL_GUIDE.md
项目整体规划.md
其他过时的开发记录
```

---

**整理后**: 根目录仅保留9个核心文档，其余25个文档分类归档。
