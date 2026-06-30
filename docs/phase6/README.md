# Phase 6 文档中心

**更新**: 2026-06-26

Phase 6 探索「高频平滑全身控制」，历经 v1 (MPC+WBC)、v2 (运动学 MPC+前馈 PD)、v3 (双臂力矩控制) 三个方向。历史过程文档已归档，此处只保留**当前有效**的入口。

---

## 先读什么

| 你想… | 读这个 |
|--------|--------|
| 快速了解 Phase 6 全貌和推荐方案 | [STATUS.md](STATUS.md) |
| 跑单臂/轮式 UR5e 高频控制 (v2) | [V2.md](V2.md) |
| 跑双臂力矩控制 (v3) | [V3.md](V3.md) |
| 查历史诊断/进度/对比报告 | [ARCHIVE_INDEX.md](ARCHIVE_INDEX.md) |

---

## 版本一览

```
Phase 6-v1  MPC + WBC (动力学 QP)     → 代码完成，性能一般，不推荐
Phase 6-v2  运动学 MPC + 插值 + 前馈PD  → ⭐ 单臂推荐，~2 cm RMS
Phase 6-v3  双臂力矩 IK/动力学 MPC      → Step1 生产可用 ~2.3 cm；Step2 研究线
```

---

## 快速命令

```bash
cd study_example/wheeled_ur5e_aligator_mpc   # 或项目根目录
export PYTHONPATH=.:../../build/bindings/python

# v2 参考 demo
pixi run -e all python scripts/demo_phase6_v2_simple.py

# v3 双臂力矩 — 生产路径 (Step 1)
pixi run -e all python scripts/test_phase6_v3_step1_simple.py

# v3 双臂 — 双路径对比
pixi run -e all python scripts/test_phase6_v3_benchmark.py
```

---

## 核心代码

| 模块 | 路径 |
|------|------|
| v2 集成控制器 | `wheeled_ur5e_aligator_mpc/phase6_controller.py` |
| v2 插值器 | `wheeled_ur5e_aligator_mpc/trajectory_interpolator.py` |
| v2 前馈 PD | `wheeled_ur5e_aligator_mpc/feedforward_pd_controller.py` |
| v3 共享工具 | `wheeled_ur5e_aligator_mpc/phase6_v3_common.py` |
| v3 动力学 MPC | `wheeled_ur5e_aligator_mpc/dual_arm_dynamics_mpc.py` |
| v3 坐标映射 | `wheeled_ur5e_aligator_mpc/coordinate_mapping.py` |

---

## 文档结构

```
docs/phase6/
├── README.md           ← 你在这里
├── STATUS.md           ← 当前状态与版本对比
├── V2.md               ← v2 使用说明
├── V3.md               ← v3 使用说明
├── ARCHIVE_INDEX.md    ← 归档文档索引
└── archive/            ← 历史文档（勿删，仅供追溯）
```

项目根目录保留简短指针：[PHASE6.md](../../PHASE6.md)