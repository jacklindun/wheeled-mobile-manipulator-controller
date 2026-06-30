# Phase 6 当前状态

**更新**: 2026-06-26

---

## 推荐方案

| 场景 | 推荐版本 | RMS (实测) | 说明 |
|------|----------|------------|------|
| 单臂轮式 UR5e 全身跟踪 | **v2** | ~1.8–2.5 cm | 运动学 MPC + 500 Hz 前馈 PD |
| 双臂力矩圆形跟踪 | **v3 Step 1** | **2.28 cm** | IK + 重力前馈 + 力矩 PD |
| 双臂动力学 MPC 在线优化 | v3 Step 2 | 3.56 cm (研究) | 预测精度高，闭环略差于 Step 1 |

**不推荐**: v1 (MPC+WBC)、v3 旧版 EE-only 动力学 MPC (0% 收敛)。

---

## 版本对比

### v1 — MPC + WBC

- 架构: Kino-Dynamic MPC (20 Hz) → WBC QP (100 Hz)
- 问题: 欠驱动、动力学残差大
- 状态: 代码完成，不作为主路径

### v2 — 运动学 MPC + 前馈 PD ⭐

```
运动学 MPC (20 Hz) → 插值 (25:1) → 前馈 PD (500 Hz) → MuJoCo position actuator
```

- 基于 Phase 1–3 baseline (1.83 cm, 100% 收敛)
- 避免 Phase 4 积分器不匹配
- 详见 [V2.md](V2.md)

### v3 — 双臂力矩控制

```
Step 1 (生产): IK (20 Hz) → 插值 → 重力前馈 + PD → motor actuator
Step 2 (研究): IK-informed 动力学 MPC (h=5) → 插值 → 前馈 PD → motor actuator
```

- Step 1 突破关键: **重力前馈** `τ_g(q_des)`（16 cm → 2.3 cm）
- Step 2: IK 关节参考代价 + RNEA warm start；`conv=False` 但预测 EE < 0.01 cm，用 `is_mpc_solution_usable()` 判定
- 详见 [V3.md](V3.md)

---

## 性能演进 (v3)

| 阶段 | Step 1 RMS | Step 2 RMS | 备注 |
|------|-------------|-------------|------|
| 初始 | — | 35.6 cm | 插值/坐标错误 |
| 结构修复后 | — | 18.6 cm | 仍 EE-only MPC |
| 重力前馈 + IK-MPC | **2.28 cm** | 3.56 cm | 2026-06-26 当前 |

---

## 已知限制

1. v3 文档早期写的「1.79 cm」是硬编码对比值，非实测 RMS
2. v3 Step 2 严格 KKT 收敛率仍为 0%，需用 EE 质量门控评估
3. v2 与 v3 机器人/执行器模型不同，数值不可直接横比

---

## 相关 Phase

- **Phase 1–3**: v2 的运动学 MPC 基础
- **Phase 4**: 混合动力学失败（积分器不匹配）→ 促成 v2 设计
- **Phase 5**: 轮子动力学，v1 WBC 参考
- **Phase 7**: 双臂扩展，与 v3 重叠

全局路线图见项目根目录 `ROADMAP.md`。