# Phase 6-v2 Quick Start Guide

**最优单臂轮式机械臂控制方案**

## 性能指标

✅ **稳态RMS**: 1.50 cm  
✅ **启动RMS**: 2.20 cm (0-3s)  
✅ **全程RMS**: 1.75 cm  
✅ **收敛率**: 99.7%  
✅ **控制频率**: 500 Hz  

---

## 快速运行

### 1. 基础运行（10秒，无可视化）
```bash
cd wheeled_ur5e_aligator_mpc
export PYTHONPATH=.:../../build/bindings/python
pixi run -e all python scripts/demo_phase6_v2_optimized.py
```

### 2. 带可视化运行
```bash
pixi run -e all python scripts/demo_phase6_v2_optimized.py --render
```

### 3. 自定义场景和时长
```bash
# 直线运动，15秒
pixi run -e all python scripts/demo_phase6_v2_optimized.py --scenario ee_line --duration 15

# 基座运动，20秒
pixi run -e all python scripts/demo_phase6_v2_optimized.py --scenario base_and_ee --duration 20
```

---

## 关键配置

### MPC参数
```python
mpc_dt = 0.025          # 40 Hz (2倍于baseline)
horizon = 20            # 0.5秒预测
max_iters = 10

weights = {
    'ee_pos': 300.0,           # 末端位置 (3倍)
    'terminal_ee_pos': 600.0,  # 终端位置 (3倍)
    'base_xy': 100.0,          # 基座xy (1.67倍)
    'base_z': 100.0,           # 基座z (1.67倍)
}
```

### 自适应PD增益
```python
# 0-3秒: 高增益快速响应
Kp_arm = 3600  (2倍正常值)

# 3-6秒: 线性过渡
Kp_arm: 3600 → 1800

# 6秒+: 正常增益
Kp_arm = 1800
```

### 参考轨迹
- **平滑启动**: 2秒立方缓动 (cubic ease-in)
- **避免突变**: 速度从0平滑加速

---

## 可用场景

| 场景 | 描述 | 推荐时长 |
|------|------|---------|
| `ee_circle` | 末端画圆（默认） | 10s |
| `ee_line` | 末端直线运动 | 15s |
| `base_and_ee` | 基座前进，末端保持 | 20s |
| `base_z_test` | 升降测试 | 15s |

---

## 性能对比

| 版本 | RMS误差 | 收敛率 | 特点 |
|------|---------|--------|------|
| Phase 1-3 (baseline) | 1.83 cm | 100% | 20Hz，稳定 |
| **Phase 6-v2 (优化)** | **1.75 cm** | **99.7%** | **40Hz，启动优化** |
| Phase 6-v3 (双臂) | 2.28 cm | N/A | 双臂力矩控制 |

---

## 技术亮点

1. **MPC频率翻倍**: 20Hz → 40Hz，预测更及时
2. **启动误差减半**: 平滑启动 + 自适应增益，峰值误差 -48%
3. **高频控制**: 500Hz PD控制，轨迹跟踪平滑
4. **鲁棒稳定**: 避免Phase 4的积分器不匹配问题

---

## 故障排除

### 问题1: C++绑定错误
```
terminate called after throwing an instance of 'boost::python::error_already_set'
```

**解决**: 已修复，使用位置跟踪代替姿态跟踪（避免Pinocchio对象deepcopy）

### 问题2: MPC求解慢
```
MPC solve time > 100 ms
```

**正常**: 40Hz MPC的求解时间约85ms，已优化。可降低horizon或使用code-gen MPC。

### 问题3: 启动误差大
```
0-1s误差 > 4 cm
```

**已优化**: 使用组合策略后降至2.62cm。确保使用`demo_phase6_v2_optimized.py`。

---

## 更多信息

- **完整报告**: [PHASE6_V2_PERFORMANCE_REPORT.md](../PHASE6_V2_PERFORMANCE_REPORT.md)
- **调优结果**: [results/phase6_v2_tuning/](../results/phase6_v2_tuning/)
- **项目文档**: [README.md](../README.md)
- **Phase 6总览**: [docs/phase6/README.md](../docs/phase6/README.md)

---

**最后更新**: 2026-06-30  
**推荐用于**: 单臂轮式移动操作，位置控制器，平滑轨迹跟踪
