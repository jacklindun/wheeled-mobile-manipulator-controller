# Phase 6 Step 1 调试记录

**日期**: 2026-06-25  
**状态**: 调试中 - 遇到技术问题

---

## 问题总结

### 问题1: 文件路径混乱 ✅ 已解决
**现象**: full_dynamic_mpc_controller.py被创建在深层嵌套目录  
**解决**: 已移动到 `wheeled_ur5e_aligator_mpc/full_dynamic_mpc_controller.py`

### 问题2: deepcopy错误 🔄 进行中
**现象**:
```python
Boost.Python.ArgumentError: Python argument types in
    VectorSpace.__init__(VectorSpace)
did not match C++ signature:
    __init__(_object* self, int dim)
```

**根本原因**:
- `EEPositionCostFullDynamic.__reduce__()` 在deepcopy时传递了 `self.space` 对象
- ALIGATOR的C++绑定不支持直接传递space对象
- 需要传递维度int，然后在`__init__`中重建space

**解决方案**:
```python
# 修改1: __reduce__ 传递维度而非space对象
def __reduce__(self):
    return (
        self.__class__,
        (self.space.ndx, self._pin_robot, self._target_pos, self._weight),
    )

# 修改2: __init__ 支持接收int并创建space
def __init__(self, space_or_nx, pin_robot, target_pos, weight=1.0):
    if isinstance(space_or_nx, int):
        space = aligator.manifolds.VectorSpace(space_or_nx)
    else:
        space = space_or_nx
    super().__init__(space, nu)
    # ...
```

**状态**: 代码修改已写好，但遇到文件编辑工具路径问题

---

## 当前文件位置

**正确位置**:
```
/home/ldq/spirita-work/mobile-manipulator/aligator/
study_example/wheeled_ur5e_aligator_mpc/
└── wheeled_ur5e_aligator_mpc/
    └── full_dynamic_mpc_controller.py  ✅ 在这里
```

**文件大小**: 12,608 bytes  
**创建时间**: 2026-06-25 15:14

---

## 需要的修改

### 文件: wheeled_ur5e_aligator_mpc/full_dynamic_mpc_controller.py

**修改点1** (约第54行):
```python
# 原来:
def __init__(self, space, pin_robot, target_pos, weight=1.0):
    nu = 8
    super().__init__(space, nu)
    # ...

# 修改为:
def __init__(self, space_or_nx, pin_robot, target_pos, weight=1.0):
    nu = 8
    if isinstance(space_or_nx, int):
        space = aligator.manifolds.VectorSpace(space_or_nx)
    else:
        space = space_or_nx
    super().__init__(space, nu)
    # ...
```

**修改点2** (约第110行):
```python
# 原来:
def __reduce__(self):
    return (
        self.__class__,
        (self.space, self._pin_robot, self._target_pos, self._weight),
    )

# 修改为:
def __reduce__(self):
    return (
        self.__class__,
        (self.space.ndx, self._pin_robot, self._target_pos, self._weight),
    )
```

---

## 手动修复步骤

如果自动编辑失败，可以手动修复：

```bash
cd /home/ldq/spirita-work/mobile-manipulator/aligator/study_example/wheeled_ur5e_aligator_mpc

# 编辑文件
vim wheeled_ur5e_aligator_mpc/full_dynamic_mpc_controller.py

# 或者使用sed
sed -i 's/def __init__(self, space, pin_robot, target_pos, weight=1.0):/def __init__(self, space_or_nx, pin_robot, target_pos, weight=1.0):/' wheeled_ur5e_aligator_mpc/full_dynamic_mpc_controller.py

sed -i 's/(self.space, self._pin_robot, self._target_pos, self._weight),/(self.space.ndx, self._pin_robot, self._target_pos, self._weight),/' wheeled_ur5e_aligator_mpc/full_dynamic_mpc_controller.py
```

---

## 测试命令

修复后运行：

```bash
cd /home/ldq/spirita-work/mobile-manipulator/aligator/study_example/wheeled_ur5e_aligator_mpc

eval "$(pixi shell-hook -e all)"

PYTHONPATH=/home/ldq/spirita-work/mobile-manipulator/aligator/build/bindings/python:$PYTHONPATH \
python wheeled_ur5e_aligator_mpc/full_dynamic_mpc_controller.py
```

**预期输出**:
```
============================================================
Phase 6 Full Dynamic MPC 测试
============================================================

控制器参数:
  状态维度: 23
  控制维度: 8
  MPC horizon: 10
  MPC dt: 0.05 s

参考轨迹:
  场景: stationary
  时长: 2.0 s
  EE目标: [0.619 0.064 0.857]

求解MPC...

✓ MPC求解成功!
  求解时间: X.XXX s
  收敛: True/False
  迭代次数: XX
  轨迹长度: 11 步
  控制范围: τ∈[XX.XX, XX.XX] N·m

============================================================
```

---

## 下一步

修复完成后：

1. **验证MPC能求解** ✅
2. **检查输出轨迹格式** ✅
3. **记录性能数据**
4. **进入Step 2**: 创建插值器

---

## 备注

**工具限制**: Edit工具在当前会话中遇到路径问题  
**建议**: 用户手动执行上述sed命令完成修复  
**替代方案**: 重新启动会话或使用vim手动编辑

**总耗时**: Step 1预计2-3天，当前已用0.7天

---

**记录时间**: 2026-06-25 19:45  
**记录者**: Claude  
**状态**: 等待手动修复后继续
