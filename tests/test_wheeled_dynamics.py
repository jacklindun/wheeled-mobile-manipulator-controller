"""
Phase 5测试套件：轮子动力学

测试内容：
1. 轮子动力学计算正确性
2. 差速驱动运动学
3. 非完整约束
4. 简单闭环运动
"""

import sys
from pathlib import Path
import numpy as np
import pytest

# 添加路径
_project_root = Path(__file__).resolve().parents[1]
sys.path[:0] = [
    str(_project_root.parents[1] / "build" / "bindings" / "python"),
]

from wheeled_ur5e_aligator_mpc.wheeled_dynamics import (
    WheeledUR5eDynamics,
    WheelParameters,
    NonholonomicConstraint,
    inverse_diff_drive,
)
from wheeled_ur5e_aligator_mpc.pinocchio_model import PinocchioWheeledUR5eModel


@pytest.fixture
def dynamics():
    """创建轮子动力学实例"""
    pin_robot = PinocchioWheeledUR5eModel()
    wheel_params = WheelParameters()
    return WheeledUR5eDynamics(pin_robot, dt=0.05, wheel_params=wheel_params)


@pytest.fixture
def nominal_state():
    """创建名义状态"""
    x = np.zeros(23)
    x[2] = 0.2  # base_z
    x[6:12] = [3.14159265, 1.04719755, -1.57079633, 0.52359878, 0., 0.]
    return x


def test_initialization(dynamics):
    """测试初始化"""
    assert dynamics.nu == 8
    assert dynamics.space.nx == 23


def test_forward_straight(dynamics, nominal_state):
    """测试直线前进"""
    x0 = nominal_state.copy()
    u = np.zeros(8)
    u[0] = u[1] = 5.0  # 相等扭矩

    data = dynamics.createData()
    dynamics.forward(x0, u, data)

    # 轮速应该增加
    assert data.xnext[15] > 0  # ω_left
    assert data.xnext[16] > 0  # ω_right
    assert np.abs(data.xnext[15] - data.xnext[16]) < 1e-10  # 相等

    # 基座应该前进
    assert data.xnext[0] > x0[0]  # x增加
    assert np.abs(data.xnext[1] - x0[1]) < 1e-10  # y不变


def test_forward_spin(dynamics, nominal_state):
    """测试原地旋转"""
    x0 = nominal_state.copy()
    u = np.zeros(8)
    u[0] = -5.0  # 左轮反向
    u[1] = 5.0   # 右轮正向

    data = dynamics.createData()
    dynamics.forward(x0, u, data)

    # 轮速应该反向
    assert data.xnext[15] < 0  # ω_left
    assert data.xnext[16] > 0  # ω_right

    # 基座应该旋转，不平移
    assert np.abs(data.xnext[0] - x0[0]) < 0.01  # x基本不变
    assert np.abs(data.xnext[1] - x0[1]) < 0.01  # y基本不变
    assert data.xnext[3] != x0[3]  # yaw改变


def test_diff_drive_kinematics(dynamics):
    """测试差速驱动运动学"""
    # 直线
    v, ω = dynamics._diff_drive_kinematics(np.array([10.0, 10.0]))
    assert np.abs(v - 1.0) < 1e-6
    assert np.abs(ω) < 1e-6

    # 旋转
    v, ω = dynamics._diff_drive_kinematics(np.array([-10.0, 10.0]))
    assert np.abs(v) < 1e-6
    assert ω > 0


def test_inverse_diff_drive():
    """测试差速驱动逆运动学"""
    params = WheelParameters()

    # 直线
    ω_left, ω_right = inverse_diff_drive(1.0, 0.0, params)
    assert np.abs(ω_left - 10.0) < 1e-6
    assert np.abs(ω_right - 10.0) < 1e-6

    # 旋转
    ω_left, ω_right = inverse_diff_drive(0.0, 4.0, params)
    assert np.abs(ω_left + 10.0) < 1e-6
    assert np.abs(ω_right - 10.0) < 1e-6


def test_nonholonomic_constraint_zero():
    """测试非完整约束：直线运动"""
    x = np.zeros(23)
    x[3] = 0.0  # yaw = 0
    x[12] = 1.0  # vx_world = 1
    x[13] = 0.0  # vy_world = 0

    residual = NonholonomicConstraint.compute_residual(x)
    assert np.abs(residual) < 1e-10


def test_nonholonomic_constraint_rotated():
    """测试非完整约束：旋转坐标系"""
    x = np.zeros(23)
    x[3] = np.pi / 4  # yaw = 45度
    # 在世界坐标系中沿yaw方向移动
    x[12] = np.cos(np.pi / 4)
    x[13] = np.sin(np.pi / 4)

    residual = NonholonomicConstraint.compute_residual(x)
    # 在body frame中应该是纯前进，无侧向
    assert np.abs(residual) < 1e-10


def test_nonholonomic_constraint_violation():
    """测试非完整约束：违反约束"""
    x = np.zeros(23)
    x[3] = 0.0
    x[12] = 0.0
    x[13] = 1.0  # 纯侧向运动（违反约束）

    residual = NonholonomicConstraint.compute_residual(x)
    assert np.abs(residual - 1.0) < 1e-10


def test_jacobian_shape():
    """测试雅可比矩阵形状"""
    x = np.zeros(23)
    jac = NonholonomicConstraint.compute_jacobian(x)
    assert jac.shape == (23,)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
