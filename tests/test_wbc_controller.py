"""
Phase 6测试：WBC控制器

测试WBC的QP求解功能
"""

import sys
from pathlib import Path
import numpy as np
import pytest

_project_root = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(_project_root.parents[1] / "build" / "bindings" / "python")]

from wheeled_ur5e_aligator_mpc.wbc_controller import WholeBodyController, WBCWeights
from wheeled_ur5e_aligator_mpc.pinocchio_model import PinocchioWheeledUR5eModel
from wheeled_ur5e_aligator_mpc.wheeled_dynamics import WheelParameters


@pytest.fixture
def wbc():
    """创建WBC实例"""
    pin_robot = PinocchioWheeledUR5eModel()
    wheel_params = WheelParameters()
    weights = WBCWeights()
    return WholeBodyController(pin_robot, wheel_params, weights)


@pytest.fixture
def nominal_state():
    """创建名义状态（23-dim）"""
    x = np.zeros(23)
    x[2] = 0.2  # base_z
    x[3] = 0.0  # yaw
    x[6:12] = [3.14159265, 1.04719755, -1.57079633, 0.52359878, 0., 0.]  # arm
    return x


def test_wbc_initialization(wbc):
    """测试WBC初始化"""
    assert wbc._n_vars == 19
    assert wbc._n_eq == 11
    assert wbc._weights.w_dynamics == 1000.0


def test_compute_dynamics(wbc, nominal_state):
    """测试动力学计算"""
    q = nominal_state[:12]
    v = nominal_state[12:]

    M, h = wbc._compute_dynamics(q, v)

    # 检查形状
    assert M.shape == (11, 11)
    assert h.shape == (11,)

    # 质量矩阵应该正定
    eigvals = np.linalg.eigvals(M)
    assert np.all(eigvals > 0), "质量矩阵不是正定的"

    # 重力项应该非零（机械臂有重力）
    assert np.linalg.norm(h) > 0


def test_s_matrix(wbc):
    """测试选择矩阵"""
    S = wbc._S_matrix()

    assert S.shape == (11, 8)

    # 轮子扭矩映射
    assert S[3, 0] == 1.0  # left wheel
    assert S[4, 1] == 1.0  # right wheel

    # 机械臂扭矩映射
    assert np.allclose(S[5:11, 2:8], np.eye(6))


def test_wbc_zero_acceleration(wbc, nominal_state):
    """测试WBC跟踪零加速度"""
    a_des = np.zeros(11)

    τ_opt, info = wbc.compute_control(nominal_state, a_des)

    # 检查输出
    assert τ_opt.shape == (8,)
    assert "solve_time_ms" in info
    assert "dynamics_residual" in info

    # 求解时间应该很快
    assert info["solve_time_ms"] < 10.0, f"求解太慢: {info['solve_time_ms']:.2f} ms"

    # 动力学残差应该小
    assert info["dynamics_residual"] < 0.1, f"动力学残差太大: {info['dynamics_residual']:.6f}"

    print(f"\n  求解时间: {info['solve_time_ms']:.3f} ms")
    print(f"  动力学残差: {info['dynamics_residual']:.6e}")
    print(f"  最优扭矩: {τ_opt}")


def test_wbc_nonzero_acceleration(wbc, nominal_state):
    """测试WBC跟踪非零加速度"""
    # 期望：基座前进，机械臂第一个关节转动
    a_des = np.zeros(11)
    a_des[0] = 1.0  # 基座x加速度
    a_des[5] = 0.5  # 机械臂第一关节加速度

    τ_opt, info = wbc.compute_control(nominal_state, a_des)

    # 求解成功
    assert info["solve_time_ms"] < 10.0
    assert info["dynamics_residual"] < 0.5

    # 扭矩应该非零
    assert np.linalg.norm(τ_opt) > 0

    print(f"\n  期望加速度: {a_des}")
    print(f"  最优扭矩: {τ_opt}")
    print(f"  实际加速度: {info['a_optimal']}")


def test_wbc_torque_limits(wbc, nominal_state):
    """测试扭矩限制"""
    # 期望非常大的加速度
    a_des = np.full(11, 10.0)

    τ_opt, info = wbc.compute_control(nominal_state, a_des)

    # 扭矩应该在限制内
    assert np.all(τ_opt[:2] >= -10.0) and np.all(τ_opt[:2] <= 10.0), "轮子扭矩超限"
    assert τ_opt[2] >= -150 and τ_opt[2] <= 150, "肩关节扭矩超限"
    assert τ_opt[7] >= -28 and τ_opt[7] <= 28, "腕关节扭矩超限"


def test_wbc_smoothness(wbc, nominal_state):
    """测试扭矩平滑性"""
    a_des = np.zeros(11)
    a_des[5] = 1.0

    # 第一步
    τ_1, _ = wbc.compute_control(nominal_state, a_des)

    # 第二步（相同的期望）
    τ_2, _ = wbc.compute_control(nominal_state, a_des, τ_prev=τ_1)

    # 扭矩应该相似（平滑项起作用）
    diff = np.linalg.norm(τ_2 - τ_1)
    assert diff < 5.0, f"扭矩变化太大: {diff:.2f}"

    print(f"\n  τ_1: {τ_1}")
    print(f"  τ_2: {τ_2}")
    print(f"  差异: {diff:.3f}")


def test_wbc_consistency(wbc, nominal_state):
    """测试多次调用的一致性"""
    a_des = np.random.randn(11) * 0.1

    τ_1, info_1 = wbc.compute_control(nominal_state, a_des)
    τ_2, info_2 = wbc.compute_control(nominal_state, a_des)

    # 相同输入应该产生相同输出
    assert np.allclose(τ_1, τ_2, atol=1e-6), "WBC输出不一致"


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
