#!/usr/bin/env python3
"""
Quick verification that all dual-arm code uses V2 model.

Run from wheeled_ur5e_aligator_mpc/ directory:
    pixi run -e all python scripts/verify_v2_migration.py
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import mujoco

try:
    import aligator
except ImportError:
    _repo_root = Path(__file__).resolve().parents[3]
    sys.path[:0] = [str(_repo_root / "build" / "bindings" / "python")]
    import aligator

from wheeled_ur5e_aligator_mpc.dual_arm_pinocchio_model import DualArmPinocchioModel


def test_pinocchio_model():
    """Test that Pinocchio model loads V2."""
    print("=" * 60)
    print("测试 1: Pinocchio模型加载")
    print("=" * 60)

    model = DualArmPinocchioModel()

    assert model.nq == 16, f"Expected nq=16, got {model.nq}"
    assert model.nu == 16, f"Expected nu=16, got {model.nu}"

    print(f"✓ nq: {model.nq}")
    print(f"✓ nu: {model.nu}")

    # Test FK
    q_nominal = model.get_q_nominal()
    p_left = model.fk_left_ee(q_nominal)
    p_right = model.fk_right_ee(q_nominal)

    print(f"✓ Left EE @ nominal: [{p_left[0]:.3f}, {p_left[1]:.3f}, {p_left[2]:.3f}]")
    print(f"✓ Right EE @ nominal: [{p_right[0]:.3f}, {p_right[1]:.3f}, {p_right[2]:.3f}]")

    # Expected positions for V2 model
    expected_left = np.array([0.180, 0.514, 1.895])
    expected_right = np.array([0.180, -0.046, 1.895])

    err_left = np.linalg.norm(p_left - expected_left)
    err_right = np.linalg.norm(p_right - expected_right)

    assert err_left < 0.01, f"Left EE position mismatch: {err_left:.4f}m"
    assert err_right < 0.01, f"Right EE position mismatch: {err_right:.4f}m"

    print("✓ FK位置与V2预期值匹配\n")


def test_mujoco_model():
    """Test that MuJoCo can load V2."""
    print("=" * 60)
    print("测试 2: MuJoCo模型加载")
    print("=" * 60)

    mjcf_path = Path(__file__).resolve().parents[1] / "assets" / "wheeled_dual_ur5e_v2.xml"

    assert mjcf_path.exists(), f"V2 model not found: {mjcf_path}"
    print(f"✓ 找到模型文件: {mjcf_path.name}")

    model = mujoco.MjModel.from_xml_path(str(mjcf_path))

    assert model.nq == 16, f"Expected nq=16, got {model.nq}"
    assert model.nu == 16, f"Expected nu=16, got {model.nu}"

    print(f"✓ nq: {model.nq}")
    print(f"✓ nu: {model.nu}")
    print(f"✓ nbody: {model.nbody} (V2增强的视觉几何)")
    print(f"✓ njnt: {model.njnt}")

    # Check critical sites exist
    left_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "left_ee_site")
    right_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "right_ee_site")

    assert left_site_id >= 0, "left_ee_site not found"
    assert right_site_id >= 0, "right_ee_site not found"

    print(f"✓ left_ee_site ID: {left_site_id}")
    print(f"✓ right_ee_site ID: {right_site_id}\n")


def test_fk_consistency():
    """Test that Pinocchio FK matches MuJoCo for V2 model."""
    print("=" * 60)
    print("测试 3: Pinocchio vs MuJoCo FK一致性")
    print("=" * 60)

    # Load both models
    pin_model = DualArmPinocchioModel()
    mjcf_path = Path(__file__).resolve().parents[1] / "assets" / "wheeled_dual_ur5e_v2.xml"
    mj_model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    mj_data = mujoco.MjData(mj_model)

    # Get site IDs
    left_site_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_SITE, "left_ee_site")
    right_site_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_SITE, "right_ee_site")

    # Test at nominal config
    q_nominal = pin_model.get_q_nominal()

    # Pinocchio FK
    p_left_pin = pin_model.fk_left_ee(q_nominal)
    p_right_pin = pin_model.fk_right_ee(q_nominal)

    # MuJoCo FK
    mj_data.qpos[:] = q_nominal
    mujoco.mj_forward(mj_model, mj_data)
    p_left_mj = mj_data.site_xpos[left_site_id].copy()
    p_right_mj = mj_data.site_xpos[right_site_id].copy()

    # Compare
    err_left = np.linalg.norm(p_left_pin - p_left_mj)
    err_right = np.linalg.norm(p_right_pin - p_right_mj)

    print(f"Left EE:")
    print(f"  Pinocchio: [{p_left_pin[0]:.6f}, {p_left_pin[1]:.6f}, {p_left_pin[2]:.6f}]")
    print(f"  MuJoCo:    [{p_left_mj[0]:.6f}, {p_left_mj[1]:.6f}, {p_left_mj[2]:.6f}]")
    print(f"  误差: {err_left*1000:.4f} mm")

    print(f"\nRight EE:")
    print(f"  Pinocchio: [{p_right_pin[0]:.6f}, {p_right_pin[1]:.6f}, {p_right_pin[2]:.6f}]")
    print(f"  MuJoCo:    [{p_right_mj[0]:.6f}, {p_right_mj[1]:.6f}, {p_right_mj[2]:.6f}]")
    print(f"  误差: {err_right*1000:.4f} mm")

    # V2 should maintain <1mm accuracy
    assert err_left < 0.001, f"Left EE FK mismatch: {err_left*1000:.2f}mm"
    assert err_right < 0.001, f"Right EE FK mismatch: {err_right*1000:.2f}mm"

    print(f"\n✓ FK一致性: 两臂误差均 < 1mm\n")


def main():
    print("\n" + "=" * 60)
    print("V2模型迁移验证")
    print("=" * 60 + "\n")

    try:
        test_pinocchio_model()
        test_mujoco_model()
        test_fk_consistency()

        print("=" * 60)
        print("✅ 所有测试通过！V2模型迁移成功")
        print("=" * 60)
        print("\n更新的文件:")
        print("  - wheeled_ur5e_aligator_mpc/dual_arm_pinocchio_model.py")
        print("  - scripts/demo_dual_arm_mpc.py")
        print("  - scripts/demo_dual_arm_fk.py")
        print("  - tests/test_dual_arm_pinocchio_model.py")
        print("\n详细报告: MODEL_V2_MIGRATION.md\n")

        return 0

    except Exception as e:
        print(f"\n❌ 测试失败: {e}\n")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
