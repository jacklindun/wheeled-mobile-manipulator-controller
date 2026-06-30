"""
Unit tests for DualArmPinocchioModel.

Tests dual FK, dual Jacobians, and validates against MuJoCo reference.
"""

import numpy as np
import pytest

# Import test utilities from existing tests
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wheeled_ur5e_aligator_mpc.dual_arm_pinocchio_model import DualArmPinocchioModel

try:
    import mujoco
except ImportError:
    pytest.skip("mujoco not available", allow_module_level=True)


@pytest.fixture
def model():
    """Fixture providing a DualArmPinocchioModel instance."""
    return DualArmPinocchioModel()


@pytest.fixture
def mujoco_model():
    """Fixture providing MuJoCo model for reference."""
    mjcf_path = Path(__file__).resolve().parents[1] / "assets" / "wheeled_dual_ur5e_v2.xml"
    return mujoco.MjModel.from_xml_path(str(mjcf_path))


class TestModelLoading:
    """Test that the dual-arm model loads correctly."""

    def test_model_dimensions(self, model):
        """Check DOF counts."""
        assert model.nq == 16
        assert model.nu == 16
        assert model.model.nq == 16

    def test_frame_ids_exist(self, model):
        """Check that both EE frames are found."""
        assert model.left_ee_frame_id >= 0
        assert model.right_ee_frame_id >= 0
        assert model.left_ee_frame_id != model.right_ee_frame_id

    def test_nominal_config(self, model):
        """Check nominal configuration shape and values."""
        q_nom = model.get_q_nominal()
        assert q_nom.shape == (16,)
        # Base at origin
        assert np.allclose(q_nom[:2], [0.0, 0.0])
        # Both arms have shoulder_pan = π
        assert np.isclose(q_nom[4], np.pi)
        assert np.isclose(q_nom[10], np.pi)


class TestForwardKinematics:
    """Test dual FK against MuJoCo reference."""

    def test_left_fk_at_nominal(self, model, mujoco_model):
        """Left EE FK at nominal config."""
        q = model.get_q_nominal()

        # Pinocchio FK
        p_pin = model.fk_left_ee(q)

        # MuJoCo FK
        data = mujoco.MjData(mujoco_model)
        data.qpos[:] = q
        mujoco.mj_forward(mujoco_model, data)
        left_site_id = mujoco.mj_name2id(mujoco_model, mujoco.mjtObj.mjOBJ_SITE, "left_ee_site")
        p_mj = data.site_xpos[left_site_id]

        # Should match within 1mm
        assert np.allclose(p_pin, p_mj, atol=1e-3), \
            f"Left FK mismatch: pin={p_pin}, mj={p_mj}"

    def test_right_fk_at_nominal(self, model, mujoco_model):
        """Right EE FK at nominal config."""
        q = model.get_q_nominal()

        # Pinocchio FK
        p_pin = model.fk_right_ee(q)

        # MuJoCo FK
        data = mujoco.MjData(mujoco_model)
        data.qpos[:] = q
        mujoco.mj_forward(mujoco_model, data)
        right_site_id = mujoco.mj_name2id(mujoco_model, mujoco.mjtObj.mjOBJ_SITE, "right_ee_site")
        p_mj = data.site_xpos[right_site_id]

        # Should match within 1mm
        assert np.allclose(p_pin, p_mj, atol=1e-3), \
            f"Right FK mismatch: pin={p_pin}, mj={p_mj}"

    def test_left_right_symmetry(self, model):
        """At nominal, left and right EE should be symmetric about y-axis."""
        q = model.get_q_nominal()
        p_left = model.fk_left_ee(q)
        p_right = model.fk_right_ee(q)

        # x and z should be identical
        assert np.isclose(p_left[0], p_right[0], atol=1e-3)
        assert np.isclose(p_left[2], p_right[2], atol=1e-3)

        # y should be opposite (left at +0.3, right at -0.3 shoulder offset)
        # Actual EE y depends on arm config, but should be symmetric
        assert np.isclose(p_left[1], -p_right[1], atol=0.05)

    def test_fk_with_pose(self, model):
        """Test fk_*_ee_pose returns both position and rotation."""
        q = model.get_q_nominal()

        p_left, R_left = model.fk_left_ee_pose(q)
        assert p_left.shape == (3,)
        assert R_left.shape == (3, 3)
        assert np.allclose(R_left @ R_left.T, np.eye(3), atol=1e-6)  # Orthogonal

        p_right, R_right = model.fk_right_ee_pose(q)
        assert p_right.shape == (3,)
        assert R_right.shape == (3, 3)
        assert np.allclose(R_right @ R_right.T, np.eye(3), atol=1e-6)


class TestJacobians:
    """Test dual Jacobian computation."""

    def test_left_jacobian_shape(self, model):
        """Left Jacobian should be (6, 16)."""
        q = model.get_q_nominal()
        J = model.jacobian_left_ee(q)
        assert J.shape == (6, 16)

        J_pos = model.position_jacobian_left_ee(q)
        assert J_pos.shape == (3, 16)
        assert np.allclose(J_pos, J[:3, :])

    def test_right_jacobian_shape(self, model):
        """Right Jacobian should be (6, 16)."""
        q = model.get_q_nominal()
        J = model.jacobian_right_ee(q)
        assert J.shape == (6, 16)

        J_pos = model.position_jacobian_right_ee(q)
        assert J_pos.shape == (3, 16)
        assert np.allclose(J_pos, J[:3, :])

    def test_left_jacobian_base_coupling(self, model):
        """Base motion (first 4 DOF) should affect left EE."""
        q = model.get_q_nominal()
        J_left = model.position_jacobian_left_ee(q)

        # Base_x and base_y should have non-zero effect
        assert np.abs(J_left[0, 0]) > 0.5  # base_x -> EE_x
        assert np.abs(J_left[1, 1]) > 0.5  # base_y -> EE_y

    def test_right_jacobian_independence(self, model):
        """Right arm joints should not affect left EE (and vice versa)."""
        q = model.get_q_nominal()

        J_left = model.position_jacobian_left_ee(q)
        J_right = model.position_jacobian_right_ee(q)

        # Right arm joints (cols 10-16) should have ~zero effect on left EE
        assert np.linalg.norm(J_left[:, 10:16]) < 1e-6, \
            "Right arm joints affecting left EE"

        # Left arm joints (cols 4-10) should have ~zero effect on right EE
        assert np.linalg.norm(J_right[:, 4:10]) < 1e-6, \
            "Left arm joints affecting right EE"

    def test_jacobian_finite_diff(self, model):
        """Verify left Jacobian with finite differences."""
        q = model.get_q_nominal()
        J = model.position_jacobian_left_ee(q)

        eps = 1e-7
        J_fd = np.zeros((3, 16))

        p0 = model.fk_left_ee(q)
        for i in range(16):
            q_plus = q.copy()
            q_plus[i] += eps
            p_plus = model.fk_left_ee(q_plus)
            J_fd[:, i] = (p_plus - p0) / eps

        # Should match within 1e-5
        assert np.allclose(J, J_fd, atol=1e-5), \
            f"Jacobian FD mismatch: max_error={np.max(np.abs(J - J_fd))}"


class TestUtilities:
    """Test utility methods."""

    def test_print_ee_positions(self, model, capsys):
        """Test debug print method."""
        q = model.get_q_nominal()
        model.print_ee_positions(q)

        captured = capsys.readouterr()
        assert "Left EE:" in captured.out
        assert "Right EE:" in captured.out

    def test_invalid_q_shape(self, model):
        """Test error handling for wrong q shape."""
        q_wrong = np.zeros(10)  # Should be 16

        with pytest.raises(ValueError, match="Expected q shape"):
            model.fk_left_ee(q_wrong)

        with pytest.raises(ValueError, match="Expected q shape"):
            model.jacobian_right_ee(q_wrong)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
