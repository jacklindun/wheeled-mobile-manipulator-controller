"""
Phase 6: MPC-WBC接口 (升级版 - 使用Kino-Dynamic MPC)

负责：
1. 状态空间转换（16-dim Kino-Dynamic MPC ↔ 23-dim WBC）
2. 从MPC轨迹直接获取准确的加速度（从ABA计算）
3. 频率同步（MPC 20Hz, WBC 100Hz）

升级说明：
- 旧版：使用10-dim运动学MPC，通过差分估计加速度 ❌
- 新版：使用16-dim kino-dynamic MPC，从ABA获取准确加速度 ✅
"""

import numpy as np


class MPCWBCInterface:
    """
    MPC和WBC之间的数据接口 (Kino-Dynamic版本)

    MPC输出：16-dim状态 [q_base(4), q_arm(6), v_arm(6)]
    WBC输入：23-dim状态 [q_base(4), θ_wheels(2), q_arm(6), v_base(3), ω_wheels(2), v_arm(6)]

    关键改进：MPC轨迹包含准确的加速度（从ABA计算），WBC无需"猜测"动力学！
    """

    def __init__(self, kinodynamic_mpc, wbc_dt=0.01):
        """
        Parameters
        ----------
        kinodynamic_mpc : KinoDynamicMPCController
            Kino-dynamic MPC控制器
        wbc_dt : float
            WBC时间步长（100Hz = 0.01s）
        """
        self.mpc = kinodynamic_mpc
        self.mpc_dt = kinodynamic_mpc.dt
        self.wbc_dt = wbc_dt

        # MPC轨迹缓存 (包含准确的加速度！)
        self.mpc_trajectory = None  # {"xs": [...], "us": [...], "accelerations": [...]}
        self.last_mpc_time = -1.0

    def kinodynamic_state_to_wbc(self, x_kinodyn, x_wbc_prev):
        """
        16-dim Kino-dynamic状态 → 23-dim WBC状态

        Parameters
        ----------
        x_kinodyn : (16,) array
            Kino-dynamic状态 [q_base(4), q_arm(6), v_arm(6)]
        x_wbc_prev : (23,) array
            上一时刻的WBC状态 (用于轮子状态)

        Returns
        -------
        x_wbc : (23,) array
            WBC状态 [q_base(4), θ_wheels(2), q_arm(6), v_base(3), ω_wheels(2), v_arm(6)]
        """
        x_wbc = np.zeros(23)

        # 位置部分
        x_wbc[0:4] = x_kinodyn[0:4]      # q_base
        x_wbc[4:6] = x_wbc_prev[4:6]      # θ_wheels (保持或更新)
        x_wbc[6:12] = x_kinodyn[4:10]    # q_arm

        # 速度部分
        # v_base需要从MPC控制中获取 (暂时保持上一时刻值)
        x_wbc[12:15] = x_wbc_prev[12:15]  # v_base (world frame)
        x_wbc[15:17] = x_wbc_prev[15:17]  # ω_wheels
        x_wbc[17:23] = x_kinodyn[10:16]   # v_arm

        return x_wbc

    def get_desired_acceleration_from_mpc(self, t):
        """
        ✅ 从MPC轨迹直接获取准确的加速度

        这是关键改进：不再使用差分估计 a = (v_des - v_current) / dt
        而是使用MPC通过ABA计算的准确加速度！

        Parameters
        ----------
        t : float
            当前时间

        Returns
        -------
        a_des : (11,) array
            期望加速度 [a_base(3), α_wheels(2), a_arm(6)]
            如果没有MPC轨迹，返回零
        """
        if self.mpc_trajectory is None or 'accelerations' not in self.mpc_trajectory:
            return np.zeros(11)

        # 计算在MPC轨迹中的索引
        t_mpc = t - self.last_mpc_time
        idx = int(t_mpc / self.mpc_dt)

        if idx < 0 or idx >= len(self.mpc_trajectory['accelerations']):
            idx = min(max(0, idx), len(self.mpc_trajectory['accelerations']) - 1)

        # ✅ 直接返回MPC计算的准确加速度 (包含完整动力学信息)
        a_des = self.mpc_trajectory['accelerations'][idx]

        return a_des

    def update_mpc_trajectory(self, mpc_trajectory, t):
        """
        更新MPC轨迹缓存（包含加速度）

        Parameters
        ----------
        mpc_trajectory : dict
            {
                "xs": (N+1, 16) array - 状态轨迹,
                "us": (N, 10) array - 控制轨迹,
                "accelerations": (N, 11) array - 加速度轨迹 ✅ 新增
            }
        t : float
            当前时间
        """
        self.mpc_trajectory = mpc_trajectory
        self.last_mpc_time = t

    def should_update_mpc(self, t):
        """
        判断是否需要更新MPC

        Parameters
        ----------
        t : float
            当前时间

        Returns
        -------
        should_update : bool
        """
        if self.last_mpc_time < 0:
            return True

        return (t - self.last_mpc_time) >= self.mpc_dt


def create_simple_reference_trajectory(scenario="circle", duration=10.0, dt=0.05):
    """
    创建简单的参考轨迹用于测试

    Parameters
    ----------
    scenario : str
        场景类型：'circle', 'line', 'stationary'
    duration : float
        时长
    dt : float
        时间步长

    Returns
    -------
    ref_traj : dict
        参考轨迹 {"ee_pos": (N, 3), "base": (N, 3), ...}
    """
    from .reference import ReferenceGenerator
    from .pinocchio_model import PinocchioWheeledUR5eModel
    from .robot_model import WheeledUR5eModel

    robot = WheeledUR5eModel()
    pin_robot = PinocchioWheeledUR5eModel()

    p_ee_nominal, R_ee_nominal = pin_robot.fk_pose(robot.q_nominal)

    ref_gen = ReferenceGenerator(
        scenario=scenario if scenario in ["ee_circle", "ee_line", "base_and_ee", "base_z_test"] else "ee_circle",
        ee_start=p_ee_nominal,
        ee_start_rot=R_ee_nominal
    )

    num_steps = int(duration / dt)
    ref_traj = ref_gen.get_reference(t=0.0, horizon=num_steps, dt=dt)

    return ref_traj
