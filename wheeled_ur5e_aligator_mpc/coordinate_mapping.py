"""
坐标顺序定义和映射

Pinocchio/ALIGATOR 使用的 qpos 顺序（Q_ORDER）：
  [base_x, base_y, base_yaw, base_z, left_arm_joints(6), right_arm_joints(6)]

MuJoCo actuator ctrl 顺序（CTRL_ORDER）：
  [base_x, base_y, base_z, base_yaw, left_arm_joints(6), right_arm_joints(6)]

关键差异：base 的 yaw 和 z 位置互换
"""

import numpy as np

# qpos 到 ctrl 的索引映射
Q_TO_CTRL = np.array([
    0, 1, 3, 2,  # base: x, y, yaw->z, z->yaw
    4, 5, 6, 7, 8, 9,  # left arm (unchanged)
    10, 11, 12, 13, 14, 15  # right arm (unchanged)
], dtype=int)

# ctrl 到 qpos 的索引映射（逆映射）
CTRL_TO_Q = np.array([
    0, 1, 3, 2,  # base: x, y, z->yaw, yaw->z
    4, 5, 6, 7, 8, 9,  # left arm
    10, 11, 12, 13, 14, 15  # right arm
], dtype=int)


def q_to_ctrl(q_order_array: np.ndarray) -> np.ndarray:
    """将 Pinocchio/qpos 顺序转换为 MuJoCo ctrl 顺序"""
    return q_order_array[Q_TO_CTRL]


def ctrl_to_q(ctrl_order_array: np.ndarray) -> np.ndarray:
    """将 MuJoCo ctrl 顺序转换为 Pinocchio/qpos 顺序"""
    return ctrl_order_array[CTRL_TO_Q]


# 正确的 nominal base 配置 [x, y, yaw, z]
BASE_NOMINAL_Q = np.array([0.0, 0.0, 0.0, 0.2])

# Base PD gains（按 qpos 顺序：x, y, yaw, z）
BASE_GAINS_KP_Q = np.array([200.0, 200.0, 100.0, 1000.0])
BASE_GAINS_KD_Q = np.array([50.0, 50.0, 20.0, 200.0])

# Torque limits（按 qpos 顺序：x, y, yaw, z）
BASE_TAU_MAX_Q = np.array([200.0, 200.0, 100.0, 1000.0])

# 双臂 nominal 配置 [base(4), left_arm(6), right_arm(6)]
DUAL_ARM_Q_NOMINAL = np.array([
    0.0, 0.0, 0.0, 0.2,
    -2.5434, -0.6884,  1.6850, 0.4209, -1.3484,  0.0000,
     1.4529, -0.7472,  2.3605, 0.3727, -1.9646,  0.0000,
])

# 双臂力矩限幅（qpos 顺序）
DUAL_ARM_TAU_MAX_Q = np.array([
    *BASE_TAU_MAX_Q,
    150, 150, 150, 28, 28, 28,
    150, 150, 150, 28, 28, 28,
])
