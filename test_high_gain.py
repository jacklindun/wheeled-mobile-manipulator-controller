#!/usr/bin/env python3
"""快速测试：高增益版本"""
import sys
sys.path.insert(0, ".")
sys.path.insert(0, "../../build/bindings/python")

# 临时修改默认增益
from wheeled_ur5e_aligator_mpc import phase6_v3_common

# 保存原始函数
original_make_pd = phase6_v3_common.make_pd_controller

# 创建高增益版本
def high_gain_pd():
    from wheeled_ur5e_aligator_mpc.feedforward_pd_controller import FeedforwardPDController, FeedforwardPDGains
    gains = FeedforwardPDGains(
        Kp_base_xy=300.0, Kd_base_xy=60.0,
        Kp_base_z=2000.0, Kd_base_z=400.0,
        Kp_base_yaw=200.0, Kd_base_yaw=40.0,
        Kp_arm=2400.0, Kd_arm=240.0,  # 提升到Phase 6-v2的1.33倍
    )
    from wheeled_ur5e_aligator_mpc.phase6_v3_common import DUAL_ARM_TAU_MAX_Q
    controller = FeedforwardPDController(gains)
    controller.set_control_limits(-DUAL_ARM_TAU_MAX_Q, DUAL_ARM_TAU_MAX_Q)
    return controller

# 替换
phase6_v3_common.make_pd_controller = high_gain_pd

# 导入并运行原始测试
from scripts.test_phase6_v3_step1_simple import main
print("="*60)
print("高增益测试：Kp_arm=2400 (vs 原始500)")
print("="*60)
main(duration=10.0)
