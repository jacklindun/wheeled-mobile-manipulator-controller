#!/bin/bash
# Phase 6-v2 双臂优化测试脚本

echo "Phase 6-v2 双臂轨迹跟踪测试"
echo "======================================"
echo ""
echo "架构: 固定基座IK → 线性插值 → MuJoCo高增益"
echo "- IK: 固定基座，双臂联合优化"
echo "- 插值: 25:1 (20Hz → 500Hz)"
echo "- 执行器: kp=10000 (shoulder), kp=8000 (elbow)"
echo ""
echo "运行测试..."
echo ""

pixi run -e all python scripts/test_phase6_v2_simple.py
