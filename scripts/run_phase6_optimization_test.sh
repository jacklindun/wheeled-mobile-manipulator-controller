#!/bin/bash
# Phase 6-v2 双臂优化测试脚本

echo "======================================================================"
echo "Phase 6-v2 双臂轨迹跟踪测试"
echo "======================================================================"
echo ""
echo "请选择测试版本:"
echo ""
echo "1. 最佳版本 (推荐)"
echo "   - 固定基座IK + 线性插值 + 高增益"
echo "   - 预期结果: ~14.5 cm"
echo ""
echo "2. 降速版本"
echo "   - 在最佳版本基础上降低轨迹速度"
echo "   - 预期结果: ~13.7 cm"
echo ""
echo "3. 查看优化总结"
echo "   - 显示完整的优化历程和分析"
echo ""
echo "4. 退出"
echo ""
read -p "请输入选择 (1-4): " choice

case $choice in
    1)
        echo ""
        echo "运行最佳版本..."
        echo ""
        pixi run -e all python scripts/test_phase6_v2_simple.py
        ;;
    2)
        echo ""
        read -p "请输入角速度 (默认0.2 rad/s): " omega
        omega=${omega:-0.2}
        echo ""
        echo "运行降速版本 (omega=$omega)..."
        echo ""
        pixi run -e all python scripts/test_phase6_v2_slowdown.py --omega $omega
        ;;
    3)
        echo ""
        python PHASE6_OPTIMIZATION_SUMMARY.md
        ;;
    4)
        echo "退出"
        exit 0
        ;;
    *)
        echo "无效选择"
        exit 1
        ;;
esac
