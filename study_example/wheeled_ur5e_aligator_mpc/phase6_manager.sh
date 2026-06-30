#!/bin/bash
# Phase 6 完整管理工具

PROJECT_ROOT="/home/ldq/spirita-work/mobile-manipulator/aligator/study_example/wheeled_ur5e_aligator_mpc"

echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║              Phase 6 项目管理工具                             ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

# 功能选择
show_menu() {
    echo "请选择操作:"
    echo "  1. 查看项目状态"
    echo "  2. 运行测试"
    echo "  3. 查看文档列表"
    echo "  4. 查看代码统计"
    echo "  5. 运行Step 2测试 (插值器)"
    echo "  6. 运行Step 3测试 (前馈PD)"
    echo "  7. 运行Step 4测试 (集成控制器)"
    echo "  8. 生成完整报告"
    echo "  9. 退出"
    echo ""
    echo -n "请输入选项 [1-9]: "
}

# 1. 项目状态
show_status() {
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "Phase 6 项目状态"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

    echo ""
    echo "📊 完成进度:"
    echo "  Step 1: Full Dynamic MPC        ⚠️  90% (API问题)"
    echo "  Step 2: 轨迹插值器              ✅ 100%"
    echo "  Step 3: 前馈PD控制器            ✅ 100%"
    echo "  Step 4: 集成控制器              ✅ 100%"
    echo "  Step 5: MuJoCo闭环demo         ⏳  0%"
    echo ""
    echo "  总体进度: 70%"

    echo ""
    echo "💻 代码文件:"
    ls -lh "$PROJECT_ROOT/wheeled_ur5e_aligator_mpc/"*.py 2>/dev/null | grep -E "(interpolator|feedforward|phase6_controller|full_dynamic)" | awk '{printf "  %-40s %8s\n", $9, $5}'

    echo ""
    echo "📚 文档文件:"
    ls -1 "$PROJECT_ROOT"/PHASE6*.md 2>/dev/null | wc -l | xargs -I {} echo "  共 {} 个Phase 6文档"

    echo ""
}

# 2. 运行测试
run_tests() {
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "运行Phase 6测试套件"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

    cd "$PROJECT_ROOT"
    eval "$(pixi shell-hook -e all)"

    echo ""
    echo "测试 Step 2: 插值器..."
    PYTHONPATH=$PWD:$PYTHONPATH python wheeled_ur5e_aligator_mpc/trajectory_interpolator.py 2>&1 | tail -5

    echo ""
    echo "测试 Step 3: 前馈PD..."
    PYTHONPATH=$PWD:$PYTHONPATH python wheeled_ur5e_aligator_mpc/feedforward_pd_controller.py 2>&1 | tail -5

    echo ""
    echo "测试 Step 4: 集成控制器..."
    PYTHONPATH=$PWD:$PYTHONPATH python wheeled_ur5e_aligator_mpc/phase6_controller.py 2>&1 | tail -5

    echo ""
}

# 3. 文档列表
show_docs() {
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "Phase 6 文档列表"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

    echo ""
    echo "核心文档:"
    ls -lh "$PROJECT_ROOT"/PHASE6_WORK_SUMMARY.md 2>/dev/null && echo "  ✓ PHASE6_WORK_SUMMARY.md (工作总结 ⭐)"
    ls -lh "$PROJECT_ROOT"/PHASE6_NEW_DESIGN.md 2>/dev/null && echo "  ✓ PHASE6_NEW_DESIGN.md (新架构设计)"
    ls -lh "$PROJECT_ROOT"/PHASE6_DAILY_REPORT.md 2>/dev/null && echo "  ✓ PHASE6_DAILY_REPORT.md (今日报告)"

    echo ""
    echo "导航文档:"
    ls -lh "$PROJECT_ROOT"/PHASE6_NAVIGATION_GUIDE.md 2>/dev/null && echo "  ✓ PHASE6_NAVIGATION_GUIDE.md"
    ls -lh "$PROJECT_ROOT"/PHASE6_README.md 2>/dev/null && echo "  ✓ PHASE6_README.md"

    echo ""
    echo "所有文档:"
    ls -1 "$PROJECT_ROOT"/PHASE6*.md 2>/dev/null | sed 's|.*/||' | nl

    echo ""
}

# 4. 代码统计
show_code_stats() {
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "Phase 6 代码统计"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

    cd "$PROJECT_ROOT/wheeled_ur5e_aligator_mpc"

    echo ""
    echo "各模块代码行数:"
    wc -l trajectory_interpolator.py feedforward_pd_controller.py phase6_controller.py full_dynamic_mpc_controller.py 2>/dev/null

    echo ""
    echo "Python文件统计:"
    find . -name "*.py" | wc -l | xargs -I {} echo "  总Python文件数: {}"
    find . -name "*.py" -exec wc -l {} + 2>/dev/null | tail -1 | awk '{print "  总代码行数: " $1}'

    echo ""
    echo "测试文件:"
    ls -1 ../tests/test_*.py 2>/dev/null | wc -l | xargs -I {} echo "  测试文件数: {}"

    echo ""
}

# 5-7. 运行单个测试
run_step2() {
    echo ""
    echo "运行 Step 2: 轨迹插值器测试"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    cd "$PROJECT_ROOT"
    eval "$(pixi shell-hook -e all)"
    PYTHONPATH=$PWD:$PYTHONPATH python wheeled_ur5e_aligator_mpc/trajectory_interpolator.py
    echo ""
}

run_step3() {
    echo ""
    echo "运行 Step 3: 前馈PD控制器测试"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    cd "$PROJECT_ROOT"
    eval "$(pixi shell-hook -e all)"
    PYTHONPATH=$PWD:$PYTHONPATH python wheeled_ur5e_aligator_mpc/feedforward_pd_controller.py
    echo ""
}

run_step4() {
    echo ""
    echo "运行 Step 4: 集成控制器测试"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    cd "$PROJECT_ROOT"
    eval "$(pixi shell-hook -e all)"
    PYTHONPATH=$PWD:$PYTHONPATH python wheeled_ur5e_aligator_mpc/phase6_controller.py
    echo ""
}

# 8. 生成完整报告
generate_report() {
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "Phase 6 完整报告"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

    REPORT_FILE="$PROJECT_ROOT/PHASE6_STATUS_REPORT_$(date +%Y%m%d_%H%M%S).md"

    cat > "$REPORT_FILE" << 'ENDREPORT'
# Phase 6 状态报告

**生成时间**: $(date '+%Y-%m-%d %H:%M:%S')

## 完成进度

| Step | 名称 | 状态 | 完成度 |
|------|------|------|--------|
| 1 | Full Dynamic MPC | ⚠️ | 90% |
| 2 | 轨迹插值器 | ✅ | 100% |
| 3 | 前馈PD控制器 | ✅ | 100% |
| 4 | 集成控制器 | ✅ | 100% |
| 5 | MuJoCo闭环demo | ⏳ | 0% |

**总进度**: 70%

## 代码文件

ENDREPORT

    ls -lh "$PROJECT_ROOT/wheeled_ur5e_aligator_mpc/"*.py 2>/dev/null | grep -E "(interpolator|feedforward|phase6_controller|full_dynamic)" >> "$REPORT_FILE"

    echo "" >> "$REPORT_FILE"
    echo "## 文档文件" >> "$REPORT_FILE"
    echo "" >> "$REPORT_FILE"
    ls -1 "$PROJECT_ROOT"/PHASE6*.md 2>/dev/null >> "$REPORT_FILE"

    echo ""
    echo "✓ 报告已生成: $REPORT_FILE"
    echo ""
}

# 主循环
while true; do
    show_menu
    read choice

    case $choice in
        1) show_status ;;
        2) run_tests ;;
        3) show_docs ;;
        4) show_code_stats ;;
        5) run_step2 ;;
        6) run_step3 ;;
        7) run_step4 ;;
        8) generate_report ;;
        9)
            echo ""
            echo "感谢使用Phase 6管理工具！"
            echo ""
            exit 0
            ;;
        *)
            echo ""
            echo "无效选项，请重新选择"
            echo ""
            ;;
    esac

    echo ""
    echo "按Enter继续..."
    read
done
