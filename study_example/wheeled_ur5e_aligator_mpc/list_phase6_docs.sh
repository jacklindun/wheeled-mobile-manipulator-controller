#!/bin/bash
# Phase 6 文档列表（2026-06-26 重组后）

PROJ_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
PHASE6_DIR="$PROJ_ROOT/docs/phase6"

echo "=========================================="
echo "Phase 6 文档中心"
echo "=========================================="
echo ""
echo "主入口: $PHASE6_DIR/README.md"
echo ""

echo "📋 当前有效文档:"
echo "-------------------------------------------"
for f in README.md STATUS.md V2.md V3.md ARCHIVE_INDEX.md; do
  ls -lh "$PHASE6_DIR/$f" 2>/dev/null && echo "  ✓ $f" || echo "  ✗ $f"
done
echo ""

echo "🗂️  归档文档:"
echo "-------------------------------------------"
find "$PHASE6_DIR/archive" -name "*.md" | wc -l | xargs echo "  归档 .md 数量:"
du -sh "$PHASE6_DIR/archive" 2>/dev/null | awk '{print "  归档总大小: " $1}'
echo ""

echo "🚀 快速阅读:"
echo "-------------------------------------------"
echo "  cat $PHASE6_DIR/STATUS.md"
echo "  cat $PHASE6_DIR/V3.md"
echo ""
echo "=========================================="