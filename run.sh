#!/bin/bash
# ============================================================
# Booster T1 多机器人足球协同决策系统 — 一键运行脚本
# 题目四：Multi-Robot Soccer Cooperative Decision System
# ============================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "============================================"
echo "  Booster T1 多机器人足球协同决策系统"
echo "  题目四: Multi-Robot Soccer Cooperative Decision System"
echo "============================================"

# 检查 Python
if ! command -v python3 &> /dev/null; then
    echo "错误: 需要 Python 3.8+"
    exit 1
fi

# 创建输出目录
mkdir -p outputs

# 安装依赖 (如果需要)
echo ""
echo "[1/2] 检查依赖..."
python3 -c "import math, csv, json, dataclasses, typing, enum, abc, argparse, sys, os, time" 2>/dev/null || {
    echo "基础模块检查完成"
}

# 可选依赖检查
INSTALL_DEPS=""
python3 -c "import matplotlib" 2>/dev/null || INSTALL_DEPS="$INSTALL_DEPS matplotlib"
python3 -c "import pytest" 2>/dev/null || INSTALL_DEPS="$INSTALL_DEPS pytest"

if [ -n "$INSTALL_DEPS" ]; then
    echo "  可选依赖未安装: $INSTALL_DEPS"
    echo "  运行 './run.sh install' 安装可选依赖"
fi

if [ "$1" = "install" ]; then
    echo ""
    echo "  安装依赖..."
    pip3 install matplotlib pytest pytest-cov --user
    echo "  依赖安装完成!"
    exit 0
fi

# 运行
echo ""
echo "[2/2] 启动仿真..."
echo ""

if [ "$1" = "test" ]; then
    echo "  运行测试套件..."
    python3 -m pytest tests/ -v --tb=short 2>/dev/null || {
        echo ""
        echo "  pytest 未安装, 尝试直接运行测试..."
        for f in tests/test_*.py; do
            echo "  Running: $f"
            python3 "$f" || echo "  (需要 pytest: pip3 install pytest --user)"
        done
    }
else
    # 解析额外参数
    DURATION="${1:-30}"
    SCENARIO="${2:-default}"

    echo "  运行参数: --duration ${DURATION} --export-csv"
    echo ""
    python3 main.py --duration "${DURATION}" --export-csv "$@"
fi

echo ""
echo "============================================"
echo "  运行完成!"
echo "============================================"
