#!/bin/bash
# ================================================================
# Booster T1 多机器人足球协同决策系统 — 一键启动脚本
# ================================================================
# 用法:
#   ./run.sh                    交互菜单
#   ./run.sh 3d                 MuJoCo 3D 可视化 (默认场景)
#   ./run.sh 3d pass            MuJoCo 3D + 传球场景
#   ./run.sh 3d shoot           MuJoCo 3D + 射门场景
#   ./run.sh 3d threat          MuJoCo 3D + 防守场景
#   ./run.sh 2d                 Matplotlib 2D 图形窗口
#   ./run.sh 2d pass            Matplotlib 2D + 传球场景
#   ./run.sh ascii              ASCII 终端可视化
#   ./run.sh headless [秒数]    无渲染模式 (仅日志)
#   ./run.sh test               运行测试套件
#   ./run.sh install            安装全部依赖
# ================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# ---- 颜色 ----
BOLD='\033[1m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m'

# ---- 横幅 ----
banner() {
    echo -e "${BOLD}${BLUE}"
    echo "  ╔══════════════════════════════════════════════════════╗"
    echo "  ║     Booster T1  多机器人足球协同决策系统             ║"
    echo "  ║     Multi-Robot Soccer Cooperative Decision System   ║"
    echo "  ╚══════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

# ---- 依赖检查 ----
check_deps() {
    local MISSING=""

    # 基础模块
    python3 -c "import math, csv, json, dataclasses, typing, enum, abc, argparse, sys, os, time" 2>/dev/null || {
        echo -e "${RED}错误: Python 3.8+ 基础模块缺失${NC}"
        exit 1
    }

    # matplotlib
    python3 -c "import matplotlib" 2>/dev/null || MISSING="$MISSING matplotlib"

    # pytest
    python3 -c "import pytest" 2>/dev/null || MISSING="$MISSING pytest"

    # MuJoCo
    python3 -c "import mujoco; import glfw" 2>/dev/null || MISSING="$MISSING mujoco"

    if [ -n "$MISSING" ]; then
        echo -e "${YELLOW}  可选依赖未安装:${MISSING}${NC}"
        echo -e "  ${CYAN}运行 './run.sh install' 安装全部依赖${NC}"
        echo ""
    fi
}

# ---- 解析参数 (支持 --duration X --scenario Y 透传) ----
parse_extra_args() {
    local EXTRA=""
    while [ $# -gt 0 ]; do
        case "$1" in
            --duration) EXTRA="$EXTRA --duration $2"; shift 2 ;;
            --scenario) EXTRA="$EXTRA --scenario $2"; shift 2 ;;
            --headless) EXTRA="$EXTRA --headless"; shift ;;
            --export-csv) EXTRA="$EXTRA --export-csv"; shift ;;
            --export-gif) EXTRA="$EXTRA --export-gif $2"; shift 2 ;;
            *) shift ;;
        esac
    done
    echo "$EXTRA"
}

# ---- 交互菜单 ----
interactive_menu() {
    banner
    echo -e "  ${BOLD}请选择运行模式:${NC}"
    echo ""
    echo -e "  ${GREEN}[1]${NC} MuJoCo 3D 可视化 🎮        ${CYAN}← 推荐${NC}"
    echo -e "  ${GREEN}[2]${NC} Matplotlib 2D 图形窗口 📊"
    echo -e "  ${GREEN}[3]${NC} ASCII 终端可视化 ⌨️"
    echo -e "  ${GREEN}[4]${NC} 无渲染模式 (仅日志) 📝"
    echo -e "  ${GREEN}[5]${NC} 运行测试套件 🧪"
    echo -e "  ${GREEN}[6]${NC} 安装全部依赖 📦"
    echo -e "  ${GREEN}[0]${NC} 退出"
    echo ""
    read -r -p "  输入选项 [1]: " CHOICE
    CHOICE="${CHOICE:-1}"

    case "$CHOICE" in
        1) mode_3d "$@" ;;
        2) mode_2d "$@" ;;
        3) mode_ascii "$@" ;;
        4) mode_headless "$@" ;;
        5) run_tests ;;
        6) install_deps ;;
        0) echo "  退出"; exit 0 ;;
        *) echo -e "${RED}无效选项${NC}"; exit 1 ;;
    esac
}

# ---- 场景子菜单 ----
pick_scenario() {
    echo ""
    echo -e "  ${BOLD}选择场景:${NC}"
    echo -e "  ${GREEN}[1]${NC} default  — 默认开球"
    echo -e "  ${GREEN}[2]${NC} pass     — 传球场景"
    echo -e "  ${GREEN}[3]${NC} shoot    — 射门场景"
    echo -e "  ${GREEN}[4]${NC} threat   — 防守场景"
    echo ""
    read -r -p "  输入选项 [1]: " SC
    case "${SC:-1}" in
        1) echo "default" ;;
        2) echo "pass" ;;
        3) echo "shoot" ;;
        4) echo "threat" ;;
        *) echo "default" ;;
    esac
}

pick_duration() {
    echo ""
    read -r -p "  仿真时长 (秒) [30]: " DUR
    echo "${DUR:-30}"
}

# ================================================================
# 各模式入口
# ================================================================

mode_3d() {
    banner
    check_deps

    # 检查 MuJoCo
    python3 -c "import mujoco; import glfw" 2>/dev/null || {
        echo -e "${RED}错误: MuJoCo 未安装, 请运行 './run.sh install'${NC}"
        exit 1
    }

    local SCENARIO="$1"
    local DURATION="$2"

    [ -z "$SCENARIO" ] && SCENARIO=$(pick_scenario)
    [ -z "$DURATION" ] && DURATION=$(pick_duration)

    echo ""
    echo -e "  ${BLUE}启动:${NC} MuJoCo 3D | 场景=${SCENARIO} | ${DURATION}s"
    echo -e "  ${CYAN}提示:${NC} 鼠标拖拽旋转 | 滚轮缩放 | 右键平移"
    echo ""

    python3 main.py \
        --viz mujoco \
        --scenario "$SCENARIO" \
        --duration "$DURATION" \
        --export-csv

    echo ""
    echo -e "${GREEN}仿真完成!${NC}"
}

mode_2d() {
    banner
    check_deps

    python3 -c "import matplotlib" 2>/dev/null || {
        echo -e "${RED}错误: matplotlib 未安装, 请运行 './run.sh install'${NC}"
        exit 1
    }

    local SCENARIO="$1"
    local DURATION="$2"

    [ -z "$SCENARIO" ] && SCENARIO=$(pick_scenario)
    [ -z "$DURATION" ] && DURATION=$(pick_duration)

    echo ""
    echo -e "  ${BLUE}启动:${NC} Matplotlib 2D | 场景=${SCENARIO} | ${DURATION}s"
    echo ""

    python3 main.py \
        --viz matplotlib \
        --scenario "$SCENARIO" \
        --duration "$DURATION" \
        --export-csv

    echo ""
    echo -e "${GREEN}仿真完成!${NC}"
}

mode_ascii() {
    banner
    check_deps

    local SCENARIO="$1"
    local DURATION="$2"

    [ -z "$SCENARIO" ] && SCENARIO=$(pick_scenario)
    [ -z "$DURATION" ] && DURATION=$(pick_duration)

    echo ""
    echo -e "  ${BLUE}启动:${NC} ASCII 终端 | 场景=${SCENARIO} | ${DURATION}s"
    echo ""

    python3 main.py \
        --viz ascii \
        --scenario "$SCENARIO" \
        --duration "$DURATION" \
        --export-csv

    echo ""
    echo -e "${GREEN}仿真完成!${NC}"
}

mode_headless() {
    banner
    local DURATION="${1:-30}"

    echo ""
    echo -e "  ${BLUE}启动:${NC} Headless (无渲染) | ${DURATION}s"
    echo ""

    python3 main.py \
        --headless \
        --duration "$DURATION" \
        --export-csv

    echo ""
    echo -e "${GREEN}仿真完成! 日志: outputs/decision_log.csv${NC}"
}

run_tests() {
    banner
    echo -e "  ${BLUE}运行测试套件...${NC}"
    echo ""

    if python3 -c "import pytest" 2>/dev/null; then
        python3 -m pytest tests/ -v --tb=short
    else
        echo -e "${YELLOW}  pytest 未安装, 尝试直接运行...${NC}"
        for f in tests/test_*.py; do
            echo "  Running: $f"
            python3 "$f" || true
        done
    fi

    echo ""
    echo -e "${GREEN}测试完成!${NC}"
}

install_deps() {
    banner
    echo -e "  ${BLUE}安装全部依赖...${NC}"
    echo ""

    pip3 install --user matplotlib pytest pytest-cov mujoco glfw 2>&1

    echo ""
    echo -e "${GREEN}依赖安装完成!${NC}"
    echo ""
    echo -e "  已安装:"
    python3 -c "import matplotlib; print(f'    matplotlib  {matplotlib.__version__}')" 2>/dev/null || true
    python3 -c "import pytest;     print(f'    pytest      {pytest.__version__}')" 2>/dev/null || true
    python3 -c "import mujoco;     print(f'    mujoco      {mujoco.__version__}')" 2>/dev/null || true
    python3 -c "import glfw;       print(f'    glfw        {glfw.__version__}')" 2>/dev/null || true
}

# ================================================================
# 主入口
# ================================================================

# 无参数时进入交互菜单
if [ $# -eq 0 ]; then
    interactive_menu
    exit 0
fi

# 解析第一个参数
CMD="$1"
shift 2>/dev/null || true

case "$CMD" in
    3d|mujoco)
        SC=""; DUR=""; FS=""
        for arg in "$@"; do
            case "$arg" in
                pass|shoot|threat|default) SC="$arg" ;;
                ''|*[!0-9]*) ;;  # skip non-numeric
                *) DUR="$arg" ;;
            esac
        done
        mode_3d "$SC" "$DUR" "$FS"
        ;;

    2d|matplotlib)
        SC=""; DUR=""
        for arg in "$@"; do
            case "$arg" in
                pass|shoot|threat|default) SC="$arg" ;;
                ''|*[!0-9]*) ;;
                *) DUR="$arg" ;;
            esac
        done
        mode_2d "$SC" "$DUR"
        ;;

    ascii|text)
        SC=""; DUR=""
        for arg in "$@"; do
            case "$arg" in
                pass|shoot|threat|default) SC="$arg" ;;
                ''|*[!0-9]*) ;;
                *) DUR="$arg" ;;
            esac
        done
        mode_ascii "$SC" "$DUR"
        ;;

    headless|--headless)
        DUR="${1:-30}"
        mode_headless "$DUR"
        ;;

    test|tests|pytest)
        run_tests
        ;;

    install|setup)
        install_deps
        ;;

    -h|--help|help)
        echo "Booster T1 一键启动脚本"
        echo ""
        echo "用法:"
        echo "  ./run.sh                      交互菜单"
        echo "  ./run.sh 3d [场景] [秒数]     MuJoCo 3D 可视化"
        echo "  ./run.sh 2d [场景] [秒数]     Matplotlib 2D 图形"
        echo "  ./run.sh ascii [场景] [秒数]  ASCII 终端可视化"
        echo "  ./run.sh headless [秒数]      无渲染模式"
        echo "  ./run.sh test                 运行测试"
        echo "  ./run.sh install              安装依赖"
        echo ""
        echo "示例:"
        echo "  ./run.sh 3d pass 60           MuJoCo 3D + 传球 + 60秒"
        echo "  ./run.sh 2d threat            Matplotlib + 防守场景"
        echo "  ./run.sh headless 120         无渲染跑2分钟"
        ;;

    *)
        echo -e "${RED}未知命令: $CMD${NC}"
        echo "运行 './run.sh help' 查看帮助"
        exit 1
        ;;
esac
