#!/bin/bash
# ================================================================
# Booster T1 — 单球员射空门测试 独立启动脚本
# ================================================================
# 用法:
#   ./run_shoot.sh                    交互模式
#   ./run_shoot.sh [bx] [by]         指定球位 (默认 viz=ascii)
#   ./run_shoot.sh [bx] [by] [viz]   指定球位 + 可视化
#   ./run_shoot.sh [bx] [by] [viz] [dur]  全参数
#
# 可视化选项: none | ascii | mujoco
#
# 示例:
#   ./run_shoot.sh                         交互选择
#   ./run_shoot.sh 3.0 0.0 mujoco         球正中 + 3D 渲染
#   ./run_shoot.sh 2.5 1.0 ascii 15       偏右 1m + ASCII + 15s
#   ./run_shoot.sh 2.0 -1.2 none 20       偏左 + 无渲染 + 20s
#   ./run_shoot.sh --help                  查看帮助
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
MAGENTA='\033[0;35m'
NC='\033[0m'

# ---- 横幅 ----
banner() {
    echo -e "${BOLD}${GREEN}"
    echo "  ╔══════════════════════════════════════════════════════╗"
    echo "  ║  ⚽  Booster T1  单球员射空门测试                    ║"
    echo "  ║     Solo Shoot Test — 1 T1 vs Empty Goal            ║"
    echo "  ╚══════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

# ---- 帮助 ----
show_help() {
    banner
    echo ""
    echo -e "  ${BOLD}Booster T1 单球员射空门测试 — 独立启动脚本${NC}"
    echo ""
    echo -e "  ${BOLD}用法:${NC}"
    echo "    ./run_shoot.sh                       交互模式"
    echo "    ./run_shoot.sh <bx> <by>             指定球位"
    echo "    ./run_shoot.sh <bx> <by> <viz>       指定球位 + 可视化"
    echo "    ./run_shoot.sh <bx> <by> <viz> <dur> 全参数指定"
    echo ""
    echo -e "  ${BOLD}参数:${NC}"
    echo -e "    bx   球 X 坐标 (默认 3.0, 范围 ±4.5)"
    echo -e "    by   球 Y 坐标 (默认 0.0, 范围 ±3.0)"
    echo -e "    viz  可视化: none | ascii | mujoco (默认 ascii)"
    echo -e "    dur  最大时长秒数 (默认 10)"
    echo ""
    echo -e "  ${BOLD}预设球位:${NC}"
    echo -e "    ${CYAN}正面${NC}    (3.0,  0.0)  球门正前方 3m"
    echo -e "    ${CYAN}偏右${NC}    (3.0,  1.5)  球门右侧"
    echo -e "    ${CYAN}偏左${NC}    (3.0, -1.5)  球门左侧"
    echo -e "    ${CYAN}近角右${NC}  (2.0,  0.8)  近距离偏右"
    echo -e "    ${CYAN}近角左${NC}  (2.0, -0.8)  近距离偏左"
    echo -e "    ${CYAN}远射中${NC}  (4.0,  0.0)  远距离正中"
    echo -e "    ${CYAN}极限角${NC}  (1.5,  2.0)  极限角度"
    echo ""
    echo -e "  ${BOLD}示例:${NC}"
    echo "    ./run_shoot.sh                       # 交互选择"
    echo "    ./run_shoot.sh 3.0 0.0 mujoco        # 3D 可视化"
    echo "    ./run_shoot.sh 2.5 1.0 ascii 15      # ASCII + 15 秒"
    echo "    ./run_shoot.sh 2.0 -1.2 none 20      # 无渲染批量测试"
    echo ""
}

# ---- 依赖检查 ----
check_deps() {
    local MISSING=""

    python3 -c "import math, csv, json, dataclasses, typing, enum, argparse, sys, os, time" 2>/dev/null || {
        echo -e "${RED}错误: Python 3.8+ 基础模块缺失${NC}"
        exit 1
    }

    python3 -c "import mujoco; import glfw" 2>/dev/null || MISSING="$MISSING mujoco"

    if [ -n "$MISSING" ]; then
        echo -e "${YELLOW}  可选依赖未安装:${MISSING}${NC}"
        echo -e "  ${CYAN}运行 './run.sh install' 安装全部依赖${NC}"
        echo ""
    fi
}

# ---- 预设球位 ----
declare -A PRESETS=(
    ["1"]="正面|3.0|0.0"
    ["2"]="偏右|3.0|1.5"
    ["3"]="偏左|3.0|-1.5"
    ["4"]="近角右|2.0|0.8"
    ["5"]="近角左|2.0|-0.8"
    ["6"]="远射中|4.0|0.0"
    ["7"]="极限角|1.5|2.0"
)

# ---- 交互模式 ----
interactive_mode() {
    banner
    echo -e "  ${BOLD}选择球位预设:${NC}"
    echo ""
    echo -e "  ${GREEN}[1]${NC} 正面    — (3.0,  0.0)  球门正前方 3m"
    echo -e "  ${GREEN}[2]${NC} 偏右    — (3.0,  1.5)  球门右侧"
    echo -e "  ${GREEN}[3]${NC} 偏左    — (3.0, -1.5)  球门左侧"
    echo -e "  ${GREEN}[4]${NC} 近角右  — (2.0,  0.8)  近距离偏右"
    echo -e "  ${GREEN}[5]${NC} 近角左  — (2.0, -0.8)  近距离偏左"
    echo -e "  ${GREEN}[6]${NC} 远射中  — (4.0,  0.0)  远距离正中"
    echo -e "  ${GREEN}[7]${NC} 极限角  — (1.5,  2.0)  极限角度"
    echo -e "  ${GREEN}[8]${NC} 自定义  — 手动输入坐标"
    echo ""
    read -r -p "  输入选项 [1]: " CHOICE
    CHOICE="${CHOICE:-1}"

    if [ "$CHOICE" == "8" ]; then
        read -r -p "  球 X 坐标 [3.0]: " BX
        BX="${BX:-3.0}"
        read -r -p "  球 Y 坐标 [0.0]: " BY
        BY="${BY:-0.0}"
    elif [ -n "${PRESETS[$CHOICE]}" ]; then
        IFS='|' read -r _ BX BY <<< "${PRESETS[$CHOICE]}"
    else
        IFS='|' read -r _ BX BY <<< "${PRESETS[1]}"
    fi

    echo ""
    echo -e "  ${BOLD}选择可视化方式:${NC}"
    echo -e "  ${GREEN}[1]${NC} ASCII 终端 ⌨️       ${CYAN}← 推荐${NC}"
    echo -e "  ${GREEN}[2]${NC} MuJoCo 3D 🎮"
    echo -e "  ${GREEN}[3]${NC} 无渲染 📝           ${CYAN}← 快速测试${NC}"
    echo ""
    read -r -p "  输入选项 [1]: " VIZ_CHOICE
    case "${VIZ_CHOICE:-1}" in
        2) VIZ="mujoco" ;;
        3) VIZ="none" ;;
        *) VIZ="ascii" ;;
    esac

    echo ""
    read -r -p "  最大时长 (秒) [10]: " DUR
    DUR="${DUR:-10}"

    run_test "$BX" "$BY" "$VIZ" "$DUR"
}

# ---- 运行测试 ----
run_test() {
    local BX="$1"
    local BY="$2"
    local VIZ="$3"
    local DUR="$4"

    banner
    check_deps

    echo ""
    echo -e "  ${BLUE}╔══════════════════════════════════════╗${NC}"
    echo -e "  ${BLUE}║${NC}  ${BOLD}测试参数${NC}                         ${BLUE}║${NC}"
    echo -e "  ${BLUE}╠══════════════════════════════════════╣${NC}"
    printf  "  ${BLUE}║${NC}  球位:     (${GREEN}%+.2f${NC}, ${GREEN}%+.2f${NC})         ${BLUE}║${NC}\n" "$BX" "$BY"
    printf  "  ${BLUE}║${NC}  可视化:   ${CYAN}%-10s${NC}                ${BLUE}║${NC}\n" "$VIZ"
    printf  "  ${BLUE}║${NC}  最大时长: ${YELLOW}%.0f 秒${NC}                    ${BLUE}║${NC}\n" "$DUR"
    echo -e "  ${BLUE}╚══════════════════════════════════════╝${NC}"
    echo ""

    if [ "$VIZ" == "mujoco" ]; then
        python3 -c "import mujoco; import glfw" 2>/dev/null || {
            echo -e "${RED}错误: MuJoCo 未安装, 回退到 ASCII 模式${NC}"
            VIZ="ascii"
        }
    fi

    echo -e "  ${CYAN}提示:${NC} 球员从球后方出发，通过 FSM (IDLE→CHASE→SHOOT) 决策射门"
    if [ "$VIZ" == "mujoco" ]; then
        echo -e "  ${CYAN}      鼠标拖拽旋转 | 滚轮缩放 | 右键平移 | Esc 退出${NC}"
    fi
    echo ""

    python3 experiments/solo_shoot_test.py \
        --ball-x "$BX" --ball-y "$BY" \
        --viz "$VIZ" --duration "$DUR"

    local EXIT_CODE=$?
    echo ""
    if [ $EXIT_CODE -eq 0 ]; then
        echo -e "${GREEN}  ⚽ 射门成功！GOAL!${NC}"
    else
        echo -e "${YELLOW}  💨 未进球 (退出码: $EXIT_CODE)${NC}"
    fi
    echo ""
}

# ---- 批量测试 ----
run_batch() {
    banner
    check_deps
    echo ""
    echo -e "  ${BLUE}批量测试 — 跑全部预设球位 (无渲染模式)${NC}"
    echo ""

    local TOTAL=0
    local GOALS=0
    local RESULTS=()

    for key in 1 2 3 4 5 6 7; do
        IFS='|' read -r NAME BX BY <<< "${PRESETS[$key]}"
        echo -e "  ${CYAN}[${key}/7]${NC} ${NAME} (${BX}, ${BY}) ..."
        python3 experiments/solo_shoot_test.py \
            --ball-x "$BX" --ball-y "$BY" \
            --viz none --duration 10 2>&1 | tail -1
        if [ ${PIPESTATUS[0]} -eq 0 ]; then
            GOALS=$((GOALS + 1))
            RESULTS+=("  ${GREEN}✓${NC} ${NAME}")
        else
            RESULTS+=("  ${RED}✗${NC} ${NAME}")
        fi
        TOTAL=$((TOTAL + 1))
        echo ""
    done

    echo -e "  ${BOLD}批量结果: ${GOALS}/${TOTAL} 进球${NC}"
    for r in "${RESULTS[@]}"; do
        echo -e "$r"
    done
    echo ""
}

# ================================================================
# 主入口
# ================================================================

# 无参数时进入交互模式
if [ $# -eq 0 ]; then
    interactive_mode
    exit 0
fi

# 解析第一个参数
case "${1:-}" in
    -h|--help|help)
        show_help
        exit 0
        ;;

    batch|--batch)
        run_batch
        exit 0
        ;;

    # 检查是否为数字 (球位 X 坐标)
    *[!0-9.\-]*)
        echo -e "${RED}未知参数: $1${NC}"
        echo "运行 './run_shoot.sh --help' 查看帮助"
        exit 1
        ;;
esac

# 位置参数模式
BX="${1:-3.0}"
BY="${2:-0.0}"
VIZ="${3:-ascii}"
DUR="${4:-10}"

run_test "$BX" "$BY" "$VIZ" "$DUR"
