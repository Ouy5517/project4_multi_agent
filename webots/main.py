from __future__ import annotations

import argparse
from pathlib import Path
from typing import Protocol

from common.evaluation_log import EvaluationLogger
from common.robot_action import RobotAction
from common.world_state import WorldState, load_world_state
from robot_adapter.mock_adapter import MockRobotAdapter
from robot_adapter.webots_adapter import WebotsAdapter
from strategy.team_strategy import TeamStrategy


PROJECT_ROOT = Path(__file__).resolve().parent
SCENARIOS_DIR = PROJECT_ROOT / "scenarios"
OUTPUTS_DIR = PROJECT_ROOT / "outputs"


class RobotAdapter(Protocol):
    def execute(self, actions: list[RobotAction]) -> None:
        ...


def run_scenario(
    scenario_path: Path,
    strategy: TeamStrategy,
    adapter: RobotAdapter,
    logger: EvaluationLogger,
) -> None:
    """加载单个场景，运行一次或多阶段决策。"""
    world_state = load_world_state(scenario_path)
    states = [world_state] + [
        WorldState.from_dict(next_state) for next_state in world_state.next_states
    ]

    print("\n" + "=" * 72)
    print(f"场景名称：{world_state.scenario_name}")
    print(f"WorldState 摘要：{world_state.summary()}")

    for index, state in enumerate(states, start=1):
        phase_name = f"阶段 {index}" if len(states) > 1 else "单阶段"
        actions = strategy.decide(state)
        adapter.execute(actions)
        decision_summary = strategy.last_decision_summary

        print(f"\n{phase_name}")
        print(f"当前状态机状态：{strategy.state_machine.current_state.value}")
        print("决策动作：")
        for action in actions:
            print(
                f"- {action.robot_id}: {action.action_type.value} -> "
                f"target={action.target}, confidence={action.confidence:.2f}"
            )
            print(f"  决策原因：{action.reason}")

        logger.log(
            scenario_name=state.scenario_name,
            world_state=state,
            actions=actions,
            decision_summary=f"{phase_name}: {decision_summary}",
        )

    print(f"\n日志保存路径：{logger.log_path}")


def create_adapter(adapter_mode: str) -> RobotAdapter:
    if adapter_mode == "webots":
        return WebotsAdapter()
    return MockRobotAdapter()


def main() -> None:
    parser = argparse.ArgumentParser(description="Booster T1 多机器人足球协同决策系统")
    parser.add_argument(
        "--adapter",
        choices=("mock", "webots"),
        default="mock",
        help="动作适配器模式：mock 只打印动作意图，webots 打印 Webots / Booster T1 执行建议。",
    )
    args = parser.parse_args()

    strategy = TeamStrategy()
    adapter = create_adapter(args.adapter)
    logger = EvaluationLogger(OUTPUTS_DIR / "decision_log.jsonl")

    scenario_files = sorted(SCENARIOS_DIR.glob("*.json"))
    if not scenario_files:
        raise FileNotFoundError(f"未找到场景文件：{SCENARIOS_DIR}")

    logger.reset()
    for scenario_path in scenario_files:
        run_scenario(scenario_path, strategy, adapter, logger)


if __name__ == "__main__":
    main()
