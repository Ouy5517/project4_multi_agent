# System Architecture

The completed strategy stack uses:

- `common/world_state.py`: unified ball, robot, opponent, goal, and field state.
- `strategy/pass_strategy.py`: safe pass decision, scoring, hard constraints, dynamic receive point, and interception prediction.
- `strategy/team_strategy.py`: high-level team actions using `PassDecision`.
- `robot_adapter/*`: mock and Webots explanatory adapters.
- `demo/pass_decision_demo.py`: explainable pass decision demo.
- `demo/run_pass_experiments.py`: strategy comparison experiments.

The intended runtime integration is:

Webots world -> Runner/mck -> RPC service -> Booster SDK -> WorldState builder -> PassStrategy -> high-level action commands.

Runner integration is currently blocked by missing official Booster config YAML files.
