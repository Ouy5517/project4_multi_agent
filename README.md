This project is a 3v3 simulation soccer Agent example built on
`booster_agent_framework`. It reads team-view ROS2 simulation ground truth and
GameController referee state, runs a `py_trees` behavior-tree strategy, and
sends walking or kicking commands to the configured team robots.

The official 3v3 simulation soccer website will be provided later.

## Runtime Environment

The overall architecture is an Agent built on `booster_agent_framework`. It is
designed to run inside the virtual robots created by Booster Studio. Booster
Studio download link: <https://studio.booster.tech/>.

A typical Agent project repository has this structure:

```text
.
├── res/           # Static assets such as logos
├── src/           # Agent logic code
├── agent.toml     # Agent entry declaration and metadata
│                  # This project uses src/main.py:SoccerSimAgent
└── build.toml     # Runtime dependencies and platform support
                   # This project depends on py_trees==2.4.0
```

## Code Map

This section answers two questions: **what the code looks like** and **where to
start when changing behavior**.

### Directory Structure

```text
src/
├── soccer_framework/      # Soccer environment framework package and public API
│   ├── __init__.py        # Public entry point for PlayContext, RobotCommand, SoccerConfig, etc.
│   ├── types.py           # Data types + PlayContextProvider abstraction
│   ├── config.py          # SoccerConfig, SoccerStrategyTuning, SoccerDebugConfig, and from_env
│   ├── game_state.py      # GameController JSON encoding/decoding
│   ├── ros_truth.py       # RosTruthProvider ROS ground-truth adapter
│   ├── robot.py           # TeamRobotManager + kick/control adapters
│   ├── game_controller.py # GameControllerRosProvider GC topic provider
│   ├── ros_adapter.py     # SoccerRosAdapter node/subscription/executor owner
│   └── telemetry.py       # SoccerLogger + JSONL structured logging plugin
├── runtime.py             # SoccerKit and SoccerTeamRuntime assembly/control loop + ROS adapter
├── main.py                # Booster Agent entry point and Agent lifecycle owner
├── tactics/               # Pure model layer; no BT or ROS dependency
│   ├── geometry.py        # Coordinate transforms + team-view field geometry
│   ├── navigation.py      # ObstacleCollector obstacle collection
│   ├── targeting/         # Tactical targets split by responsibility
│   │   └── __init__.py    # Targeting facade with stable public API
│   ├── motion.py          # MotionController obstacle avoidance/walking/kick commands
│   ├── kick_hysteresis.py # Kick enter/exit hysteresis model
│   └── ready_stance.py    # READY positioning calculation
├── behavior_tree/         # BT infrastructure: blackboard, nodes, subtrees, assembly
│   ├── __init__.py        # Exports TeamStrategyTree, TeamCommandExecutor, create_team_tree
│   ├── blackboard.py      # BlackboardKeys table + BlackboardClient
│   ├── tree.py            # TeamStrategyTree + create_team_tree top-level assembly
│   ├── ready_subtree.py   # READY subtree factory
│   ├── safety_subtree.py  # SafetyGuards and SafetyOverrides subtree factories
│   └── nodes/
│       ├── data.py        # Data leaf nodes that write to the blackboard
│       ├── conditions.py  # Common ball/rule/hardware condition leaf nodes
│       └── actions.py     # Common action leaf nodes such as StopAll, READY, CommitTeamCommands
└── play/                  # Template core: all PLAY-phase strategy code
    ├── __init__.py        # Explicitly registers the default Playbook at the end
    ├── playbook.py        # Playbook, DefaultPlaybook, RoleAssignment, select_chaser
    ├── registry.py        # PlaybookRegistry + global PLAYBOOKS registry
    ├── role.py            # Role abstraction and registration
    ├── default_roles.py   # Default dynamic roles; defender is available for extension
    ├── play_subtree.py    # create_play_subtree PLAY subtree factory
    └── nodes.py           # Shared leaf nodes and attack subtree builder
```

### Dependency Direction

```text
play             -> behavior_tree + tactics + soccer_framework (+ SoccerKit from runtime)
behavior_tree    -> runtime (types only) + tactics + soccer_framework
tactics          -> soccer_framework
runtime          -> play + behavior_tree + tactics + soccer_framework
soccer_framework -> (no internal dependency)
```

`soccer_framework` never imports any upper-layer package. This is what keeps the
contract true that participants only need to care about the data and commands
provided by the framework. `SoccerKit` in `runtime` does not depend on `play`:
the two are decoupled through the `Playbook` protocol. `SoccerKit` provides
capability tools, while `play` implements PLAY-phase decisions.

### Behavior Tree Overview

```text
TeamRoot
├── DataLayer
│   ├── UpdateClock
│   ├── UpdatePlayContext
│   ├── UpdateGameState / UpdateRecentBall / UpdateRobotPoses
│   └── UpdateRobotStatus(N)
├── MatchControl
│   ├── SafetyGuards
│   │   ├── no game controller -> StopAll
│   │   ├── all inactive -> StopAll
│   │   ├── stopped=true -> StopAll
│   │   ├── non-playing state -> StopAll
│   │   └── PLAYING without ball -> StopAll
│   ├── ReadyPhase
│   │   └── ReadySlots: GoReadyTarget(N)
│   ├── PlayingPhase
│   │   └── PlaybookCore: AssignRoles + Roles(Player(N))
│   └── unsupported state -> StopAll
├── SafetyOverrides
│   └── PlayerSafety(N): allowed / fall-down / walk-mode overlays
└── CommitTeamCommands
```

See [docs/bt_structure.md](docs/bt_structure.md) for a leaf-level behavior tree
map.

Strategy code always uses the team field coordinate frame. Robot poses and ball
coordinates from simulation ground truth are already expressed relative to the
current team: the own goal is at `x=-field_length/2`, the opponent goal is at
`x=+field_length/2`, `+x` is the team's attacking direction, and `-x` is the
team's defensive direction. Simulation strategies can use `/teamN/...` ground
truth directly. Do not mirror coordinates again by `team_id` inside `play/` or
`tactics/`. If a future input source provides absolute field coordinates,
normalization should happen in the `PlayContextProvider` adapter layer.

### Where To Start

Start with these three files under [src/play/](src/play/) when making strategy
changes:

| What you want to change | Entry point |
| --- | --- |
| **Role assignment, such as all-out attack when trailing or two-player attack** | `Playbook.assign_roles` in [src/play/playbook.py](src/play/playbook.py) |
| **Attack or pass target point** | `ChaserRole.kick_target` in [src/play/default_roles.py](src/play/default_roles.py) |
| **Support positioning** | `SupporterRole.target` in [src/play/default_roles.py](src/play/default_roles.py) |
| **Defensive positioning for custom extensions** | `DefenderRole.target` in [src/play/default_roles.py](src/play/default_roles.py) |
| **Goalkeeper guard position** | `GoalkeeperRole.target` in [src/play/default_roles.py](src/play/default_roles.py) |
| **Add a new role such as interceptor or two strikers** | Derive `RoleStrategy` from [src/play/role.py](src/play/role.py), call `register_role(...)` in `Playbook.__init__`; for pure positioning roles, override `target()` and use `MoveToTarget`; for compound roles, use `build_attack_subtree` |
| **Register a new Playbook, either default or by name** | `PLAYBOOKS.register(name, factory)` in [src/play/registry.py](src/play/registry.py); see the end of [src/play/__init__.py](src/play/__init__.py) to change the default |
| **Ball-chasing score, deciding who chases** | `DefaultPlaybook.select_chaser` in [src/play/playbook.py](src/play/playbook.py) |
| **PLAY subtree shape** | [src/play/play_subtree.py](src/play/play_subtree.py) |
| **Full-team stop when ball or referee data is missing** | `SafetyGuards` in [src/behavior_tree/safety_subtree.py](src/behavior_tree/safety_subtree.py) |
| **Fallback action for a player without an assigned role** | `Playbook.waiting_command` in [src/play/playbook.py](src/play/playbook.py) |
| Goalkeeper formula | `ReadyStance.goalkeeper_guard_target` in [src/tactics/ready_stance.py](src/tactics/ready_stance.py) |
| Attack scoring details, such as shooting lane, pass scoring, and dribbling | [src/tactics/targeting/attack.py](src/tactics/targeting/attack.py) |
| Support positioning algorithm, including teammate spacing repulsion | [src/tactics/targeting/support.py](src/tactics/targeting/support.py) |
| Restart avoidance or sideline recovery target | [src/tactics/targeting/restart.py](src/tactics/targeting/restart.py) / [src/tactics/targeting/recovery.py](src/tactics/targeting/recovery.py) |
| Obstacle avoidance or teammate avoidance | [src/tactics/navigation.py](src/tactics/navigation.py) |
| Team field coordinate geometry and coordinate transforms | [src/tactics/geometry.py](src/tactics/geometry.py) |
| Kick enter/exit hysteresis | [src/tactics/kick_hysteresis.py](src/tactics/kick_hysteresis.py) |
| READY / SafetyGuards / SafetyOverrides | [src/behavior_tree/ready_subtree.py](src/behavior_tree/ready_subtree.py) / [src/behavior_tree/safety_subtree.py](src/behavior_tree/safety_subtree.py) |
| How one frame of data is written to the blackboard | [src/behavior_tree/nodes/data.py](src/behavior_tree/nodes/data.py) |

### Example Template: Derive a Playbook

`TeamStrategyTree` does not create any default Playbook by itself. All Playbooks,
including `DefaultPlaybook`, are passed in explicitly during construction and
registered through the `PLAYBOOKS` registry in
[src/play/registry.py](src/play/registry.py). The `DefaultPlaybook` registration
is at the end of [src/play/__init__.py](src/play/__init__.py), using the exact
same interface participants use to register their own Playbook.

Run one match with the default strategy:

```python
from src.behavior_tree import TeamStrategyTree
from src.play import PLAYBOOKS
from src.runtime import SoccerKit
from src.soccer_framework import SoccerConfig

kit = SoccerKit(SoccerConfig())
tree = TeamStrategyTree(kit, PLAYBOOKS.create_default(kit), context_provider)
```

Derive your own Playbook and override only two or three methods:

```python
from src.behavior_tree import TeamStrategyTree
from src.play import DefaultPlaybook, PLAYBOOKS, PlayContext, RoleAssignment
from src.runtime import SoccerKit
from src.soccer_framework import SoccerConfig


class AggressivePlaybook(DefaultPlaybook):
    def assign_roles(self, context: PlayContext):
        base = super().assign_roles(context)
        # Temporarily count the goalkeeper as a supporter for all-out attack.
        game = context.known_game
        own_team = game.get_team_state(self.kit.config.team_id)
        other_team = next(
            (
                team
                for team in game.teams
                if team.team_number != self.kit.config.team_id
            ),
            None,
        )
        if (
            own_team is not None
            and other_team is not None
            and own_team.score + 1 < other_team.score
        ):
            mapping = dict(base.by_player)
            goalkeeper = next(
                (
                    player_id
                    for player_id, role in base.by_player.items()
                    if role == "goalkeeper"
                ),
                None,
            )
            if goalkeeper is not None:
                mapping[goalkeeper] = "supporter"
            return RoleAssignment(mapping)
        return base


# Register one line in your entry module, same as DefaultPlaybook registration in play/__init__.py.
PLAYBOOKS.register("aggressive", AggressivePlaybook)

kit = SoccerKit(SoccerConfig())
tree = TeamStrategyTree(kit, PLAYBOOKS.create("aggressive", kit), context_provider)
```

You can also skip the registry and pass the object directly, which is convenient
for one-off usage:

```python
tree = TeamStrategyTree(kit, AggressivePlaybook(kit), context_provider)
```

The PLAY subtree shape, command dispatch path, and READY/Safety phases do not
need to change.

### Add a New Role in Three Steps

To introduce a role beyond chaser/supporter/defender/goalkeeper, such as an
`interceptor` that blocks passing lanes:

```python
from src.play import (
    DefaultPlaybook, RoleStrategy, RoleAssignment, PlayContext, MoveToTarget,
)
from src.soccer_framework import Pose2D


class InterceptorRole(RoleStrategy):
    name = "interceptor"

    def target(self, kit, player_id: int, context: PlayContext) -> Pose2D:
        # Compute a positioning Pose2D, such as on an opponent passing lane.
        ...

    def build_subtree(self, kit, player_id: int):
        return MoveToTarget(
            kit,
            player_id,
            lambda context: self.target(kit, player_id, context),
            reason_fn=lambda: "interceptor hold",
        )


class TacticalPlaybook(DefaultPlaybook):
    def __init__(self, kit):
        super().__init__(kit)
        self.register_role(InterceptorRole())   # Registration order is Selector branch priority.

    def assign_roles(self, context):
        # Mark any player_id as "interceptor" to activate it.
        return RoleAssignment({1: "chaser", 2: "interceptor", 3: "goalkeeper"})


kit = SoccerKit(SoccerConfig())
tree = TeamStrategyTree(kit, TacticalPlaybook(kit), context_provider)
```

For a role with conditional kicking, such as a goalkeeper that comes out to clear
the ball when it enters the penalty area, use `build_attack_subtree` to assemble
"kick when the condition is true, otherwise move to position":

```python
from src.play import AttackSubtreeConfig, RoleStrategy, build_attack_subtree
from src.soccer_framework import Pose2D


class GoalkeeperRole(RoleStrategy):
    """Default goalkeeper guard; actively clears when the ball enters our danger area."""

    name = "goalkeeper"

    def target(self, kit, player_id, context):
        return kit.ready_stance.goalkeeper_guard_target(context.known_ball)

    def wants_to_kick(self, kit, player_id, context):
        return kit.targeting.ball_in_own_defensive_area(context.known_ball)

    def kick_target(self, kit, player_id, context):
        return Pose2D(kit.field.opponent_goal_x(), 0.0, 0.0)

    def build_subtree(self, kit, player_id):
        return build_attack_subtree(
            kit,
            player_id,
            AttackSubtreeConfig(
                target_fn=lambda context: self.target(kit, player_id, context),
                kick_target_fn=lambda context: self.kick_target(kit, player_id, context),
                wants_kick_fn=lambda context: self.wants_to_kick(kit, player_id, context),
                reason_fn=lambda: "goalkeeper guard",
                kick_reason_fn=lambda target: kit.targeting.kick_reason(
                    target,
                    default="goalkeeper clear",
                ),
            ),
        )
```

The behavior tree checks `wants_to_kick` plus `IsInKickRange` to decide whether
the current frame should kick or move to the guard point. The benefit is that the
player's role name on the blackboard remains `"goalkeeper"` throughout. There is
no need to temporarily rewrite the goalkeeper as `"chaser"` in
`Playbook.assign_roles`, and debug logs remain easier to read.

## Documentation

- [docs/developer_protocol.md](docs/developer_protocol.md): simulation
  environment data protocol, including ROS topics, GameController JSON, and
  boosteros control interfaces. Use it if you do not want to use the wrappers in
  `src/soccer_framework`.
- [docs/bt_structure.md](docs/bt_structure.md): detailed behavior tree
  structure.
