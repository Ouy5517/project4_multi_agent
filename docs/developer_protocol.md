# Soccer Development Protocol

This document summarizes the data topics, GameController JSON messages, and
boosteros control interfaces used by the current 3v3 simulation soccer match.
If developers do not want to use the wrappers provided by
`src/soccer_framework`, they can implement their own integration by following
the protocol described here.

## 1. Match Identity and Team Member Conventions

During official matches, environment variables specify the current Agent's team
identity and the list of own robots:

| Environment variable | Meaning | Default match value |
|---|---|---|
| `SOCCER_TEAM_ID` | Team number that the current Agent belongs to | team1 is `1`; team2 is `2` |
| `SOCCER_ROBOT_NAMES` | Comma-separated own robot names controlled by the current Agent | team1 is `robot1,robot2,robot3`; team2 is `robot4,robot5,robot6` |

The order of `SOCCER_ROBOT_NAMES` determines the strategy-layer `player_id`: the
first robot is `player_id=1`, the second is `player_id=2`, and the third is
`player_id=3`. Runtime uses these values to derive `/teamN/...` ground-truth
topics, own `RobotState` mappings, and `BoosterRobot.virtual_robot_name`.

Participants who modify the code must keep this convention:

- Do not hard-code the current team number in strategy or runtime code.
- Do not hard-code own robots as always being `robot1,robot2,robot3`.
- When the team number or own members are needed, read them from the
  configuration corresponding to `SOCCER_TEAM_ID` and `SOCCER_ROBOT_NAMES`.
- Opponent robot positions may be read, but this Agent can only control the own
  robots listed in `SOCCER_ROBOT_NAMES`.

## 2. Team-View Ground Truth Data

Simulation ground truth is published under `/teamN/...` topic prefixes in the
team field coordinate frame. Here, `N` is the team number, usually `1` or `2`.
Robot poses and ball coordinates under `/teamN` are already expressed relative
to that team's field coordinate frame. Simulation strategies can subscribe to
and use this ground truth directly; participants do not need to do another
coordinate transform.

Team field coordinate frame definition:

- Each team has its own team field coordinate frame. Robots and the ball under
  the same `/teamN` topic prefix use the same team coordinate frame.
- The field center is `(0, 0)`. The default M-Field size is `14.0m x 9.0m`.
- For every team, the own goal is fixed at `x=-7.0`, and the opponent goal is
  fixed at `x=+7.0`.
- `+x` always means this team's attacking direction, and `-x` always means this
  team's defensive direction.
- The own half is `x <= 0`, and the opponent half is `x >= 0`.
- `robot_pose.theta` is the robot heading in radians; `theta=0` means facing the
  `+x` attacking direction.
- The ball topic reuses `geometry_msgs/msg/Pose2D`; currently only `x` and `y`
  are read, while `theta` is ignored.

This is different from an absolute field coordinate frame. In an absolute field
coordinate frame, team1 and team2 may share the same fixed field origin and
direction, so one team's attacking direction may be `+x` while the other's may
be `-x`. The `/teamN/...` ground truth is not exposed to strategies that way: it
has already been normalized from the team's perspective. For example, if the
same physical ball is in team1's attacking half, it may appear as `x>0` in
`/team1/.../ball`; for team2 this is its defensive half, and the ball is still
expressed from team2's perspective under `/team2/.../ball`. The strategy only
needs to remember that `ball.x > 0` means the ball is on the team's attacking
side, and `ball.x < 0` means it is on the team's defensive side.

Therefore, the strategy layer should not mirror coordinates again based on
`team_id`, half, or topic prefix. If the input source changes to absolute field
coordinates in the future, mirroring or rotation normalization should happen in
an adapter layer such as `PlayContextProvider`. Once data enters `PlayContext`,
it should still follow the team field coordinate convention above.

### Topic List

3v3 uses 6 robots by default. Each team view publishes poses for all 6 robots
and one team-level soccer ball position topic.

| Team view | Topic | Message type | Description |
|---|---|---|---|
| team1 | `/team1/robot1/soccer/sim/ground_truth/robot_pose` | `geometry_msgs/msg/Pose2D` | Pose of `robot1` from the team1 view |
| team1 | `/team1/robot2/soccer/sim/ground_truth/robot_pose` | `geometry_msgs/msg/Pose2D` | Pose of `robot2` from the team1 view |
| team1 | `/team1/robot3/soccer/sim/ground_truth/robot_pose` | `geometry_msgs/msg/Pose2D` | Pose of `robot3` from the team1 view |
| team1 | `/team1/robot4/soccer/sim/ground_truth/robot_pose` | `geometry_msgs/msg/Pose2D` | Pose of `robot4` from the team1 view |
| team1 | `/team1/robot5/soccer/sim/ground_truth/robot_pose` | `geometry_msgs/msg/Pose2D` | Pose of `robot5` from the team1 view |
| team1 | `/team1/robot6/soccer/sim/ground_truth/robot_pose` | `geometry_msgs/msg/Pose2D` | Pose of `robot6` from the team1 view |
| team1 | `/team1/soccer/sim/ground_truth/ball` | `geometry_msgs/msg/Pose2D` | Soccer ball position from the team1 view |
| team2 | `/team2/robot1/soccer/sim/ground_truth/robot_pose` | `geometry_msgs/msg/Pose2D` | Pose of `robot1` from the team2 view |
| team2 | `/team2/robot2/soccer/sim/ground_truth/robot_pose` | `geometry_msgs/msg/Pose2D` | Pose of `robot2` from the team2 view |
| team2 | `/team2/robot3/soccer/sim/ground_truth/robot_pose` | `geometry_msgs/msg/Pose2D` | Pose of `robot3` from the team2 view |
| team2 | `/team2/robot4/soccer/sim/ground_truth/robot_pose` | `geometry_msgs/msg/Pose2D` | Pose of `robot4` from the team2 view |
| team2 | `/team2/robot5/soccer/sim/ground_truth/robot_pose` | `geometry_msgs/msg/Pose2D` | Pose of `robot5` from the team2 view |
| team2 | `/team2/robot6/soccer/sim/ground_truth/robot_pose` | `geometry_msgs/msg/Pose2D` | Pose of `robot6` from the team2 view |
| team2 | `/team2/soccer/sim/ground_truth/ball` | `geometry_msgs/msg/Pose2D` | Soccer ball position from the team2 view |

The current `RosTruthProvider` reads both own robots and opponent robots under
the same `/teamN` topic prefix. For example, team1 runtime subscribes by default
to `/team1/robot1..robot6/.../robot_pose` and
`/team1/soccer/sim/ground_truth/ball`; team2 runtime subscribes by default to
`/team2/robot1..robot6/.../robot_pose` and
`/team2/soccer/sim/ground_truth/ball`.

### Message Structure

`geometry_msgs/msg/Pose2D`:

```text
float64 x
float64 y
float64 theta
```

Robot pose mapping to the strategy layer:

```text
Pose2D.x     -> RobotState.pose.x
Pose2D.y     -> RobotState.pose.y
Pose2D.theta -> RobotState.pose.theta
```

Ball position mapping to the strategy layer:

```text
Pose2D.x -> BallState.x
Pose2D.y -> BallState.y
theta    -> ignored
```

### Subscription Example

```python
from geometry_msgs.msg import Pose2D
from rclpy.node import Node


class TruthDebugNode(Node):
    def __init__(self) -> None:
        super().__init__("truth_debug")
        self.create_subscription(
            Pose2D,
            "/team1/robot1/soccer/sim/ground_truth/robot_pose",
            self._robot_cb,
            1,
        )
        self.create_subscription(
            Pose2D,
            "/team1/soccer/sim/ground_truth/ball",
            self._ball_cb,
            1,
        )

    def _robot_cb(self, msg: Pose2D) -> None:
        self.get_logger().info(
            f"robot1 x={msg.x:.2f} y={msg.y:.2f} theta={msg.theta:.2f}"
        )

    def _ball_cb(self, msg: Pose2D) -> None:
        self.get_logger().info(f"ball x={msg.x:.2f} y={msg.y:.2f}")
```

## 3. Referee Message `/soccer/game_controller`

Referee state is published through the ROS2 topic `/soccer/game_controller`.
The message type is `std_msgs/msg/String`, and the `data` field contains a JSON
representation with GameController v19 semantics. The current template
subscribes only to this ROS topic and does not directly consume the
GameController UDP binary packet.

Runtime parses JSON through `game_control_state_from_json()` to obtain a
`GameControlState`. If no fresh referee topic is received for more than
2 seconds, the control loop stops the robots.

### Topic and ROS Message

| Topic | Message type | Description |
|---|---|---|
| `/soccer/game_controller` | `std_msgs/msg/String` | `data` is the GameController v19 JSON payload |

`std_msgs/msg/String`:

```text
string data
```

### JSON Example

```json
{
  "version": 19,
  "packetNumber": 1024,
  "playersPerTeam": 3,
  "competitionType": "MIDDLE",
  "stopped": false,
  "gamePhase": "NORMAL",
  "state": "PLAYING",
  "setPlay": "NONE",
  "firstHalf": true,
  "kickingTeam": 255,
  "secsRemaining": 540,
  "secondaryTime": 0,
  "teams": [
    {
      "teamNumber": 1,
      "fieldPlayerColour": 0,
      "goalkeeperColour": 0,
      "goalkeeper": 3,
      "score": 0,
      "penaltyShot": 0,
      "singleShots": 0,
      "messageBudget": 0,
      "players": [
        {
          "penalty": "NONE",
          "secsTillUnpenalised": 0,
          "warnings": 0,
          "cautions": 0
        }
      ]
    },
    {
      "teamNumber": 2,
      "fieldPlayerColour": 1,
      "goalkeeperColour": 1,
      "goalkeeper": 3,
      "score": 0,
      "penaltyShot": 0,
      "singleShots": 0,
      "messageBudget": 0,
      "players": [
        {
          "penalty": "NONE",
          "secsTillUnpenalised": 0,
          "warnings": 0,
          "cautions": 0
        }
      ]
    }
  ]
}
```

The `players` arrays above only show the first player on each team. The actual
message contains the full player array for each team, and the strategy uses
`players[player_id - 1]`.

### Top-Level Fields

| Field | Type | Meaning |
|---|---|---|
| `version` | int | GameController protocol version. Currently `19`. |
| `packetNumber` | int | Referee message sequence number, useful for checking whether messages keep updating. |
| `playersPerTeam` | int | Number of field players per team. Usually `3` in 3v3. |
| `competitionType` | string | Competition division. Current enum values: `SMALL`, `MIDDLE`, `LARGE`. |
| `stopped` | bool | Stop/ball-placement state under `state=PLAYING`. When `true`, robots should not continue obvious movement. |
| `gamePhase` | string | Match phase. Current enum values: `NORMAL`, `PENALTY_SHOOT_OUT`, `EXTRA_TIME`, `TIMEOUT`. |
| `state` | string | Main match state. Current enum values: `INITIAL`, `READY`, `SET`, `PLAYING`, `FINISHED`. |
| `setPlay` | string | Current set play or restart type. Current enum values: `NONE`, `DIRECT_FREE_KICK`, `INDIRECT_FREE_KICK`, `PENALTY_KICK`, `THROW_IN`, `GOAL_KICK`, `CORNER_KICK`. |
| `firstHalf` | bool | Whether this is the first half. Current strategy does not depend on changing field orientation by half. |
| `kickingTeam` | int | Team number for the current kick-off or set play. `255` means no team. |
| `secsRemaining` | int | Main match seconds remaining. |
| `secondaryTime` | int | Auxiliary countdown for the current phase or set play, such as Ready, Set, the kick-off ownership window, or the restart window. |
| `teams` | array | Array of two team states. Each element is a `TeamState` JSON object. |

### `teams[]` Fields

| Field | Type | Meaning |
|---|---|---|
| `teamNumber` | int | Team number, usually `1` or `2`. |
| `fieldPlayerColour` | int | Field player color id, interpreted by GameController or the simulator. |
| `goalkeeperColour` | int | Goalkeeper color id, interpreted by GameController or the simulator. |
| `goalkeeper` | int | Goalkeeper player number, represented as a 1-based player id. |
| `score` | int | Current score. |
| `penaltyShot` | int | Penalty-shot related count, retained from GameController v19. |
| `singleShots` | int | Single-shot related count, retained from GameController v19. |
| `messageBudget` | int | Team communication budget field. The current strategy does not use it. |
| `players` | array | Player state array for this team, accessed by `player_id - 1`. |

### `teams[].players[]` Fields

| Field | Type | Meaning |
|---|---|---|
| `penalty` | string | Current penalty type for the player. `NONE` means the player is not penalized. |
| `secsTillUnpenalised` | int | Seconds remaining until the penalty is lifted. A positive value means the player should be treated as unavailable. |
| `warnings` | int | Warning count. |
| `cautions` | int | Caution count. |

The current template recognizes these `penalty` enum values:

```text
NONE
ILLEGAL_POSITIONING
MOTION_IN_SET
LOCAL_GAME_STUCK
INCAPABLE_ROBOT
PICKED_UP
BALL_HOLDING
LEAVING_THE_FIELD
PLAYING_WITH_ARMS_HANDS
PUSHING
SENT_OFF
SUBSTITUTE
```

The strategy layer usually only needs to check:

- `state`: decides whether the current phase is Ready, Set, Playing, or finished.
- `stopped`: whether movement must pause during `PLAYING`.
- `setPlay` + `kickingTeam` + `secondaryTime`: decides whether this is our or
  the opponent's kick-off, set play, or restart.
- `teams[].score`: current score.
- `teams[].players[player_id - 1].penalty` and `secsTillUnpenalised`: decide
  whether an own player can participate.

### Subscription Example

```python
import json

from rclpy.node import Node
from std_msgs.msg import String


class GameControllerDebugNode(Node):
    def __init__(self) -> None:
        super().__init__("gc_debug")
        self.create_subscription(
            String,
            "/soccer/game_controller",
            self._gc_cb,
            10,
        )

    def _gc_cb(self, msg: String) -> None:
        payload = json.loads(msg.data)
        self.get_logger().info(
            "state=%s stopped=%s setPlay=%s kickingTeam=%s"
            % (
                payload.get("state"),
                payload.get("stopped"),
                payload.get("setPlay"),
                payload.get("kickingTeam"),
            )
        )
```

## 4. boosteros Control Interface List

The current project centralizes boosteros access in `TeamRobotManager` and
`PlayerKickStateMachine` in `src/soccer_framework/robot.py`. Strategy code
should not call boosteros directly. The strategy should only output
`RobotCommand`; runtime dispatches it uniformly.

Detailed official boosteros interface documentation:
[boosteros interface documentation](https://booster.feishu.cn/wiki/FV4SwjEeXiGJ1wkZJEacT3kCniQ).

| Class/interface | Current usage | Brief description |
|---|---|---|
| `BoosterRobot(virtual_robot_name=..., enable_tf_listener=False, timeout=10.0)` | Created once for each `player_id` at startup | `virtual_robot_name` comes from `SOCCER_ROBOT_NAMES` or the ROS parameter `robot_names`; when `default` is written, runtime passes an empty string. |
| `BoosterRobot.list_gaits()` | Kept as a hardware-adapter helper interface | Can be used to inspect gait names exposed by the SDK. |
| `BoosterRobot.set_gait(gait)` | Called when a walking command is needed in READY/PLAYING and the robot is not currently in walk mode | Switches to the soccer gait. |
| `BoosterRobot.set_mode("walk")` | Called when a walking command is needed in READY/PLAYING and the robot is not currently in walk mode | Switches to walk mode. |
| `BoosterRobot.get_mode()` | Runtime reads this SDK snapshot during READY/PLAYING control ticks | Checks whether the current mode is still walk; the referee may asynchronously switch the robot to prepare, so movable phases need to detect that. |
| `BoosterRobot.get_fall_down_state()` | Runtime reads this SDK snapshot in READY/PLAYING when not in walk mode | Reads fall-down state. The current code uses the returned object's `state` and `recoverable` fields. Walk mode is treated as normal by default. |
| `BoosterRobot.get_up()` | Called during fall-down recovery | Triggers get-up. The manager throttles retries internally to 1 second. |
| `BoosterRobot.set_velocity(vx, vy, vyaw)` | Normal move/stop command | Biped chassis velocity interface. The current navigation layer mainly uses `vx` and `vyaw`; lateral `vy` is usually kept at `0`. |
| `SoccerKickManager(robot)` | Created once for each robot at startup | boosteros automatic kicking manager. It shares the chassis control channel with `set_velocity`. |
| `SoccerKickManager.start()` | Called when entering a kick intent | Starts automatic kick control. The project keeps a minimum active kick duration to avoid frequent start/stop cycles. |
| `SoccerKickManager.update_command(direction, power)` | Updated on each control tick while kicking | `direction` is the kick direction in the robot body coordinate frame. `power` is clamped to `[1.0, 10.0]`. |
| `SoccerKickManager.update_ball(x, y)` | Updated on each control tick while kicking | `x` and `y` are the ball position in the robot body coordinate frame, not field coordinates. |
| `SoccerKickManager.stop()` | Called when leaving kick intent or force-stopping | Stops automatic kick control and releases the chassis back to `set_velocity`. |

Control channel constraints:

- At any instant, one robot's chassis can be controlled by either `set_velocity`
  or `SoccerKickManager`, not both.
- A normal move command first attempts to stop `SoccerKickManager`; if the robot
  is still within the minimum active kick duration, `set_velocity` is skipped for
  the current frame.
- Stop semantics such as penalization, GameController staleness, and runtime
  closing output zero velocity and forcibly release the kick channel when needed.
- Before entering `SoccerKickManager`, the strategy approaches the ball in
  team-view field coordinates. After entry, runtime transforms the ball and
  target direction into the robot body coordinate frame before calling
  `update_ball()` and `update_command()`.
