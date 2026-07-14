# Multi-Agent Role Allocation

Each robot owns an independent `RobotAgent`.

Every 20Hz decision tick:

1. `SharedWorldStateBuilder` creates a single immutable snapshot.
2. `RoleAllocator` assigns team roles from the same snapshot.
3. Each `RobotAgent` observes the snapshot and chooses behavior.
4. `ActionArbitrator` resolves same-team active kick conflicts.
5. All four commands are applied before one MuJoCo physics step.

Roles are dynamic and not equivalent to a single `active_robot` gate. Blue roles include `BALL_HANDLER`, `RECEIVER`, and `SUPPORT`; red roles include `PRESSER`, `COVER`, and `CLEARER`.

The coordinator only shares team intent, such as `PASS_INTENT`; it does not replace per-agent behavior decisions.

