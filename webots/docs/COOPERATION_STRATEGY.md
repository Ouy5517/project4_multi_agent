# Cooperation Strategy — Booster T1 2v2 Soccer

## Architecture

```
WorldState → Strategy.decide() → [RobotAction] → Adapter.execute()
```

## Roles

| Role | Description | Assigned To |
|------|-------------|-------------|
| BALL_HANDLER | Nearest to ball, primary actor | BLUE_1 |
| SUPPORT | Positioned for receive/pass | BLUE_2 |
| MARK | Mark opponent ball handler | RED_1 |
| BLOCK | Block passing lanes | RED_2 |

## Decision Flow

1. `WorldState` parsed from match_state_monitor
2. `TeamStrategy.decide(state)` evaluates:
   - Who has ball?
   - Is pass line clear?
   - Is opponent within marking range?
3. Decisions: DRIBBLE, PASS, HOLD, MARK, BLOCK

## Pass Logic (verified via unit tests)

- Check `is_pass_line_clear(ball, receiver, opponents)`
- If blocked by opponent → switch to DRIBBLE or HOLD
- Pass target: teammate nearest to receiver position

## Kick (simple_kick_executor)

- OBSERVE → MOVE_BEHIND_BALL → ALIGN → APPROACH → SHORT_FORWARD_PUSH → STOP
- Uses Move API only (no FancyKick/VisualKick)
- Ball moved by physics collision
# Final Cooperation Strategy - 2026-07-12

The strategy engine uses unified `WorldState` and real `TeamStrategy.decide()` calls.

Verified mock scenarios:

- Scenario A: safe receiver and clear pass lane -> PASS.
- Scenario B: `T1_RED_2` blocks the pass lane -> DRIBBLE.
- Scenario C: ball handler near red goal -> SHOOT.
- Scenario D: defensive threat with no blue carrier -> BLOCK.

In real mode the same strategy path is prepared, but the final mck run crashed before the physical dribble script could execute RPC actions.

# MuJoCo Concurrent Match Strategy - 2026-07-13

The `concurrent-match` mode replaces the single active-robot demonstration sequence with four independent robot agents that decide on the same simulation tick.

Concurrent decision flow:

1. Build one immutable `SharedWorldState` snapshot.
2. Allocate team roles for both blue and red teams.
3. Let `T1_BLUE_1`, `T1_BLUE_2`, `T1_RED_1`, and `T1_RED_2` each choose a behavior from that same snapshot.
4. Run same-team kick arbitration so only one teammate can issue an active kick on a tick.
5. Apply all four robot commands before one shared `mujoco.mj_step()`.

The concurrent roles include ball handler, supporter, presser, interceptor, blocker, and passing option. Logged behaviors include `PASS`, `SHOOT`, `CLEAR`, `INTERCEPT_BALL`, `PRESS_BALL`, `BLOCK_LINE`, and `OPEN_FOR_PASS`.
