# Possession And Arbitration

`PossessionManager` tracks ball ownership from physical foot-ball contacts, robot distance to the ball, and contested proximity.

States:

- `FREE`
- `BLUE_CONTROL`
- `RED_CONTROL`
- `CONTESTED`

Same-team arbitration is handled by `ActionArbitrator`. Each team may have only one active kick lock for active foot-ball behaviors such as `DRIBBLE`, `PASS`, `SHOOT`, `CLEAR`, or `COUNTER_ATTACK`. Opposing teams may still contest the ball simultaneously.

This preserves real ball physics: the ball moves through contact with foot proxy geoms only.

