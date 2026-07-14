# Pass Strategy Design

`PassStrategy.decide_pass(world, passer_id, config)` evaluates each teammate candidate.

Hard eliminations:

- Receive point outside field safety margin.
- Pass distance too short or too long.
- Opponent inside receive safety radius.
- Opponent can intercept the pass line.

Scores:

- Distance score.
- Receive safety score.
- Space score.
- Line score.
- Forward advance score.
- Attack value score.

The selected pass must pass all hard constraints unless emergency mode is explicitly enabled.
