# Team Integration Guide

Strategy integration steps:

1. Convert simulator or RoboCup state into `WorldState`.
2. Call `PassStrategy.decide_pass(world, passer_id, config)`.
3. If `should_pass` is true, face `target_point` and command a pass/kick at `pass_speed`.
4. Command the receiver to `target_point`.
5. If false, use the returned reason and component scores to choose dribble or support movement.

The strategy is independent of Webots and can be used from the official RoboCup Demo brain layer once a WorldState adapter is implemented.
