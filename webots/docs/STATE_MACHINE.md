# State Machine

The strategy state machine maps returned actions into attack and defense states.

- PASS -> ATTACK_PASS
- DRIBBLE -> ATTACK_DRIBBLE
- SHOOT -> ATTACK_SHOOT
- BLOCK or MARK_OPPONENT -> DEFEND_MARK
- CHASE_BALL -> SEARCH_BALL
- HOLD or no actionable state -> IDLE/HOLD fallback

Final mock verification returned PASS, DRIBBLE, SHOOT, and BLOCK.

