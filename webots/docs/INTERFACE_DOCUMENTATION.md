# Interface Documentation

Main interface:

```python
PassDecision decide_pass(WorldState world, str passer_id, PassConfig config)
```

`PassDecision` fields:

- `should_pass`
- `receiver_id`
- `target_point`
- `pass_speed`
- `total_score`
- `risk_level`
- `reason`
- `component_scores`

Fixed-point interface:

```python
PassDecision fixed_point_pass(WorldState world, str passer_id, Point target_point, PassConfig config)
```

Configuration file:

- `/home/plon/Workspace/booster_soccer_project/config/pass_strategy.yaml`
