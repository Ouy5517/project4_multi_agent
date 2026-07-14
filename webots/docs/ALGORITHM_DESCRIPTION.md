# Algorithm Description

For each receiver, the strategy predicts a dynamic receive point:

```text
target = receiver_position + receiver_velocity * lead_time + forward_offset
```

The point is clamped to the field boundary. The pass line is checked by projecting every opponent onto the segment from passer to target. If the projection lies on the segment, the algorithm compares:

- opponent distance to the line;
- estimated ball arrival time at the projection;
- estimated opponent interception time;
- configured time and distance margins.

Unsafe candidates are eliminated before scoring, so high attack score cannot compensate for unsafe receiving or blocked routes.
