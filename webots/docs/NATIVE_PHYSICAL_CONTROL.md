# Native Physical Control

Final native path: **N3**.

The project now includes a Webots-native controller that does not use mck, RPC,
FancyKick, VisualKick, or Supervisor ball translation. It inventories Webots
devices, enables PositionSensors, records the initial pose, and drives T1 Motor
targets with smooth interpolation.

## Worlds

- `worlds/T1_native_physical_kick.wbt`: one T1 robot, soccer ball, goals, match monitor, and simplified static WorldState markers.
- `worlds/T1_native_assisted_kick.wbt`: same setup plus transparent torso stabilizers and visible `ASSISTED PHYSICAL KICK` label.

## Runs

- Unassisted run: `results/native_physical_kick/20260712_170618_c747cb`
  - Motor count: 23.
  - PositionSensor count: 23.
  - Stable stand: failed; robot fell during HOLD_STAND.
  - Ball displacement: 0.0 m.

- Assisted run: `results/native_physical_kick/20260712_170644_be6d9b`
  - Motor count: 23.
  - PositionSensor count: 23.
  - Stable stand: not completed within the original script timeout.
  - Robot did not fall at timeout (`z≈0.4843`).
  - No kick/contact occurred before timeout.
  - Ball displacement: 0.0 m.

## Important Boundary

The soccer ball was not moved by Supervisor code. Native physical kick success is
not claimed because no run produced ball displacement greater than 0.05 m.

