# T1 Native Ball Controller

This Webots controller does not start mck or RPC. It inventories all T1 devices,
enables position sensors, holds the initial pose, then uses bounded Webots Motor
position commands to perform a small right-foot physical push sequence.

The controller never writes to `SOCCER_BALL.translation`; ball position is read
from `match_state_monitor` JSONL logs.
