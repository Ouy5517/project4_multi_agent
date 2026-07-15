# API Notes

Real robot action uses the Booster RPC service.

- Change mode / Prepare / Walking: API 2000.
- Move / Stop: API 2001 with bounded velocity.
- GetMode: API 2017.

The final real run did not reach RPC action execution because mck crashed before ready. `GetStatus` API 2018 is not used for final validation.

