# Final Report

This project implements the pass-strategy core for a Booster T1 multi-robot soccer course project.

The completed software models the world state, evaluates teammate pass candidates, applies hard receive safety constraints, checks pass-line obstruction, predicts opponent interception, computes dynamic receive points, and emits explainable decisions.

The system architecture separates strategy from robot control. This allowed testing the algorithm even though Webots Runner integration is blocked by missing official `/opt/booster/configs` YAML files.

Testing includes 18 automated unit and scenario tests. Experiments compare nearest teammate, soft-score-only, and full safe-pass strategies. Results show the full strategy refuses unsafe passes that the simpler baselines still attempt.

Current limitation: Webots/RPC/SDK closed-loop control is not verified because the official configuration files required by `mck` are absent from the local packages.

Future work:

- obtain the official `booster_config` package;
- restore `/opt/booster/configs`;
- run Webots time-progression and SDK locomotion tests;
- connect RoboCup Demo brain state to `WorldState`;
- execute physical or simulated passing trials.
