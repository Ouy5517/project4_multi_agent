# Booster Studio Agent Development Rules

This directory is the official Booster Studio 3v3 Soccer match Agent project for
RoboCup 3v3 development.

Rules for Codex:

- Treat this project, its Booster Studio managed Docker simulation environment,
  and the `.agent` file produced by Booster Studio as the formal delivery path.
- Do not edit files under `C:\Users\ztwx4\BoosterStudio\resources` as project
  source. That directory is installation/template material only.
- Keep `src/soccer_framework/` read-only by default. Change it only when the
  official template contract is understood and a focused test or Studio evidence
  requires it.
- Prefer strategy changes in `src/play/` and pure tactical helpers in
  `src/tactics/`.
- Do not reinstall Docker, ROS, or Python globally for this project. Use Docker
  Desktop with the WSL2 backend and Booster Studio's own build/simulation flow.
- Do not delete or prune Docker images, containers, volumes, or WSL distributions
  unless explicitly instructed by the user after backup.
- Before modifying behavior, read the adjacent official files and preserve the
  generated baseline. Use small changes and verify with static checks plus
  Booster Studio build/simulation.
- Final `.agent` artifacts must be produced by Booster Studio `Build Agent Only`,
  not by manual zipping or hand-built packaging.
