# Final Demo Guide

Realtime four-Agent match:

```bash
./scripts/start_final_soccer_demo.sh --match --seed 42 --target-fps 60
```

Stable deterministic showcase:

```bash
./scripts/start_final_soccer_demo.sh --showcase
```

Generate the 60 FPS trajectory replay:

```bash
./scripts/start_final_soccer_demo.sh --record --seed 42
```

Replay the latest generated video:

```bash
./scripts/start_final_soccer_demo.sh --replay
```

The match mode is the true concurrent mode. The showcase mode preserves the deterministic Visual V3 acceptance path.
