# Final Acceptance

Run:

```bash
./scripts/run_final_acceptance.sh
```

The output directory is `results/final_acceptance/<run_id>/` and includes:

- `final_acceptance.json`
- `concurrency_acceptance.json`
- `frontend_acceptance.json`
- `physics_integrity.json`
- `motion_quality.json`
- `goal_visibility.json`
- `video_smoothness_report.json`
- `video_provenance.json`
- `summary.json`
- `final_frame.png`
- `contact_sheet.png`
- `checksums.txt`

`final_release_success=true` requires pytest, physics integrity, four-Agent concurrency, physical ball integrity, realtime present metrics, goal visibility, motion quality, 60 FPS video smoothness, and provenance checks.
