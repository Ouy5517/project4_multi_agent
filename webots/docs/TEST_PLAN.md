# Test Plan

Automated tests cover:

- open teammate;
- opponent near receive point;
- static pass-line blocker;
- fast opponent cutting into line;
- fixed-point pass blocked;
- multiple teammate competition;
- no safe candidate;
- receive point clamping;
- too close and too far targets;
- dynamic teammate receiving;
- emergency fallback;
- multiple-opponent blockade;
- weight-change selection.

Run:

```bash
cd /home/plon/Workspace/booster_soccer_project
source coop_env/bin/activate
pytest -q
```
