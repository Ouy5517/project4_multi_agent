# NAO-Inspired Visual Proxy Design

The Visual V2 robots are NAO-inspired compact humanoid visual proxies built only from MuJoCo primitives.

Important accuracy note:

- No official NAO mesh is used.
- No official NAO model is downloaded.
- The robot remains the accepted assisted MuJoCo proxy, not an official NAO asset.
- The physical kinematic scale is preserved so the successful foot-ball collision geometry stays valid.

The visual shell adds rounded parts around the accepted proxy:

- Ellipsoid head shell with deep navy visor
- Short dark neck
- Rounded white torso shell with team-color chest panel
- Dark pelvis/waist shell
- Rounded shoulder shells
- White upper-arm shells and dark forearms
- White thigh and shin shells
- Dark knee shells
- Wide white visual feet aligned with the physical foot proxies

Team panels:

- `T1_BLUE_1`: deep blue
- `T1_BLUE_2`: bright blue
- `T1_RED_1`: deep red
- `T1_RED_2`: orange red

All Visual V2 shell geoms are presentation-only:

```xml
contype="0"
conaffinity="0"
group="2"
density="0"
```

The `density="0"` setting is required because MuJoCo can include non-colliding geoms in body mass and inertia. Keeping density at zero ensures the visual shell does not change the already accepted physics behavior.

The successful baseline model remains at:

```text
mujoco_soccer/models/t1_2v2_soccer.xml
```

The independent Visual V2 model is:

```text
mujoco_soccer/models/t1_2v2_soccer_visual_v2.xml
```

