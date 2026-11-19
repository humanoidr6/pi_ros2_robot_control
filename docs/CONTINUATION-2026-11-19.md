# November 19 continuation — B4 corridor selected

This supersedes the active-map selection in the November 18 handoff. Keep its
hardware-calibration requirements and original backups.

## Completed today

- Reconnected after the user powered on the robot. Tailscale laptop approval
  completed; remote GUI installation was possible.
- Fixed a startup issue: the default navigate-through-poses tree still depended
  on a disabled `spin` recovery server. Added a matching tree without recoveries.
- Verified the map and navigation lifecycle managers activated all nodes.
- Corrected readiness reporting to require **active lifecycle state**, not just
  advertised action servers. Yesterday's readiness flags alone were insufficient.
- Tested a planner-only route on the earlier map: start `(5.575, -6.825)`, goal
  `(12.275, -3.675)`, 297 points, 7.46 m. Checked 894 interpolated samples against
  original occupancy data: all free, minimum distance to a non-free cell ~0.608 m.
  This is a software path test, not verified physical clearance or movement.
- Confirmed `/goal` rejects disarmed driving, `/arm` rejects dry-run mode,
  and `/stop` returns successfully. No navigation motor output was enabled.
- Added goal generation tracking so Stop invalidates pending goal acceptance;
  late accepted goals are cancelled and old results cannot overwrite a new goal.
- Added seven goal-state regression tests. All 24 tests (15 navigation,
  9 manual remote) passed, plus Python compile and whitespace checks.
- Installed `desktop.py`/`safety.py` on laptop at `~/robot-navigation/`, bundled
  its existing pexpect/ptyprocess, established the 18766 SSH tunnel, created
  **Robot Navigation.cmd** in both Windows Desktop directories, and opened it.

## User's new map

User provided and explicitly selected:

```text
/home/spirit/Downloads/b4_corridor_map.yaml
/home/spirit/Downloads/b4_corridor_map.pgm
```

Image: 605 × 2039 pixels; resolution 0.05 m/pixel; origin `[-9.22, -45.3, 0]`.
This is an approximately 30.25 × 101.95 m bounding rectangle, not a claim that
the entire rectangle is surveyed. Pixel classes are 0, 205 and 254. The image
was inspected but **not cleaned or altered**.

Deployed unchanged to `/home/robot/maps/b4_corridor/`. The previous November 17
map and PBStream remain intact. The B4 image is substantially different and no
matching PBStream was supplied, so reusing the old map-frame localization would
be incorrect without a verified transform.

## New active architecture

The current launch uses `localization:=amcl hardware:=false` and the B4 YAML.
AMCL owns map→odom; Cartographer local lidar odometry owns odom→base_link.
The local lidar configuration loads no PBStream and disables pose-graph loop
closure corrections. The motor gate remains dry-run and disarmed.

Map loading and AMCL startup were verified: both lifecycle nodes are active,
AMCL received the 605 × 2039 map, and `/map` returns the B4 filename and origin.
The global costmap is waiting for map→base_link until the initial pose is set;
the API correctly reports planner/navigator not ready instead of fabricating it.
An initial physical position/heading has **not** been supplied or fabricated.
AMCL and global planning may wait for that initial pose. Select it in the GUI
using the actual robot location, then inspect red scan points against map walls.

Current deployed files: `/home/robot/robot_navigation/`.
Log: `/home/robot/navigation_b4_20261119.log`.
PID file: `/home/robot/robot_navigation/navigation.pid` (inspect live processes;
do not assume a saved PID survives reboot).

The launch was started in a new process group with SIGINT restored to default,
so `kill -INT <launch-PID>` triggers ROS launch's clean shutdown. Earlier shells
with nohup/background jobs inherited ignored SIGINT and left children running;
inspect the process group before restarting, and never launch a second lidar or
conflicting TF publisher. Do not signal unrelated process groups.

## Still pending

1. Initialize actual robot pose on B4 and validate
   localization, then test planning on B4 (the successful path test above used
   the earlier map).
2. Measure width, length, wheel-centre separation and speed. These remain null
   in `robot_settings.json`; `directions_verified=true` preserves the user's
   confirmation of corrected manual controls.
3. Exercise waypoint save/load, cancellation races against real ROS actions,
   and initial-pose behavior. Existing unit tests do not replace integration.
4. Only after geometry/speed and localization checks: supervised low-speed
   physical commissioning. Hardware navigation must not be called ready yet.
5. Map cleanup remains the user's responsibility. Do not rewrite the B4 map.

Robot SSH and laptop SSH identity/credential details remain in the prior
handoff without passwords. No credentials are committed here.
