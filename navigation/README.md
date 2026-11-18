# Saved-map navigation — commissioning in progress

**Hardware drive is disabled by default.** The planner and navigator reached
ready state on the Raspberry Pi, but a successful path result, physical pose
alignment, calibration and autonomous route have not yet been verified.

Read [the November 18 handoff](../docs/HANDOFF-2026-11-18.md) before continuing.
It records the live deployment, map backups, remaining work, and safety limits.

## Robot dependencies

ROS 2 Humble, the existing YDLidar driver and Cartographer, Python YAML,
`RPi.GPIO`, and:

```bash
sudo apt-get update
sudo apt-get install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-regulated-pure-pursuit-controller
```

Source ROS and the robot workspace. Stop other mapping/lidar stacks and GPIO
controllers, then launch from this repository:

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
ros2 launch ./navigation/saved_map.launch.py \
  map:=$HOME/maps/manual_floor_20261117_160302.yaml \
  state:=$HOME/maps/manual_floor_20261117_160302.pbstream
```

The deployed copy currently resides at `/home/robot/robot_navigation/`.
Do not pass `hardware:=true` until calibration and supervised commissioning
are complete. The launch validates `robot_settings.json`; missing geometry
or speed measurements prevent hardware activation. The default planning radius
is provisional. Manual 100% PWM settings do not apply to autonomous navigation.

## Desktop panel

Requires Python Tk, Pillow/ImageTk, pexpect and OpenSSH. Run:

```bash
python3 navigation/desktop.py
```

Under WSLg use `DISPLAY=:0`. The panel uses SSH forwarding from local port 18766
to the robot's loopback-only API. First establish/verify the robot's SSH host
key; passwords are prompted, not saved. Override the default robot account with
`ROBOT_SSH_HOST` if required.

Click the map to select a point. **Plan path** only requests a route. A separate
test-start point allows planning without claiming the robot is physically there.
Named destinations are separate coordinate records; the map image is unedited.
The panel shows robot pose and live lidar points for alignment checks.

Setting the robot pose, confirming alignment and arming are distinct operations.
In dry-run mode, Go and Arm remain disabled. Space/Stop cancels and disarms.
The desktop panel implementation is not yet integration-tested or installed on
the laptop; see the handoff for deployment instructions.

## Tests without motion

```bash
python3 -m unittest discover -s navigation -p 'test_*.py' -v
python3 -m unittest discover -s remote_control -p 'test_*.py' -v
python3 -m compileall -q navigation remote_control
```

These tests do not import GPIO, send robot commands, or certify hardware safety.
