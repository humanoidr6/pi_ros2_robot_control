# Manual mapping remote

Tk desktop controls connect to a loopback-only bridge on the Raspberry Pi through
an SSH tunnel over Tailscale. The GUI includes a live occupancy map, hold-to-drive
buttons and keyboard controls, a latched emergency stop, and map saving.

## Computer or Ubuntu WSL laptop

Install Python 3, OpenSSH client, Tk, Pillow (including ImageTk), and pexpect.
On Ubuntu the Python packages are `python3-tk`, `python3-pil`,
`python3-pil.imagetk`, and `python3-pexpect`.

From this checkout:

```bash
./remote_control/start-remote.sh
```

The default target is `robot@100.118.233.95`. Override it with
`ROBOT_SSH_HOST=user@tailscale-host`. First establish and verify an SSH connection
to that target so its host key is in `~/.ssh/known_hosts`. The GUI requires a
known host key and prompts for the robot password when needed; it does not save
passwords. It uses local port 18765 and an SSH control socket under `~/.ssh`.

WSLg must be available to show the GUI on Windows. If this repository is at
`~/pi_ros2_robot_control` in the Ubuntu distro, a Windows `.cmd` launcher can use:

```bat
@echo off
wsl.exe -d Ubuntu -- bash -lc "exec ~/pi_ros2_robot_control/remote_control/start-remote.sh"
```

Only one GUI instance is allowed per Linux user on a computer. Close controllers
on other computers before transferring control; the bridge does not arbitrate
multiple operators.

## Raspberry Pi

Requires the existing ROS 2 Humble robot workspace, `RPi.GPIO`, YDLidar driver,
Cartographer, and `nav2_map_server`. Source ROS and the robot workspace, then
start the lidar-only mapping launch:

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
ros2 launch robot_control mapping.launch.py
```

In another sourced shell, run this repository's `remote_control/bridge.py`:

```bash
python3 remote_control/bridge.py
```

The repository is a source snapshot with control modules at its root; it is not
a complete standalone colcon package (the existing Pi workspace supplies the
package manifest/resource and Python package layout). Install the mapping launch
and `config/lidar_2d.lua` into that workspace when updating it. `setup.py`
includes the config data-file rule for that layout.

Do not run `robot.launch.py`, `motor_controller`, wheel diagnostics, or an
autonomous drive script alongside the bridge. The bridge owns the motor GPIOs.
Its lock prevents a second bridge, but legacy scripts do not share that lock.

## Controls and limits

- Hold an arrow button or W/A/S/D / arrow key. Release to stop.
- Space or the red button latches emergency stop; Reset clears the latch.
- PWM is adjustable from 10–100%, initially **100% at the owner's request**.
- Right-wheel inversion is on; left-wheel inversion is off. This corrects the
  reported Left/Reverse mismatch. Check the physical directions at low PWM.
- GUI focus loss, stale GUI heartbeat, and closing the window stop commands.
- The bridge watchdog stops PWM and disables drivers after 350 ms without fresh
  commands. It also blocks movement with lidar older than 400 ms, insufficient
  valid returns, or any detected obstacle within 50 cm.
- These software checks do not guarantee stopping distance, particularly at
  100% PWM. Lidar cannot reliably detect stairs/drop-offs. Use physical barriers
  and retain access to the power cutoff.
- The live map is white for free, black for occupied, and grey for unknown space.
  It is not a full-floor coverage indicator.

## Saved data and map editing

Save Map stops the robot and writes timestamped files on the Pi under
`~/maps/manual_floor_YYYYMMDD_HHMMSS`:

- `.pgm`: occupancy image;
- `.yaml`: image reference, resolution, origin, and thresholds;
- `.pbstream`: Cartographer state, when its write-state service succeeds.

Back up the files before editing the PGM. Preserve its dimensions, origin, and
resolution; update the YAML image name if renaming the image. Editing the PGM
does not change the PBStream. Do not paint real obstacles as free space. The GUI
shows the live SLAM map, not an edited file on disk.

## Checks without motor movement

```bash
python3 -m unittest discover -s remote_control -p 'test_*.py' -v
```

Tests use simulated GUI commands and pure safety logic, without SSH or GPIO.
