# Mapping session, 14 November 2026

## Setup and observations

- The robot is a Raspberry Pi 4 running ROS 2 Humble. A YDLidar X2 is connected
  through `/dev/ttyUSB0` (CP2102 USB serial bridge).
- The Pi workspace contains a LiDAR-only Cartographer mapping launch file.
  Scans were published at about 11.6 Hz, and `/map` was published at 0.05 m
  resolution. The LiDAR and map processes were started without motor nodes.
- The Pi's mapping and encoder processes stopped after a later Pi reboot. They
  were restarted for wheel diagnostics.
- A map snapshot from an early bounded drive is stored at
  [`maps/2026-11-14-pass4.png`](../maps/2026-11-14-pass4.png). Gray means unknown,
  white means scanned free space, and black means a detected boundary. This is
  a partial map, not a completed floor map.

## Drive experiments

`safe_mapping_drive.py` used the front and side LiDAR sectors for short timed
runs. It stops if scans are older than 0.35 seconds, if front scan coverage is
insufficient, or when its time limit expires. Later runs alternated between
turning and short forward motions. Map bounds changed only slightly after the
later runs, so the full floor was not explored.

The Pi's wheel diagnostic used 1.5 second pulses at 20% PWM. With the right
wheel commanded alone, its encoder stayed at zero. The left encoder increased
by roughly 1,000 ticks when the left wheel was commanded. When both were
commanded, only left ticks increased. A 0.75 second pulse to both wheels at
40% PWM also produced no visible wheel motion, according to the person at the
robot. LiDAR pose changes during the pulses were negligible. Treat the left
encoder ticks as unreliable evidence of physical movement until wiring and
signal noise are checked.

The IBT-2 driver current-sense pins (R_S and L_S) are not wired to the Pi, so
there is no separate electrical driver feedback. The Pi reports normal supply
status (`vcgencmd get_throttled` returned `0x0`), but that does not measure the
separate motor battery.

## Next physical checks

1. Confirm the motor battery switch is on and measure voltage across B+ and B-
   on each IBT-2 driver.
2. Confirm a common ground between the battery, both drivers, and the Pi.
3. Confirm driver logic VCC, R_EN, and L_EN receive the expected voltage while
   a bounded motor command is active.
4. Check motor leads and the right encoder's Phase A/B wiring. The documented
   right encoder pins are BCM GPIO 25 and 8.
5. Repeat a short wheel test with the robot observed in person. Resume
   autonomous mapping only after both wheels actually move and their feedback
   is understood.

No passwords or Tailscale credentials are stored in this repository.

## Handoff for the next session

- GitHub `main` contains the mapping notes, bounded drive script, wheel test
  script, and an early map image. The local checkout is
  `/home/spirit/pi_ros2_robot_control`.
- The new laptop's Linux login is `krishna@laptop-o5rgnbqb`; its checkout is
  `/home/krishna/pi_ros2_robot_control`. It was copied before this documentation
  commit and still needs a `git pull` when reachable.
- The Pi is `robot@robot` on Tailscale. Its ROS 2 Humble workspace is
  `/home/robot/robot_ws`. Its active source package has mapping launch and
  Cartographer config files that were not yet copied into the GitHub checkout.
- The latest map saved on the Pi is
  `/home/robot/maps/autonomous_mapping_2026-11-14_latest.yaml` with the matching
  `.pgm`. The image in this repository is an earlier snapshot.
- At the end of the session, the Pi and new laptop were offline on Tailscale.
  Their ROS process state should be checked fresh when they reconnect.
- The person at the robot confirmed that neither wheel physically moved during
  a 40% PWM pulse. Do not infer movement from left encoder ticks. Start the next
  session by checking the separate motor battery voltage at both IBT-2 drivers,
  then the common ground, enables, PWM wiring, and motor leads. Continue mapping
  only after wheel motion is confirmed in person.
