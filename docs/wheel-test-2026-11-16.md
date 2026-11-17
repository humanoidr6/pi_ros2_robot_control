# Wheel test, 16 November 2026

The Raspberry Pi was reachable over Tailscale. Before testing, no motor,
mapping, or LiDAR process was running. The Pi's installed encoder node publishes
`std_msgs/Int32` on both encoder topics, matching the diagnostic script on the
Pi. The encoder node was started for this test and stopped afterward.

Each wheel received a separate 0.5 second forward pulse at 40% PWM through
`wheel_diagnostic.py`. The diagnostic stops PWM and disables the IBT-2 drivers
on exit. A final GPIO command set all motor enable and PWM pins low.

| Command | Left encoder change | Right encoder change |
| --- | ---: | ---: |
| Right only | 0 | +39 |
| Left only | 0 | -61 |

Cartographer was not running, so map-frame yaw was unavailable. The person at
the robot observed that the **left physical wheel moved during both pulses**;
the right physical wheel did not move. The right encoder responded during both
single-wheel commands, while the left encoder stayed at zero. This is
inconsistent with the intended motor and encoder channel assignments. The
encoder changes could also include electrical noise, so they do not prove that
the right encoder is attached to the left wheel. Do not resume autonomous
driving until each driver's output, motor battery and ground, and both encoder
channels are checked against actual wheel motion.

The Pi's latest saved map is still from 14 November. Its `.pgm` and `.yaml`,
along with the active Cartographer configuration and mapping launch file, were
copied into this repository for reference. The local repository has older
control sources than the Pi's active ROS package; use the Pi's installed
package when diagnosing live hardware.

## Follow-up I2C scan

The user subsequently reported that the wheels are not moving. Physical motion
therefore remains unconfirmed despite the earlier observation above. Installed
`i2c-tools` on the Pi and scanned bus 1: no addresses responded. The IBT-2
drivers and encoder signals use GPIO and cannot be detected by an I2C scan.
SPI is enabled in the boot configuration and uses pins overlapping the robot's
GPIO assignments, but GPIO function readback showed those pins as inputs at
the time of inspection; a live pin conflict was not established. Motor supply
voltage at each IBT-2 still requires a physical measurement. No motor pulse was
issued during this follow-up.

## Both wheels pulse

At the user's request, commanded both wheels at 40% PWM for 0.5 seconds.
Encoder deltas were left 0 and right -30 (right count -1 to -31).
Physical movement awaits the user's observation. Diagnostic and encoder
processes exited; all eight motor GPIO pins were explicitly set low and
read back as zero before cleanup.
