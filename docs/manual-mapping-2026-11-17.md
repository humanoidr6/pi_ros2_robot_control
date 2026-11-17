# Manual mapping session — 17 November 2026

## Wheel tests and direction correction

The operator confirmed the wheels were raised before testing. Each motor was
commanded separately at 40% PWM for 1.5 seconds. Right-only produced 1,328 left
encoder edges and 137 right edges; left-only produced 1,519 left edges and 96
right edges. Both channels were quiet in the disabled interval after each pulse.
The operator confirmed both wheels moved. Encoder channel correctness remains
unverified; these measurements are not calibrated wheel speed or odometry.
The existing encoder node also crashed during shutdown in an earlier test.

After placing the robot on the floor, the operator authorized mapping and later
confirmed stairs/drop-offs were blocked. Lidar-only Cartographer ran at 5 cm map
resolution with scans around 11.45 Hz. A 20-second guarded drive completed.
A subsequent longer run was interrupted after about 30 seconds because repeated
clearances suggested circling. SLAM reported an approximately 0.51 × 0.54 m
trajectory extent and 570 degrees of signed rotation during that interval.
This was not complete floor coverage.

The operator chose manual mapping. A desktop GUI and SSH/GPIO bridge were
installed, with a live map, hold-to-drive controls, emergency stop, and saving.
At the operator's request, PWM was increased to a 100% maximum and initial
selection. The operator reported that Left acted as Reverse and Reverse acted
as Left. Right-wheel inversion was enabled in the GUI to correct that pattern;
the mapping logic passed simulated direction tests. A subsequent physical
direction verification was not recorded. The legacy autonomous controller's
polarity has not been corrected by this GUI setting.

## Laptop handover

The controller was transferred to the Ubuntu WSL environment on
`laptop-o5rgnbqb`, with a Windows desktop launcher. The previous computer's GUI
was stopped before handover. SSH connectivity, live map retrieval, and the
laptop GUI startup were verified. No passwords, SSH keys, or authentication
links are included in this repository.

## Saves verified on the robot

Paths below are **remote artifacts**, not files bundled in this commit:

| Base path under `/home/robot/maps/` | Verified formats |
| --- | --- |
| `floor_20261117_153047` | PGM, YAML |
| `floor_20261117_153047_pass1` | PGM, YAML, PBStream |
| `floor_20261117_153446_pass1` | PGM, YAML, PBStream |
| `manual_floor_20261117_154219` | PGM, YAML, PBStream |

The last confirmed save was at 15:42 IST. Later saves made through the GUI have
not been inspected. When asked to check saved data, both robot and laptop were
offline on Tailscale. The November 17 files could not be retrieved for this
repository update. The November 14 map already available locally is retained
as a historical artifact; it is not the latest manual map.

## Remaining work

Reconnect the robot, copy and inspect the newest map files, verify the corrected
physical wheel directions at low PWM, and diagnose the encoder channel/noise
issue before relying on wheel odometry. Manual mapping is available; autonomous
complete-floor exploration and coverage verification are not implemented.
