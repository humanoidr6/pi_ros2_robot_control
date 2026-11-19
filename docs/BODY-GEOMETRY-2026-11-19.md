# CAD body dimensions — 19 November 2026

The user supplied `/home/spirit/Downloads/assem1_description` as the robot body
model. Dimensions below come from every collision STL transformed through the
URDF joint tree into its `base_link`, with all joint positions zero. The source
model was not modified and its ros2_control mock hardware was not launched.

| Quantity | CAD-derived value |
| --- | ---: |
| Overall length (X) | 0.450000 m |
| Overall width including tyres (Y) | 0.419600 m |
| Overall height (Z) | 0.227921 m |
| Wheel radius | 0.045000 m |
| Tyre centreplane separation | 0.384600 m |
| Wheel joint-origin Y separation | 0.378600 m |
| Axle midpoint X relative to CAD base_link | +0.110750 m |

Tyre centreplanes, rather than joint reference origins, are used for
`wheel_base_m`. The two wheel meshes have their tread centres displaced 3 mm
outward from the joint origins. Their X and Z positions also differ by about
3 mm in the supplied export; these are model measurements, not a calibrated
effective track width under motion.

The complete zero-pose collision bounds in CAD base_link are approximately:

```text
minimum: [-0.225, -0.2098, -0.134921] metres
maximum: [ 0.225,  0.2098,  0.093000] metres
```

For a navigation base located at the drive-axle midpoint, the bounding footprint
is asymmetric:

```text
[[-0.33575, -0.2098], [0.11425, -0.2098],
 [ 0.11425,  0.2098], [-0.33575, 0.2098]]
```

Its circumscribed radius is about 0.39591 m. The provisional planning radius is
rounded up to **0.40 m**. A circle computed only from half the width and length
would miss the offset between the CAD origin and the drive axle.

## Configuration and verification

- `navigation/robot_settings.json` now contains the CAD width, length, track,
  height, wheel radius and axle-centred footprint.
- `base_frame_verified=false` deliberately remains set. The model has no lidar
  link, and the current runtime laser transform is an unverified placeholder.
- Once the runtime base is confirmed at the axle midpoint and the lidar offset
  is measured/configured, `base_frame_verified=true` enables use of the explicit
  footprint, with 2 cm padding. Do not set this flag merely to bypass the check.
- Measured speed at full PWM is still unknown. CAD does not supply a reliable
  PWM-to-speed calibration. Autonomous hardware remains blocked.
- The source contains revolute front-wall/mount joints. These bounds describe
  the zero pose; any deployed moving panel or additional hardware needs its
  swept geometry included before navigation.
- Several exported masses/inertias are implausibly small. This extraction uses
  collision vertices, not those inertia values; dynamics are not validated.

The exact report, per-link bounds and SHA-256 hashes of the URDF and referenced
STLs are in `navigation/body_geometry.json`. Reproduce with:

```bash
python3 navigation/extract_geometry.py /path/to/assem1_description \
  --output navigation/body_geometry.json
```

Tests cover URDF rotation/translation order, binary STL parsing and truncation,
footprint containment for every link, tyre versus joint separation, and refusal
to enable hardware without a verified base frame. The source CAD package itself
is not bundled in this commit.

## Deployment state

These geometry updates are committed locally/GitHub but have **not yet been
deployed to the robot**. The running B4 session still uses the earlier settings
and hardware is disabled. Next: obtain lidar sensing-centre offset from the
drive-axle midpoint (forward, left, height), obtain speed calibration, deploy
the updated settings/code, and restart in dry-run mode before physical tests.
