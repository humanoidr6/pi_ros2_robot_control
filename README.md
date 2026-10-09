# Differential Drive ROS 2 Robot

This repository contains the control stack for a custom differential drive robot. The hardware relies on a Raspberry Pi 4 operating as the primary compute unit, interfacing with BTS7960/IBT-2 motor drivers, Hall-effect quadrature encoders, and a YDLidar X2.

## System Architecture

The software is structured as a standard ROS 2 package (`robot_control`) that abstracts the hardware into individual nodes. This modular approach separates hardware interfacing from higher-level logic.

### 1. Motor Control (`motor_controller.py`)
The motor controller node translates velocity commands into hardware signals. It subscribes to the `/cmd_vel` topic (type: `geometry_msgs/Twist`). The differential drive kinematics equation determines the target velocity for the left and right tracks. These velocities scale to a PWM duty cycle (0-100%). The node utilizes `RPi.GPIO` to generate 1kHz PWM signals on the specific pins wired to the IBT-2 drivers, dictating both speed and direction.

### 2. Odometry & Encoders (`encoder_node.py`)
This node tracks wheel rotation to provide feedback for position estimation. It registers hardware interrupts on the Raspberry Pi GPIO pins connected to the Hall-effect sensors. The node increments or decrements a tick counter based on the quadrature phase (Phase A vs Phase B). The tick count is periodically published as odometry data, forming the basis for closed-loop control or mapping.

### 3. Obstacle Avoidance (`obstacle_avoidance.py`)
A reactive safety layer. The node subscribes to the `/scan` topic provided by the YDLidar driver with a `BEST_EFFORT` QoS profile. It evaluates the distance measurements in the front cone. If an object falls within the defined safety threshold, it preempts standard navigation by publishing a zero linear velocity and a fixed angular velocity to `/cmd_vel`, forcing the robot to pivot until the path is clear.

### 4. Lidar Integration
The `ydlidar_ros2_driver_node` handles serial communication with the YDLidar X2. The driver converts the proprietary serial protocol into standard `sensor_msgs/LaserScan` messages. A static transform publisher links the `base_link` frame to the `laser_frame` to maintain a correct tf tree.

## Installation

1. Execute `install_deps.sh` to install ROS 2 dependencies and the `RPi.GPIO` library.
2. Clone the repository into a ROS 2 workspace (e.g., `~/robot_ws/src/robot_control`).
3. Build the workspace:
   ```bash
   cd ~/robot_ws
   colcon build --packages-select robot_control
   source install/setup.bash
   ```

## Execution

Bring up the entire stack, including motor control, encoders, lidar, and obstacle avoidance:
```bash
ros2 launch robot_control robot.launch.py
```

To isolate the lidar subsystem for testing:
```bash
ros2 launch robot_control lidar_only.launch.py
```

See `wiring_guide.md` for the exact pinout and electrical constraints.

## Hardware Wiring Specification

The tables below detail the GPIO pin mappings and power distribution requirements linking the Raspberry Pi 4, the IBT-2 motor drivers, and the Hall-effect encoders. All logic connections utilize standard BCM numbering. Physical board pin numbers are provided for reference.

### 1. Left Motor Driver (IBT-2)

| IBT-2 Pin | Pi BCM Pin | Pi Physical Pin | Function |
| :--- | :--- | :--- | :--- |
| VCC | 3.3V | Pin 1 | Logic Power |
| GND | GND | Pin 9 | Logic Ground |
| R_EN | GPIO 27 | Pin 13 | Right Enable (Active High) |
| L_EN | GPIO 17 | Pin 11 | Left Enable (Active High) |
| RPWM | GPIO 10 | Pin 19 | Forward PWM Signal |
| LPWM | GPIO 22 | Pin 15 | Reverse PWM Signal |
| R_S | N/C | N/A | Current Sense (Unused) |
| L_S | N/C | N/A | Current Sense (Unused) |

### 2. Right Motor Driver (IBT-2)

| IBT-2 Pin | Pi BCM Pin | Pi Physical Pin | Function |
| :--- | :--- | :--- | :--- |
| VCC | 3.3V | Pin 17 | Logic Power |
| GND | GND | Pin 25 | Logic Ground |
| R_EN | GPIO 11 | Pin 23 | Right Enable (Active High) |
| L_EN | GPIO 9 | Pin 21 | Left Enable (Active High) |
| RPWM | GPIO 6 | Pin 31 | Forward PWM Signal |
| LPWM | GPIO 5 | Pin 29 | Reverse PWM Signal |
| R_S | N/C | N/A | Current Sense (Unused) |
| L_S | N/C | N/A | Current Sense (Unused) |

### 3. Quadrature Encoders (Hall Effect)

| Encoder Pin | Pi BCM Pin | Pi Physical Pin | Function |
| :--- | :--- | :--- | :--- |
| VCC | 3.3V / 5V | Variable | Sensor Power |
| GND | GND | Variable | Sensor Ground |
| Left Phase A | GPIO 23 | Pin 16 | Left Interrupt Trigger |
| Left Phase B | GPIO 24 | Pin 18 | Left Direction Check |
| Right Phase A| GPIO 25 | Pin 22 | Right Interrupt Trigger |
| Right Phase B| GPIO 8 | Pin 24 | Right Direction Check |

### 4. Power Distribution & Safety Constraints

- **Common Ground Requirement:** The logic ground (GND) on both IBT-2 drivers must be tied directly to a ground pin on the Raspberry Pi. The main battery ground must share this reference. Failure to establish a common ground results in floating PWM signals and erratic motor behavior.
- **High Current Isolation:** Route main battery power (+12V/24V) directly to the B+ and B- terminals on the IBT-2 blocks. Route the M+ and M- terminals directly to the motors. Do not route high-current battery power through the Raspberry Pi or its breadboard rails.
- **Logic Level Constraints:** The Raspberry Pi GPIO pins operate at 3.3V. While the IBT-2 logic inputs (VCC) are 5V tolerant, supplying them with 3.3V from the Pi ensures the PWM and Enable signals are interpreted correctly without requiring dedicated logic level shifting circuitry.

## Author / Creator

**B Jithendra**  
Contact: [b.jithendra31@gmail.com](mailto:b.jithendra31@gmail.com)
