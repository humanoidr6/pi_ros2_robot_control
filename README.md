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
