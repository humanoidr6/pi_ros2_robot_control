# Raspberry Pi ROS 2 Differential Drive Robot

This repository contains the software and hardware configuration for a differential drive robot built around a Raspberry Pi, IBT_2 Motor Drivers, Quadrature Encoders, and a YDLidar X2. It runs on ROS 2 (Humble/Iron) using custom Python nodes for low-level hardware control and obstacle avoidance.

## Hardware Components

1. **Compute**: Raspberry Pi 4 (Ubuntu 22.04, ROS 2)
2. **Motor Drivers**: 2x BTS7960 / IBT_2 High-Current Motor Drivers
3. **Motors & Odometry**: 2x DC Motors with Hall-effect Quadrature Encoders
4. **Lidar**: YDLidar X2 (connected via USB serial)

## Software Architecture

The system is organized into a ROS 2 package named `robot_control` containing the following nodes:

- **`motor_controller.py`**: Subscribes to `/cmd_vel` (`geometry_msgs/msg/Twist`) and translates linear and angular velocities into PWM duty cycles for the left and right IBT_2 motor drivers using RPi.GPIO.
- **`encoder_node.py`**: Reads hardware interrupts from the quadrature encoders on the GPIO pins to track wheel rotation and publish odometry/tick data.
- **`obstacle_avoidance.py`**: Subscribes to `/scan` (`sensor_msgs/msg/LaserScan`) from the YDLidar and publishes emergency stop or rotation commands to `/cmd_vel` if an obstacle is detected within a configured safe distance.
- **`ydlidar_ros2_driver_node`**: Official ROS 2 driver for the YDLidar, configured via `X2.yaml`.

## Installation & Setup

1. **System Dependencies**: Run `install_deps.sh` to install necessary ROS 2 packages and Python libraries (`RPi.GPIO`).
2. **Build Workspace**:
   ```bash
   mkdir -p ~/robot_ws/src
   # Copy this repository's contents into ~/robot_ws/src/robot_control
   cd ~/robot_ws
   colcon build
   source install/setup.bash
   ```

## Launch Instructions

The primary launch file brings up the Lidar, Motor Controller, Encoder Node, Obstacle Avoidance node, and necessary static transforms.

```bash
ros2 launch robot_control robot.launch.py
```

To run only the Lidar without activating the motors:

```bash
ros2 launch robot_control lidar_only.launch.py
```

## Hardware Wiring

Refer to `wiring_guide.md` for a complete pinout mapping connecting the Raspberry Pi GPIOs to the IBT_2 drivers and encoder sensors.
