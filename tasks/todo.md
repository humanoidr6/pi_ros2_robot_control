# Autonomous Robot Project Plan

## Phase 1: Architecture & Setup (✅ DONE)
- [x] Decide on Software Stack: ROS 2 (recommended for LiDAR) vs. Plain Python scripts.
- [x] Define GPIO pin mappings for Raspberry Pi 4 (IBT_2 PWM pins, Encoder A/B pins).
- [x] Set up the development environment (directly on the Raspberry Pi via SSH).
- [x] Create comprehensive hardware wiring guide.

## Phase 2: Motor Control & Odometry (🚧 IN PROGRESS)
- [x] Implement basic motor driver script (IBT_2 forward/backward/stop via PWM).
- [x] Implement encoder interrupt handlers to count motor ticks.
- [ ] Test encoder interrupts on real hardware (Need to test with `sudo` permissions tomorrow).
- [x] Implement basic kinematics in controller.
- [ ] Tune PID control loop to maintain target speeds (Next Step).

## Phase 3: Sensor Integration (YDLIDAR X2) (✅ DONE)
- [x] Install YDLidar SDK / ROS drivers on the Raspberry Pi.
- [x] Verify LiDAR is communicating with the Pi and fetching scan data (Port mapped, 262 points verified).
- [x] Write a script/node to parse distance data and detect obstacles in the path.

## Phase 4: Autonomous Navigation (🚧 IN PROGRESS)
- [x] Implement basic obstacle avoidance logic (stop and rotate when obstacle < 0.5m ahead).
- [x] Create unified ROS 2 launch file (`robot.launch.py`).
- [ ] Real-world test of the obstacle avoidance loop (Next Step).
- [ ] (Optional if using ROS) Set up SLAM and Nav2.
