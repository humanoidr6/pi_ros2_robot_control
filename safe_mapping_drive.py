"""Short, guarded LiDAR drive for supervised mapping on the Raspberry Pi."""
import math
import time
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import RPi.GPIO as GPIO


class SafeMappingDrive(Node):
    def __init__(self, duration):
        super().__init__('safe_mapping_drive')
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        self.pins = (17, 27, 22, 10, 9, 11, 5, 6)
        for pin in self.pins:
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        self.left_forward = GPIO.PWM(10, 1000)
        self.left_reverse = GPIO.PWM(22, 1000)
        self.right_forward = GPIO.PWM(6, 1000)
        self.right_reverse = GPIO.PWM(5, 1000)
        self.pwms = (self.left_forward, self.left_reverse,
                     self.right_forward, self.right_reverse)
        for pwm in self.pwms:
            pwm.start(0)
        for pin in (17, 27, 9, 11):
            GPIO.output(pin, GPIO.HIGH)
        self.last_scan = 0.0
        self.front_clearance = None
        self.left_clearance = None
        self.right_clearance = None
        self.started = time.monotonic()
        self.duration = duration
        self.last_report = 0.0
        self.mode = 'stopped'
        self.turn_direction = 0
        self.create_subscription(LaserScan, '/scan', self.on_scan,
                                 qos_profile_sensor_data)
        self.create_timer(0.05, self.control)
        self.get_logger().info(f'Guarded mapping drive armed for {duration:.0f} seconds')

    def on_scan(self, scan):
        sectors = {'front': [], 'left': [], 'right': []}
        for i, distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or not (scan.range_min <= distance <= scan.range_max):
                continue
            angle = scan.angle_min + i * scan.angle_increment
            if abs(angle) <= math.radians(35):
                sectors['front'].append(distance)
            elif math.radians(35) < angle < math.radians(100):
                sectors['left'].append(distance)
            elif -math.radians(100) < angle < -math.radians(35):
                sectors['right'].append(distance)
        self.last_scan = time.monotonic()
        self.front_clearance = min(sectors['front']) if len(sectors['front']) >= 8 else None
        self.left_clearance = min(sectors['left']) if len(sectors['left']) >= 8 else None
        self.right_clearance = min(sectors['right']) if len(sectors['right']) >= 8 else None

    def drive(self, left, right):
        for pwm in self.pwms:
            pwm.ChangeDutyCycle(0)
        self.left_forward.ChangeDutyCycle(max(0, left))
        self.left_reverse.ChangeDutyCycle(max(0, -left))
        self.right_forward.ChangeDutyCycle(max(0, right))
        self.right_reverse.ChangeDutyCycle(max(0, -right))

    def control(self):
        now = time.monotonic()
        if now - self.started >= self.duration:
            self.drive(0, 0)
            self.get_logger().info('Timed run complete; motors stopped')
            raise SystemExit(0)
        if now - self.last_scan > 0.35 or self.front_clearance is None:
            self.drive(0, 0)
            self.mode = 'scan unavailable'
        elif self.front_clearance >= 0.8:
            self.turn_direction = 0
            self.drive(13, 13)
            self.mode = 'forward'
        else:
            left = self.left_clearance or 0
            right = self.right_clearance or 0
            if max(left, right) < 0.7:
                self.drive(0, 0)
                self.mode = 'blocked'
            else:
                if self.turn_direction == 0:
                    self.turn_direction = 1 if left >= right else -1
                if self.turn_direction == 1:
                    self.drive(-13, 13)
                    self.mode = 'turn left'
                else:
                    self.drive(13, -13)
                    self.mode = 'turn right'
        if now - self.last_report >= 2.0:
            self.get_logger().info(
                f'{self.mode}: front={self.front_clearance}, '
                f'left={self.left_clearance}, right={self.right_clearance}, '
                f'scan_age={now - self.last_scan:.2f}s')
            self.last_report = now

    def destroy_node(self):
        self.drive(0, 0)
        for pwm in self.pwms:
            pwm.stop()
        for pin in (17, 27, 9, 11):
            GPIO.output(pin, GPIO.LOW)
        GPIO.cleanup(self.pins)
        super().destroy_node()


def main():
    rclpy.init()
    duration = min(90.0, max(1.0, float(sys.argv[1]))) if len(sys.argv) > 1 else 20.0
    node = SafeMappingDrive(duration)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
