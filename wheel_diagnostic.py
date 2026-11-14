"""Brief wheel and encoder diagnostic. Stops and disables drivers on exit."""
import time
import math
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import RPi.GPIO as GPIO
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time


class EncoderReadout(Node):
    def __init__(self):
        super().__init__('wheel_diagnostic')
        self.left = None
        self.right = None
        self.create_subscription(Int32, '/encoder/left_ticks',
                                 lambda msg: setattr(self, 'left', msg.data), 10)
        self.create_subscription(Int32, '/encoder/right_ticks',
                                 lambda msg: setattr(self, 'right', msg.data), 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def yaw(self):
        try:
            q = self.tf_buffer.lookup_transform('map', 'base_link', Time()).transform.rotation
            return math.degrees(2 * math.atan2(q.z, q.w))
        except Exception:
            return None


def spin_for(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def main():
    rclpy.init()
    node = EncoderReadout()
    pins = (17, 27, 22, 10, 9, 11, 5, 6)
    pwms = []
    try:
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        for pin in pins:
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        left = GPIO.PWM(10, 1000)
        right = GPIO.PWM(6, 1000)
        pwms = [left, right]
        for pwm in pwms:
            pwm.start(0)
        for pin in (17, 27, 9, 11):
            GPIO.output(pin, GPIO.HIGH)
        deadline = time.monotonic() + 10.0
        while (node.left is None or node.right is None) and time.monotonic() < deadline:
            spin_for(node, 0.2)
        if node.left is None or node.right is None:
            raise RuntimeError('Encoder topics did not publish; wheels were not tested')
        stages = (
            ('right only', 0, 20),
            ('left only', 20, 0),
            ('both', 20, 20),
        )
        if len(sys.argv) > 1:
            stages = tuple(stage for stage in stages if stage[0].split()[0] == sys.argv[1])
            if not stages:
                raise ValueError('Expected right, left, or both')
        duty = min(50.0, max(0.0, float(sys.argv[2]))) if len(sys.argv) > 2 else 20.0
        pulse_seconds = min(1.5, max(0.1, float(sys.argv[3]))) if len(sys.argv) > 3 else 1.5
        for label, left_duty, right_duty in stages:
            before = (node.left, node.right)
            yaw_before = node.yaw()
            left.ChangeDutyCycle(duty if left_duty else 0)
            right.ChangeDutyCycle(duty if right_duty else 0)
            spin_for(node, pulse_seconds)
            left.ChangeDutyCycle(0)
            right.ChangeDutyCycle(0)
            spin_for(node, 0.3)
            after = (node.left, node.right)
            yaw_after = node.yaw()
            print(f'{label}: before={before}, after={after}, '
                  f'delta=({after[0] - before[0] if None not in (after[0], before[0]) else None}, '
                  f'{after[1] - before[1] if None not in (after[1], before[1]) else None}), '
                  f'yaw={yaw_before}->{yaw_after}',
                  flush=True)
    finally:
        for pwm in pwms:
            pwm.ChangeDutyCycle(0)
            pwm.stop()
        for pin in (17, 27, 9, 11):
            GPIO.output(pin, GPIO.LOW)
        GPIO.cleanup(pins)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
