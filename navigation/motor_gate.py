"""Independent watchdog and explicit arming for Nav2. Dry-run by default."""
import argparse
import fcntl
import json
import math
import signal
import threading
import time
from pathlib import Path
from safety import missing_calibration, velocity_to_pwm, gate_reason


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    from tf2_ros import Buffer, TransformListener
    parser = argparse.ArgumentParser()
    parser.add_argument('--hardware', action='store_true')
    parser.add_argument('--settings', default=str(Path(__file__).with_name('robot_settings.json')))
    args, ros_args = parser.parse_known_args()
    settings = json.loads(Path(args.settings).read_text())
    missing = missing_calibration(settings)
    if args.hardware and missing:
        raise RuntimeError('Hardware disabled until calibrated: '+', '.join(missing))
    rclpy.init(args=ros_args)
    node = Node('navigation_motor_gate')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    lock = threading.RLock()
    state = dict(armed=False, localization_confirmed=False, command=(0., 0.), command_at=0.,
                 scan_at=0., transform_age=1e9, nearest=None, reason='Disarmed', hardware=args.hardware)
    pins = (17, 27, 22, 10, 9, 11, 5, 6)
    enables = (17, 27, 9, 11)
    pwms = []
    if args.hardware:
        import RPi.GPIO as GPIO
        gpio_lock = open('/tmp/manual_mapping_gpio.lock', 'w')
        fcntl.flock(gpio_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for pin in pins:
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        for pin in (10, 22, 6, 5):
            pwm = GPIO.PWM(pin, 1000)
            pwm.start(0)
            pwms.append(pwm)

    def output(left=0., right=0.):
        if not args.hardware:
            return
        for pwm in pwms:
            pwm.ChangeDutyCycle(0)
        for pin in enables:
            GPIO.output(pin, GPIO.HIGH if left or right else GPIO.LOW)
        for speed, fwd, rev in ((left, pwms[0], pwms[1]), (right, pwms[2], pwms[3])):
            (fwd if speed >= 0 else rev).ChangeDutyCycle(abs(speed))

    def command(msg):
        with lock:
            state['command'] = (msg.linear.x, msg.angular.z)
            state['command_at'] = time.monotonic()

    def scan(msg):
        ranges = [v for v in msg.ranges if math.isfinite(v) and msg.range_min <= v <= msg.range_max]
        with lock:
            state['scan_at'] = time.monotonic()
            state['nearest'] = min(ranges) if len(ranges) >= 30 else None

    def confirm(req, res):
        with lock:
            state['localization_confirmed'] = req.data
            state['armed'] = False
            output()
        res.success = True
        res.message = 'Localization confirmation set; motors remain disarmed'
        return res

    def arm(req, res):
        with lock:
            output()
            state['armed'] = False
            state['command_at'] = 0
            if req.data:
                reason = gate_reason(True, 0, time.monotonic()-state['scan_at'], state['transform_age'], state['nearest'])
                if not args.hardware or missing or not state['localization_confirmed'] or reason:
                    res.success = False
                    res.message = 'Cannot arm: '+('dry-run mode' if not args.hardware else ', '.join(missing) or reason or 'confirm localization first')
                    return res
                state['armed'] = True
            res.success = True
            res.message = 'Armed' if state['armed'] else 'Disarmed'
            return res

    publisher = node.create_publisher(String, '/navigation/drive_status', 10)
    node.create_subscription(Twist, '/cmd_vel', command, 10)
    node.create_subscription(LaserScan, '/scan', scan, qos_profile_sensor_data)
    node.create_service(SetBool, '/navigation/arm', arm)
    node.create_service(SetBool, '/navigation/confirm_localization', confirm)

    def report():
        try:
            t = buffer.lookup_transform('map', 'base_link', Time())
            age = max(0, (node.get_clock().now().nanoseconds - Time.from_msg(t.header.stamp).nanoseconds)/1e9)
        except Exception:
            age = 1e9
        with lock:
            state['transform_age'] = age
            publisher.publish(String(data=json.dumps(dict(armed=state['armed'], hardware=args.hardware,
                localization_confirmed=state['localization_confirmed'], reason=state['reason'],
                missing_calibration=missing, nearest=state['nearest'], transform_age=age))))
    node.create_timer(0.1, report)
    running = threading.Event()
    running.set()

    def watchdog():
        try:
            while running.is_set():
                with lock:
                    now = time.monotonic()
                    reason = gate_reason(state['armed'], now-state['command_at'], now-state['scan_at'], state['transform_age'], state['nearest'])
                    state['reason'] = reason
                    if reason:
                        output()
                        if state['armed'] and reason != 'Command timeout':
                            state['armed'] = False
                    else:
                        output(*velocity_to_pwm(*state['command'], settings))
                time.sleep(0.025)
        finally:
            output()
            for pwm in pwms:
                pwm.stop()
            if args.hardware:
                GPIO.cleanup(pins)
    thread = threading.Thread(target=watchdog)
    thread.start()
    def stop(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGHUP, stop)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        running.clear()
        thread.join()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
