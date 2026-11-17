"""Loopback-only ROS/GPIO bridge. Access through an authenticated SSH tunnel."""
import json
import math
import os
import signal
import subprocess
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


def motion_allowed(left, right, scan_age, nearest):
    if not left and not right:
        return True, 'Stopped'
    if scan_age > 0.4:
        return False, 'Stopped: lidar data stale'
    # Conservative all-around clearance also protects against uncertain polarity.
    if nearest is None or nearest < 0.50:
        return False, 'Stopped: obstacle within 0.50 m or insufficient lidar returns'
    return True, 'Driving — release to stop'


def main():
    import fcntl
    import RPi.GPIO as GPIO
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import OccupancyGrid

    lockfile = open('/tmp/manual_mapping_gpio.lock', 'w')
    fcntl.flock(lockfile, fcntl.LOCK_EX | fcntl.LOCK_NB)
    rclpy.init()

    class Bridge(Node):
        def __init__(self):
            super().__init__('manual_mapping_remote')
            self.lock = threading.RLock()
            self.target = (0, 0)
            self.last_command = 0
            self.last_scan = 0
            self.nearest = None
            self.estop = False
            self.saving = False
            self.status = 'Stopped'
            self.map = None
            self.map_seq = 0
            self.pins = (17, 27, 22, 10, 9, 11, 5, 6)
            self.enables = (17, 27, 9, 11)
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            for pin in self.pins:
                GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
            self.pwms = [GPIO.PWM(pin, 1000) for pin in (10, 22, 6, 5)]
            for pwm in self.pwms:
                pwm.start(0)
            self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
            self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        def scan(self, msg):
            values = [v for v in msg.ranges if math.isfinite(v) and msg.range_min <= v <= msg.range_max]
            with self.lock:
                self.last_scan = time.monotonic()
                self.nearest = min(values) if len(values) >= 30 else None

        def map_callback(self, msg):
            with self.lock:
                self.map = msg
                self.map_seq += 1

        def output(self, left, right):
            # Break before make when changing direction.
            for pwm in self.pwms:
                pwm.ChangeDutyCycle(0)
            for pin in self.enables:
                GPIO.output(pin, GPIO.HIGH if left or right else GPIO.LOW)
            for value, forward, reverse in ((left, self.pwms[0], self.pwms[1]), (right, self.pwms[2], self.pwms[3])):
                (forward if value >= 0 else reverse).ChangeDutyCycle(abs(value))

        def tick(self):
            with self.lock:
                now = time.monotonic()
                left, right = self.target
                if self.estop or self.saving or now - self.last_command > 0.35:
                    left = right = 0
                    self.target = (0, 0)
                    self.status = 'EMERGENCY STOP — reset to enable controls' if self.estop else 'Saving map — stopped' if self.saving else 'Stopped'
                else:
                    allowed, self.status = motion_allowed(left, right, now-self.last_scan, self.nearest)
                    if not allowed:
                        left = right = 0
                        self.target = (0, 0)
                self.output(left, right)

        def close(self):
            with self.lock:
                self.output(0, 0)
                for pwm in self.pwms:
                    pwm.stop()
                GPIO.cleanup(self.pins)

    bridge = Bridge()
    running = threading.Event()
    running.set()
    # Independent watchdog does not depend on ROS callbacks or HTTP requests.
    def watchdog():
        try:
            while running.is_set():
                bridge.tick()
                time.sleep(0.025)
        finally:
            bridge.close()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, payload, status=200, kind='application/json'):
            data = json.dumps(payload).encode() if kind == 'application/json' else payload
            self.send_response(status)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            with bridge.lock:
                if self.path == '/status':
                    self.reply(dict(status=bridge.status, estop=bridge.estop,
                                    scan_age=round(time.monotonic()-bridge.last_scan, 2), nearest=bridge.nearest,
                                    map_seq=bridge.map_seq, mapping=bridge.map is not None))
                elif self.path == '/map.pgm' and bridge.map is not None:
                    msg = bridge.map
                    w, h = msg.info.width, msg.info.height
                    pixels = bytearray(205 if v < 0 else 254 if v < 25 else 0 if v > 65 else 128 for v in msg.data)
                    flipped = b''.join(pixels[y*w:(y+1)*w] for y in range(h-1, -1, -1))
                    self.reply(f'P5\n{w} {h}\n255\n'.encode()+flipped, kind='image/x-portable-graymap')
                else:
                    self.reply({'error': 'Not found'}, 404)

        def do_POST(self):
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if size > 1024:
                    raise ValueError('Request too large')
                data = json.loads(self.rfile.read(size) or b'{}')
                if self.path == '/save':
                    self.save()
                    return
                with bridge.lock:
                    if self.path == '/drive':
                        left, right = float(data['left']), float(data['right'])
                        if not all(math.isfinite(v) and abs(v) <= 100 for v in (left, right)):
                            raise ValueError('PWM must be between -100 and 100')
                        if bridge.estop or bridge.saving:
                            self.reply({'error': 'Controls disabled'}, 409)
                            return
                        bridge.target = (left, right)
                        bridge.last_command = time.monotonic()
                    elif self.path in ('/stop', '/estop', '/reset'):
                        bridge.target = (0, 0)
                        bridge.last_command = 0
                        bridge.output(0, 0)
                        if self.path == '/estop':
                            bridge.estop = True
                        elif self.path == '/reset':
                            bridge.estop = False
                    else:
                        self.reply({'error': 'Not found'}, 404)
                        return
                self.reply({'ok': True})
            except (ValueError, KeyError, TypeError) as exc:
                self.reply({'error': str(exc)}, 400)

        def save(self):
            with bridge.lock:
                if bridge.saving:
                    self.reply({'error': 'Save already in progress'}, 409)
                    return
                bridge.saving = True
                bridge.target = (0, 0)
                bridge.output(0, 0)
            prefix = os.path.expanduser('~/maps/manual_floor_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
            os.makedirs(os.path.dirname(prefix), exist_ok=True)
            try:
                grid = subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', prefix,
                                       '--ros-args', '-p', 'save_map_timeout:=10.0'], capture_output=True, text=True, timeout=20)
                if grid.returncode or not os.path.exists(prefix+'.yaml'):
                    raise RuntimeError((grid.stdout+grid.stderr)[-1500:])
                state = subprocess.run(['ros2', 'service', 'call', '/write_state', 'cartographer_ros_msgs/srv/WriteState',
                                        json.dumps({'filename': prefix+'.pbstream', 'include_unfinished_submaps': True})],
                                       capture_output=True, text=True, timeout=20)
                self.reply({'saved': prefix, 'state_saved': state.returncode == 0 and os.path.exists(prefix+'.pbstream')})
            except Exception as exc:
                self.reply({'error': str(exc)}, 500)
            finally:
                with bridge.lock:
                    bridge.saving = False
                    bridge.target = (0, 0)
                    bridge.last_command = 0

    server = ThreadingHTTPServer(('127.0.0.1', 18765), Handler)
    worker = threading.Thread(target=watchdog)
    worker.start()
    threading.Thread(target=server.serve_forever, daemon=True).start()
    def terminate(*args):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    signal.signal(signal.SIGHUP, terminate)
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        running.clear()
        worker.join()
        server.shutdown()
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
