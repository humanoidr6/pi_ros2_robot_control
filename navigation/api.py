"""Loopback HTTP access to Nav2 planning, goals and named destinations."""
import argparse
import json
import math
from pathlib import Path
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


def main():
    import yaml
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.time import Time
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    from sensor_msgs.msg import LaserScan
    from rclpy.qos import qos_profile_sensor_data
    from std_srvs.srv import SetBool
    from nav2_msgs.action import ComputePathToPose, NavigateToPose
    from cartographer_ros_msgs.srv import StartTrajectory, FinishTrajectory, GetTrajectoryStates, TrajectoryQuery
    from tf2_ros import Buffer, TransformListener
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', required=True)
    args, ros_args = parser.parse_known_args()
    map_path = Path(args.map).resolve()
    meta = yaml.safe_load(map_path.read_text())
    image_path = map_path.parent / meta['image']
    waypoint_path = Path.home()/'maps/navigation_waypoints.json'
    rclpy.init(args=ros_args)
    node = Node('navigation_desktop_api')
    tf = Buffer()
    listener = TransformListener(tf, node)
    plan_client = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    goal_client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    arm_client = node.create_client(SetBool, '/navigation/arm')
    confirm_client = node.create_client(SetBool, '/navigation/confirm_localization')
    states_client = node.create_client(GetTrajectoryStates, '/get_trajectory_states')
    query_client = node.create_client(TrajectoryQuery, '/trajectory_query')
    finish_client = node.create_client(FinishTrajectory, '/finish_trajectory')
    start_client = node.create_client(StartTrajectory, '/start_trajectory')
    lock = threading.RLock()
    operations = threading.Lock()
    state = dict(drive={}, drive_at=0., goal='No goal', handle=None, scan=None)

    def drive(msg):
        with lock:
            state['drive'] = json.loads(msg.data)
            state['drive_at'] = time.monotonic()
    node.create_subscription(String, '/navigation/drive_status', drive, 10)
    def scan(msg):
        with lock:
            state['scan'] = msg
    node.create_subscription(LaserScan, '/scan', scan, qos_profile_sensor_data)

    def scan_points():
        msg = state['scan']
        if msg is None:
            return []
        try:
            t = tf.lookup_transform('map', msg.header.frame_id, Time.from_msg(msg.header.stamp))
            p, q = t.transform.translation, t.transform.rotation
            yaw = math.atan2(2*q.w*q.z, 1-2*q.z*q.z)
            return [[p.x+r*math.cos(yaw+msg.angle_min+i*msg.angle_increment),
                     p.y+r*math.sin(yaw+msg.angle_min+i*msg.angle_increment)]
                    for i, r in enumerate(msg.ranges) if i % 2 == 0 and math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        except Exception:
            return []

    def wait(future, timeout=15):
        end = time.monotonic()+timeout
        while not future.done() and time.monotonic() < end:
            time.sleep(0.02)
        if not future.done():
            future.cancel()
            raise RuntimeError('ROS operation timed out')
        return future.result()

    def service(client, req):
        if not client.wait_for_service(timeout_sec=2):
            raise RuntimeError('ROS service unavailable: '+client.srv_name)
        return wait(client.call_async(req))

    def set_bool(client, value):
        req = SetBool.Request()
        req.data = value
        response = service(client, req)
        if not response.success:
            raise RuntimeError(response.message)
        return response.message

    def pose(data):
        x, y, yaw = float(data['x']), float(data['y']), float(data.get('yaw', 0))
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise ValueError('Invalid pose')
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = x, y
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        return msg

    def robot_pose():
        try:
            t = tf.lookup_transform('map', 'base_link', Time())
            p, q = t.transform.translation, t.transform.rotation
            return dict(x=p.x, y=p.y, yaw=math.atan2(2*q.w*q.z, 1-2*q.z*q.z),
                        age=(node.get_clock().now().nanoseconds-Time.from_msg(t.header.stamp).nanoseconds)/1e9)
        except Exception:
            return None

    def cancel():
        set_bool(arm_client, False)
        with lock:
            handle = state['handle']
            state['handle'] = None
            state['goal'] = 'Cancelled / disarmed'
        if handle:
            wait(handle.cancel_goal_async(), 5)

    def localize(data):
        cancel()
        set_bool(confirm_client, False)
        desired = pose(data)
        response = service(states_client, GetTrajectoryStates.Request())
        frozen = [i for i, s in zip(response.trajectory_states.trajectory_id, response.trajectory_states.trajectory_state) if s == 2]
        if not frozen:
            raise RuntimeError('No frozen saved trajectory is loaded')
        reference = frozen[0]
        req = TrajectoryQuery.Request()
        req.trajectory_id = reference
        old = service(query_client, req)
        if not old.trajectory:
            raise RuntimeError('Saved trajectory has no poses')
        # Cartographer uses the reference trajectory pose at time zero (first pose).
        first = old.trajectory[0].pose
        q = first.orientation
        angle = math.atan2(2*q.w*q.z, 1-2*q.z*q.z)
        dx, dy = desired.pose.position.x-first.position.x, desired.pose.position.y-first.position.y
        desired.pose.position.x = math.cos(angle)*dx+math.sin(angle)*dy
        desired.pose.position.y = -math.sin(angle)*dx+math.cos(angle)*dy
        yaw = float(data.get('yaw', 0))-angle
        desired.pose.orientation.z, desired.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        for i, status in zip(response.trajectory_states.trajectory_id, response.trajectory_states.trajectory_state):
            if status == 0:
                req = FinishTrajectory.Request()
                req.trajectory_id = i
                service(finish_client, req)
        req = StartTrajectory.Request()
        req.configuration_directory = str(Path(__file__).parent)
        req.configuration_basename = 'localization.lua'
        req.use_initial_pose = True
        req.initial_pose = desired.pose
        req.relative_to_trajectory_id = reference
        result = service(start_client, req)
        if result.status.code:
            raise RuntimeError(result.status.message)
        return dict(trajectory=result.trajectory_id, message='Pose initialized. Verify scan/map alignment before confirming localization.')

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def reply(self, value, code=200, mime='application/json'):
            data = json.dumps(value).encode() if mime == 'application/json' else value
            self.send_response(code)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            try:
                if self.path == '/status':
                    with lock:
                        drive = dict(state['drive'])
                        if time.monotonic()-state['drive_at'] > 1:
                            drive.update(armed=False, reason='Motor gate unavailable')
                        self.reply(dict(pose=robot_pose(), scan=scan_points(), drive=drive, goal=state['goal'],
                            planner_ready=plan_client.server_is_ready(), navigator_ready=goal_client.server_is_ready()))
                elif self.path == '/map':
                    self.reply(meta)
                elif self.path == '/map.pgm':
                    self.reply(image_path.read_bytes(), mime='image/x-portable-graymap')
                elif self.path == '/waypoints':
                    self.reply(json.loads(waypoint_path.read_text()) if waypoint_path.exists() else {})
                else:
                    self.reply({'error': 'Unknown endpoint'}, 404)
            except Exception as exc:
                self.reply({'error': str(exc)}, 500)

        def do_POST(self):
            try:
                size = int(self.headers.get('Content-Length', 0))
                if not 0 <= size <= 16384:
                    raise ValueError('Invalid request size')
                data = json.loads(self.rfile.read(size) or b'{}')
                if self.path == '/stop':
                    cancel()
                    self.reply({'ok': True})
                    return
                with operations:
                    result = self.operation(data)
                self.reply(result)
            except (ValueError, KeyError) as exc:
                self.reply({'error': str(exc)}, 400)
            except Exception as exc:
                self.reply({'error': str(exc)}, 409)

        def operation(self, data):
            if self.path == '/plan':
                goal = ComputePathToPose.Goal()
                goal.goal = pose(data)
                goal.planner_id = 'GridBased'
                if data.get('start'):
                    goal.use_start = True
                    goal.start = pose(data['start'])
                if not plan_client.wait_for_server(timeout_sec=3):
                    raise RuntimeError('Planner not ready')
                handle = wait(plan_client.send_goal_async(goal))
                if not handle.accepted:
                    raise RuntimeError('Planner rejected request')
                result = wait(handle.get_result_async(), 20)
                if result.status != 4 or not result.result.path.poses:
                    raise RuntimeError('No traversable path found')
                return {'path': [[p.pose.position.x, p.pose.position.y] for p in result.result.path.poses]}
            if self.path == '/goal':
                with lock:
                    if time.monotonic()-state['drive_at'] > 1 or not state['drive'].get('armed'):
                        raise RuntimeError('Drive is not armed; use Plan path for non-moving tests')
                    if state['handle'] is not None:
                        raise RuntimeError('Cancel the current goal before sending another')
                goal = NavigateToPose.Goal()
                goal.pose = pose(data)
                if not goal_client.wait_for_server(timeout_sec=3):
                    raise RuntimeError('Navigator not ready')
                handle = wait(goal_client.send_goal_async(goal))
                if not handle.accepted:
                    raise RuntimeError('Navigator rejected goal')
                with lock:
                    state['handle'] = handle
                    state['goal'] = 'Navigating'
                def completed(f):
                    with lock:
                        state['goal'] = 'Reached goal' if f.result().status == 4 else 'Goal ended: '+str(f.result().status)
                        state['handle'] = None
                    # Async disarm avoids blocking the ROS executor callback.
                    req = SetBool.Request(); req.data = False
                    arm_client.call_async(req)
                handle.get_result_async().add_done_callback(completed)
                return {'message': 'Goal accepted'}
            if self.path == '/arm':
                return {'message': set_bool(arm_client, True)}
            if self.path == '/confirm-localization':
                return {'message': set_bool(confirm_client, True)}
            if self.path == '/initialpose':
                return localize(data)
            if self.path == '/waypoint':
                name = str(data['name']).strip()
                if not name or len(name) > 80:
                    raise ValueError('Use a destination name of 1–80 characters')
                p = pose(data).pose
                points = json.loads(waypoint_path.read_text()) if waypoint_path.exists() else {}
                points[name] = dict(x=p.position.x, y=p.position.y, yaw=float(data.get('yaw', 0)))
                waypoint_path.parent.mkdir(parents=True, exist_ok=True)
                temp = waypoint_path.with_suffix('.tmp')
                temp.write_text(json.dumps(points, indent=2)+'\n')
                temp.replace(waypoint_path)
                return {'message': 'Saved destination '+name}
            raise ValueError('Unknown operation')

    server = ThreadingHTTPServer(('127.0.0.1', 18766), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
