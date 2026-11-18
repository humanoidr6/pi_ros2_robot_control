"""Expose Cartographer's local odom->base_link TF as Odometry for Nav2.

This is lidar-derived feedback, not calibrated wheel odometry. No TF is published.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener


class TFOdometry(Node):
    def __init__(self):
        super().__init__('lidar_tf_odometry')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.previous = None
        self.create_timer(0.05, self.tick)

    def tick(self):
        try:
            tf = self.buffer.lookup_transform('odom', 'base_link', Time())
        except Exception:
            return
        stamp = Time.from_msg(tf.header.stamp).nanoseconds/1e9
        if self.get_clock().now().nanoseconds/1e9-stamp > 0.5:
            return
        p, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        msg = Odometry()
        msg.header = tf.header
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y = p.x, p.y
        msg.pose.pose.orientation = q
        if self.previous:
            last, x, y, angle = self.previous
            dt = stamp-last
            if dt <= 0:
                return
            if dt < 0.5:
                dx, dy = (p.x-x)/dt, (p.y-y)/dt
                msg.twist.twist.linear.x = math.cos(yaw)*dx+math.sin(yaw)*dy
                msg.twist.twist.linear.y = -math.sin(yaw)*dx+math.cos(yaw)*dy
                msg.twist.twist.angular.z = math.atan2(math.sin(yaw-angle), math.cos(yaw-angle))/dt
        self.previous = stamp, p.x, p.y, yaw
        for i in (0, 7, 35):
            msg.pose.covariance[i] = 0.1
            msg.twist.covariance[i] = 0.2
        for i in (14, 21, 28):
            msg.pose.covariance[i] = msg.twist.covariance[i] = 1e6
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = TFOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
