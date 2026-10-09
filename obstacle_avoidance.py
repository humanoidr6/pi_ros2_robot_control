import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Configuration
        self.safe_distance = 0.5  # meters
        self.forward_speed = 0.3  # m/s
        self.turn_speed = 0.5     # rad/s

        self.get_logger().info('Obstacle Avoidance Node started.')

    def scan_callback(self, msg):
        # The YDLidar scans 360 degrees. Let's look at the front cone (-30 to +30 degrees)
        # Check angle_min, angle_max, angle_increment to find indices.
        # For simplicity, assuming index 0 is front, or finding the minimum distance overall.
        
        # Filter out 0.0 ranges (errors or out of bounds)
        valid_ranges = [r for r in msg.ranges if r > 0.05 and r < float('inf')]
        
        if not valid_ranges:
            return
            
        min_distance = min(valid_ranges)
        
        cmd = Twist()
        if min_distance < self.safe_distance:
            self.get_logger().warn(f'Obstacle detected! Distance: {min_distance:.2f}m. Turning...')
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed
        else:
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0
            
        self.publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
