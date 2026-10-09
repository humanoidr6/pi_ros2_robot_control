import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None

class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoder_node')
        
        # Left Motor Encoder Pins
        self.ENC_L_A = 23
        self.ENC_L_B = 24
        
        # Right Motor Encoder Pins
        self.ENC_R_A = 25
        self.ENC_R_B = 8
        
        self.ticks_l = 0
        self.ticks_r = 0
        
        if GPIO:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.ENC_L_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.ENC_L_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.ENC_R_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.ENC_R_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
            GPIO.add_event_detect(self.ENC_L_A, GPIO.RISING, callback=self.left_callback)
            GPIO.add_event_detect(self.ENC_R_A, GPIO.RISING, callback=self.right_callback)
            self.get_logger().info('Encoder GPIO initialized.')
        else:
            self.get_logger().warn('Mock mode: RPi.GPIO not available.')

        self.pub_l = self.create_publisher(Float32, '/encoder/left_ticks', 10)
        self.pub_r = self.create_publisher(Float32, '/encoder/right_ticks', 10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        
    def left_callback(self, channel):
        # Read phase B to determine direction
        if GPIO.input(self.ENC_L_B):
            self.ticks_l += 1
        else:
            self.ticks_l -= 1
            
    def right_callback(self, channel):
        if GPIO.input(self.ENC_R_B):
            self.ticks_r += 1
        else:
            self.ticks_r -= 1

    def timer_callback(self):
        msg_l = Float32()
        msg_l.data = float(self.ticks_l)
        self.pub_l.publish(msg_l)
        
        msg_r = Float32()
        msg_r.data = float(self.ticks_r)
        self.pub_r.publish(msg_r)

    def destroy_node(self):
        if GPIO:
            GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
