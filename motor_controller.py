import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None
    print("RPi.GPIO not found. Running in mock mode.")

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # Configuration
        self.declare_parameter('wheel_base', 0.2) # meters
        self.wheel_base = self.get_parameter('wheel_base').value
        
        # Define GPIO pins (BCM numbering)
        # Motor A (Left)
        self.L_EN_A = 17
        self.R_EN_A = 27
        self.L_PWM_A = 22 # Reverse
        self.R_PWM_A = 10 # Forward
        
        # Motor B (Right)
        self.L_EN_B = 9
        self.R_EN_B = 11
        self.L_PWM_B = 5  # Reverse
        self.R_PWM_B = 6  # Forward
        
        # Setup GPIO
        if GPIO:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            pins = [self.L_EN_A, self.R_EN_A, self.L_PWM_A, self.R_PWM_A,
                    self.L_EN_B, self.R_EN_B, self.L_PWM_B, self.R_PWM_B]
            for pin in pins:
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, GPIO.LOW)
                
            # Enable the drivers
            GPIO.output(self.L_EN_A, GPIO.HIGH)
            GPIO.output(self.R_EN_A, GPIO.HIGH)
            GPIO.output(self.L_EN_B, GPIO.HIGH)
            GPIO.output(self.R_EN_B, GPIO.HIGH)
            
            # Setup PWM at 1kHz
            self.pwm_LA = GPIO.PWM(self.L_PWM_A, 1000)
            self.pwm_RA = GPIO.PWM(self.R_PWM_A, 1000)
            self.pwm_LB = GPIO.PWM(self.L_PWM_B, 1000)
            self.pwm_RB = GPIO.PWM(self.R_PWM_B, 1000)
            
            self.pwm_LA.start(0)
            self.pwm_RA.start(0)
            self.pwm_LB.start(0)
            self.pwm_RB.start(0)
            
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
            
        self.get_logger().info('Motor controller initialized.')

    def set_motor_left(self, speed):
        """ Speed from -1.0 to 1.0 """
        duty = abs(speed) * 100.0
        duty = max(0.0, min(100.0, duty))
        if GPIO:
            if speed > 0:
                self.pwm_RA.ChangeDutyCycle(duty)
                self.pwm_LA.ChangeDutyCycle(0)
            else:
                self.pwm_RA.ChangeDutyCycle(0)
                self.pwm_LA.ChangeDutyCycle(duty)
        else:
            self.get_logger().debug(f'Mock Left Motor: {speed:.2f} (Duty: {duty:.1f}%)')

    def set_motor_right(self, speed):
        """ Speed from -1.0 to 1.0 """
        duty = abs(speed) * 100.0
        duty = max(0.0, min(100.0, duty))
        if GPIO:
            if speed > 0:
                self.pwm_RB.ChangeDutyCycle(duty)
                self.pwm_LB.ChangeDutyCycle(0)
            else:
                self.pwm_RB.ChangeDutyCycle(0)
                self.pwm_LB.ChangeDutyCycle(duty)
        else:
            self.get_logger().debug(f'Mock Right Motor: {speed:.2f} (Duty: {duty:.1f}%)')

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z
        
        # Differential drive kinematics
        # v = (v_right + v_left) / 2
        # w = (v_right - v_left) / wheel_base
        # Therefore:
        v_left = v - (w * self.wheel_base / 2.0)
        v_right = v + (w * self.wheel_base / 2.0)
        
        # For simplicity, assuming max speed is 1.0 m/s for scaling
        max_speed = 1.0 
        
        # Scale to -1.0 to 1.0
        left_cmd = v_left / max_speed
        right_cmd = v_right / max_speed
        
        self.set_motor_left(left_cmd)
        self.set_motor_right(right_cmd)

    def destroy_node(self):
        if GPIO:
            self.pwm_LA.stop()
            self.pwm_RA.stop()
            self.pwm_LB.stop()
            self.pwm_RB.stop()
            GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
