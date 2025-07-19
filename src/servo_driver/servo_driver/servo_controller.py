
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        self.subscriber_ = self.create_subscription(Float32, '/servo_angle', self.angle_callback, 10)

        #Initialize I2C and PCA9685
        i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50

        self.channel = 0

        self.get_logger().info("servo controller node started")

    def angle_callback(self, msg):
        angle = max(0, min(180, msg.data)) #clamp to 180 degrees
        duty_cycle = self.angle_to_pwm(angle)
        self.pca.channels[self.channel].duty_cycle = duty_cycle
        self.get_logger().info(f"Moved servo to angle: {angle} -> PWM: {hex(duty_cycle)}")

    def angle_to_pwm(self, angle):
        #servo pulse range is about 0.5 ms (0 deg) to 2.5ms (180 deg)
        #for 16-bit pwm (0-65535, convert for 50 hz)

        min_pulse = 1000 #us
        max_pulse = 2000 #us
        pulse = int(min_pulse + (angle / 180.0) * (max_pulse - min_pulse))
        duty_cycle = int(pulse / 20000.0 * 65535)
        return duty_cycle

def main(args=None):
    rclpy.init(args=args)
    servo_controller = ServoController()
    rclpy.spin(servo_controller)
    servo_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()