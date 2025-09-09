import rclpy
from rclpy.node import Node

import time
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from digitalio import DigitalInOut
from board import SCL, SDA
from busio import I2C

from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

#this is a node that subscribes to /joint_angles(in deg) and then outputs a pwm to the servo motors to put them to that angle

class ServoDriver(Node):

    def __init__(self):
        super().__init__('servo_driver')
        
        self.subscription = self.create_subscription(
            JointState,
            '/joint_angles',
            self.angle_callback,
            10
        )

        #create an I2C Bus and pass it into a PCA9685 class
        i2c_bus = I2C(SCL, SDA)
        pca = PCA9685(i2c_bus)
        pca.frequency = 50 #50 Hz for servos

        #initialize servo and pass PCA9685 channel object to servo constructor
        self.servo_nums = 6
        self.my_servos = [
            servo.Servo(pca.channels[i]) for i in range (self.servo_nums)
            ]
    
    # callback that returns what angle the servo is supposed to be at
    # default min_pulse=750, max_pulse=2250, actuation range =180 deg
    def angle_callback(self, msg):
        for i, angle in enumerate (msg.position): #using msg.position here because the angle is in the position field of the jointstate message
            angle = max(0, min(180, angle))
            self.my_servos[i].angle = angle
            print(f"Servo set to {angle} degrees")


def main(args=None):
    rclpy.init(args=args)

    servo_driver = ServoDriver()

    rclpy.spin(servo_driver)

    servo_driver.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()