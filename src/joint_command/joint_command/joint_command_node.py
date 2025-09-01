import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

class JointCommandNode(Node):
    def __init__(self):
        super().__init__('joint_command_node')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_commands',
            self.joint_callback,
            10
        )

        #publish the joint command to the angle topic
        self.publisher_ = self.create_publisher(Float32, '/joint_angle', 10)

        #joint map containing joint number and servo channel number
        #only has one joint for now
        self.joint_map = {
            "joint_0": 0,
            "joint_1": 1,
            "joint_2": 2,
        }

    def joint_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.joint_map:
                angle = math.degrees(msg.position[i]) #convert to degrees
                angle_msg = Float32()                   #create new ros2 message of type float32
                angle_msg.data = angle                  #assigns the converted angle into the float32 message
                self.publisher_.publish(angle_msg)      #publish message

def main(args=None):
    rclpy.init(args=args)

    joint_command_node = JointCommandNode()

    rclpy.spin(joint_command_node)

    joint_command_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()