import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Bool

class GripperNode(Node):

    def __init__(self):
        super().__init__('gripper_node')

        self.subscription = self.create_subscription(
            Bool,
            '/gripper_command',
            self.gripper_callback,
            10
        )

        self.publisher_ = self.create_publisher(Float32, '/servo/joint_5/cmd', 10)

    def gripper_callback(self, msg):
        cmd_msg = Float32()
        command = msg.data
        if command == "True": #open the gripper
            cmd_msg = 180.0
            self.publisher_.publish(cmd_msg)
        else:
            cmd_msg = 0.0  #close the gripper
            self.publisher_.publish(cmd_msg)