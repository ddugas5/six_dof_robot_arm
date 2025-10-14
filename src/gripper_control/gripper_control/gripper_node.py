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

        #self.publisher_ = self.create_publisher()