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
        if msg.data: #open the gripper
            angle = 90.0
        else:
            angle = 0.0  #close the gripper

        angle = max(0.0, min(180.0, float(angle)))
        cmd_msg.data = angle
        self.publisher_.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)

    gripper_node = GripperNode()

    rclpy.spin(gripper_node)

    gripper_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()