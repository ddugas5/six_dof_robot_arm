import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

class IKSolverNode(Node):
    def __init__(self):
        super().__init__('ik_solver_node')

        self.subscription_ = self.create_subscription(Pose, '/ee_goal', self.goal_callback, 10)

        self.publisher_ = self.create_publisher(JointState, '/joint_command', 10)

    def goal_callback(self, msg: Pose):
        #extract the pose
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z

def main(args=None):
    rclpy.init(args=args)

    ik_solver_node = IKSolverNode()

    rclpy.spin(ik_solver_node)

    ik_solver_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
