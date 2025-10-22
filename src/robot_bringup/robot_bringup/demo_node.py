import rclpy
import time
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Point, Quaternion

class DemoNode(Node):

    def __init__(self):
        super().__init__('demo_node')

        self.joint_publisher = self.create_publisher(
            Pose, '/ee_goal', 10
        )

        self.gripper_publisher = self.create_publisher(
            Bool, '/gripper_command', 10
        )

        #define locations
        self.home = Pose(
            position=Point(x=4.0, y=0.0, z=3.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )

        self.pick_up = Pose(
            position=Point(x=4.0, y=0.0, z=-1.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )

        self.place_above = Pose(
            position=Point(x=4.0, y=4.0, z=3.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )

        self.place = Pose(
            position=Point(x=4.0, y=4.0, z=-1.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )

    #define open and close for the gripper
        self.gripper_open = Bool(data=True)

        self.gripper_closed = Bool(data=False)

    # publish gripper command
    def publish_gripper(self, cmd: Bool, delay=1.0):
        self.gripper_publisher.publish(cmd)
        self.get_logger().info(f"Gripper command published: {cmd.data}")
        time.sleep(delay)

    #publish the pose
    def publish_pose(self, pose, delay=3.0):
        self.joint_publisher.publish(pose)
        self.get_logger().info(f"Moving to {pose.position.x}, {pose.position.y}, {pose.position.z}")
        time.sleep(delay)

    #run the sequence
    def run_sequence(self):
        self.get_logger().info("Starting pick and place sequence")        
        self.publish_pose(self.home)

        self.publish_pose(self.pick_up)

        self.publish_gripper(self.gripper_closed)
        self.get_logger().info("Gripper closed (picking object)")
        time.sleep(1.0)

        self.publish_pose(self.home)

        self.publish_pose(self.place_above)

        self.publish_pose(self.place)

        self.publish_gripper(self.gripper_open)
        self.get_logger().info("Gripper open (dropping object)")
        time.sleep(1.0)

        self.publish_pose(self.place_above)

        self.publish_pose(self.home)
        self.get_logger().info("Returned to home. Demo complete!")




def main(args=None):
    rclpy.init(args=args)

    demo_node = DemoNode()

    demo_node.run_sequence()

    rclpy.spin(demo_node)

    demo_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()