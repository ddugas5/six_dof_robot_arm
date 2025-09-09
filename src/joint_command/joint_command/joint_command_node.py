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
        self.publisher_ = self.create_publisher(JointState, '/joint_angles', 10)

        #joint map containing joint number and servo channel number
        self.joint_map = {
            "joint_0": 0,
            "joint_1": 1,
            "joint_2": 2,
            "joint_3": 3,
            "joint_4": 4,
            "joint_5": 5,
        }

    def joint_callback(self, msg): #publish list of joint names and associated angles
        angle_msg = JointState() #create new ros2 message of type joint state
        angle_msg.name = []   #make a list for the joint names
        angle_msg.position = []     #make a list for joint angles
        for i, name in enumerate(msg.name):  #iterate through incoming message, 2 values, name and position
            if name in self.joint_map:      #if the name is in the joint_map list continue
                angle = math.degrees(msg.position[i]) #convert i-th position to degrees
                angle_msg.name.append(name)     #append name to the above list
                angle_msg.position.append(angle)    #append position to above list
        self.publisher_.publish(angle_msg)      #publish message

def main(args=None):
    rclpy.init(args=args)

    joint_command_node = JointCommandNode()

    rclpy.spin(joint_command_node)

    joint_command_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()