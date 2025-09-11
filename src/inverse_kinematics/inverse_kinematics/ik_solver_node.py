import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

class IKSolverNode(Node):
    def __init__(self):
        super().__init__('ik_solver_node')

        self.subscription_ = self.create_subscription(Pose, '/ee_goal', self.goal_callback, 10) #subscribe to /ee_goal to recieve the goal orientation of the end effector

        self.publisher_ = self.create_publisher(JointState, '/joint_commands', 10) #publish what each joint angle should look like

    def goal_callback(self, msg: Pose):
        #geometric solver for theta_2, theta_3, theta_4
        def solve_remaining(x_w, y_w, z_w):
            L2 = 3.5
            L3 = 4.4
            r = math.sqrt(x_w**2+z_w**2)

            #law of cosines for theta_3
            D = ((r**2 - L2**2 - L3**2)/(2*L2*L3))
            D = max(min(D, 1), -1) #clamp to between -1 & 1
            theta_3 = math.acos(D)
            theta_2 = math.atan2(z_w, math.sqrt(x_w**2 + y_w**2)) - math.atan2(L3*math.sin(theta_3), L2 + L3*math.cos(theta_3))

            #planar wrist for theta_4
            theta_4 = -(theta_2 + theta_3)

            return theta_2, theta_3, theta_4
        #function for converting from a quaternion to a matrix
        def quat_to_matrix(q):
            #q = [x,y,z,w]
            x, y, z, w = q

            #standard formula
            R = np.array([
                [1-2*(y*y+z*z),   2*(x*y - z*w),   2*(x*z + y*w)],
                [2*(x*y + z*w),   1-2*(x*x+z*z),   2*(y*z - x*w)],
                [2*(x*z - y*w),   2*(y*z + x*w),   1-2*(x*x + y*y)]
            ])
            return R

        #extract the pose
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z

        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg. orientation.w]
        R = quat_to_matrix(q)   #convert quaternion to rotation matrix R
        z_ee = R[:, 2]  #end-effector z-axis
                        #the third column of R is the ee local z-axis expressed in the world/base frame
        dist_to_wrist_center = 0.1  #distance from wrist center to EE tip

        #move target orientation from the ee tip to the wrist center
        #x_w, y_w, z_w is now your wrist orientation to solve for
        x_w = x - dist_to_wrist_center * z_ee[0]
        y_w = y - dist_to_wrist_center * z_ee[1]
        z_w = z - dist_to_wrist_center * z_ee[2]

        theta_1 = math.atan2(y_w, x_w)
        theta_2, theta_3, theta_4 = solve_remaining(x_w, y_w, z_w)
    
        joint_msg = JointState()
        joint_msg.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        joint_msg.position = [theta_1, theta_2, theta_3, theta_4, 0.0, 0.0]
        self.publisher_.publish(joint_msg)

def main(args=None):
    rclpy.init(args=args)

    ik_solver_node = IKSolverNode()

    rclpy.spin(ik_solver_node)

    ik_solver_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
