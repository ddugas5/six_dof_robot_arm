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
        def solve_remaining(x, y, z):
            L1 = 5.363 # shoulder height from the ground
            L2 = 3.5 #upper arm length
            L3 = 4.4 #forearm length
            r = math.sqrt(x**2+y**2)
            # z_eff = z_w - L1

            # self.get_logger().info(f"[DEBUG] wrist center (inches): x_w={x_w:.4f}, y_w={y_w:.4f}, z_w={z_w:.4f}")
            # self.get_logger().info(f"[DEBUG] planar r={r:.4f} in, s={z_eff:.4f} in  (d1={L1:.4f} in)")

            # calculate theta_2 and theta_3 with the law of cosines
            phi_1 = np.arccos((L2**2 + r**2 - L3**2)/(2*r*L2))
            phi_2 = np.arctan(z/x)
            theta_2 = phi_2 + phi_1

            phi_3 = np.arccos((L3**2 + L2**2 - r**2) / (2 * L3 * L2))
            theta_3 = phi_3 + np.pi
            theta_3_motor_space = theta_3 - (np.pi * 2)
            theta_3_motor_space = theta_3_motor_space * -1.0

            # Define the desired orientation of the end effector relative to the base frame 
            # (i.e. global frame)
            # This is the target orientation.
            # The 3x3 rotation matrix of frame 6 relative to frame 0
            rot_mat_0_6 = np.array([[1.0, 0.0, 0.0],
                                    [0.0, 1.0, 0.0],
                                    [0.0, 0.0, 1.0]])

            # The 3x3 rotation matrix of frame 3 relative to frame 0
            rot_mat_0_3 = np.array([[(np.cos(theta_1))*(np.cos(theta_2 + theta_3)), -np.sin(theta_1), (np.cos(theta_1))*(np.sin(theta_2+theta_3))],
                                    [(np.sin(theta_1))*(np.cos(theta_2 + theta_3)), -np.cos(theta_1), (-np.sin(theta_1))*(np.sin(theta_2+theta_3))],
                                    [(-np.sin(theta_2+theta_3)), 0.0, (np.cos(theta_2+theta_3))]])

            #The 3x3 inverse rotation matrix of frame 3 relative to frame 0
            inv_rot_mat_0_3 = np.linalg.inv(rot_mat_0_3)

            #The 3x3 rotation matrix of frame 6 relative to frame 3
            rot_mat_3_6 = inv_rot_mat_0_3 @ rot_mat_0_6

            #Extract wrist angles, theta_4, theta_5, theta_6
            theta_5 = np.arccos(rot_mat_3_6[2, 2])
            # theta_6 = np.arctan2(rot_mat_3_6[2, 1], -rot_mat_3_6[2, 0])
            theta_4 = np.arctan2(rot_mat_3_6[1, 2], rot_mat_3_6[0, 2])

            return theta_2, theta_3_motor_space, theta_4, theta_5
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

        # debug: raw pose
        self.get_logger().info(f"[DEBUG] raw Pose: x={x:.4f}, y={y:.4f}, z={z:.4f}  (units: inches?)")

        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg. orientation.w]
        R = quat_to_matrix(q)   #convert quaternion to rotation matrix R
        z_ee = R[:, 2]  #end-effector z-axis
                        #the third column of R is the ee local z-axis expressed in the world/base frame
        # debug: orientation z axis
        self.get_logger().info(f"[DEBUG] z_ee (world frame): [{z_ee[0]:.4f}, {z_ee[1]:.4f}, {z_ee[2]:.4f}]")

        # dist_to_wrist_center = 4.887  #distance from wrist center to EE tip

        #move target orientation from the ee tip to the wrist center
        #x_w, y_w, z_w is now your wrist orientation to solve for
        # x_w = x - dist_to_wrist_center * z_ee[0]
        # y_w = y - dist_to_wrist_center * z_ee[1]
        # z_w = z - dist_to_wrist_center * z_ee[2]

        theta_1 = math.atan2(y, x)
        theta_2, theta_3_motor_space, theta_4, theta_5 = solve_remaining(x, y, z)
    
        joint_msg = JointState()
        joint_msg.name = ["joint_0", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
        joint_msg.position = [theta_1, theta_2, theta_3_motor_space, theta_4, theta_5, 0.0]
        self.publisher_.publish(joint_msg)

def main(args=None):
    rclpy.init(args=args)

    ik_solver_node = IKSolverNode()

    rclpy.spin(ik_solver_node)

    ik_solver_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
