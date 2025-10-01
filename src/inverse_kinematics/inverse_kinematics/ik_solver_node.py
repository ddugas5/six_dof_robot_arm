import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

class IKSolverNode(Node):
    def __init__(self):
        super().__init__('ik_solver_node')
    # allow selecting elbow configuration: True -> prefer elbow-down (forearm falls down)
        self.declare_parameter('prefer_elbow_down', True)
        self.prefer_elbow_down = self.get_parameter('prefer_elbow_down').value

        self.subscription_ = self.create_subscription(Pose, '/ee_goal', self.goal_callback, 10) #subscribe to /ee_goal to recieve the goal orientation of the end effector

        self.publisher_ = self.create_publisher(JointState, '/joint_commands', 10) #publish what each joint angle should look like

    def goal_callback(self, msg: Pose):
        #geometric solver for theta_2, theta_3, theta_4
        def solve_remaining(x_w, y_w, z_w):
            L1 = 5.363 # shoulder height from the ground
            L2 = 3.5 #upper arm length
            L3 = 4.4 #forearm length
            r = math.sqrt(x_w**2+y_w**2)
            z_eff = z_w - L1

            self.get_logger().info(f"[DEBUG] wrist center (inches): x_w={x_w:.4f}, y_w={y_w:.4f}, z_w={z_w:.4f}")
            self.get_logger().info(f"[DEBUG] planar r={r:.4f} in, s={z_eff:.4f} in  (d1={L1:.4f} in)")

            # ensure target is within planar reach (L2+L3). If outside, scale slightly inside the reachable boundary
            planar_dist = math.hypot(r, z_eff)
            max_reach = L2 + L3
            if planar_dist > max_reach:
                scale = (max_reach / planar_dist) * 0.999  # nudge slightly inside
                r_scaled = r * scale
                z_eff_scaled = z_eff * scale
                self.get_logger().warning(
                    f"[WARN] wrist center out of reach: dist={planar_dist:.4f} in > max_reach={max_reach:.4f} in; scaling r/z_eff by {scale:.4f}"
                )
                self.get_logger().info(f"[DEBUG] scaled planar r={r_scaled:.4f} in, s={z_eff_scaled:.4f} in")
            else:
                r_scaled = r
                z_eff_scaled = z_eff

            # law of cosines for theta_3 -> there are two mirror solutions (elbow-up and elbow-down)
            D = ((r_scaled**2 + z_eff_scaled**2 - L2**2 - L3**2)/(2*L2*L3))
            D = max(min(D, 1), -1) # clamp to between -1 & 1

            # two possible elbow angles
            theta_3_up = math.acos(D)               # conventional positive (elbow-up)
            theta_3_down = -theta_3_up              # elbow-down (forearm falls the other way)

            # corresponding shoulder angles for each elbow choice
            theta_2_up = math.atan2(z_eff, r) - math.atan2(L3 * math.sin(theta_3_up), L2 + L3 * math.cos(theta_3_up))
            theta_2_down = math.atan2(z_eff, r) - math.atan2(L3 * math.sin(theta_3_down), L2 + L3 * math.cos(theta_3_down))

            # choose preferred configuration (allow parameter to select elbow-down)
            if getattr(self, 'prefer_elbow_down', True):
                theta_2, theta_3 = theta_2_down, theta_3_down
                chosen = 'elbow-down'
            else:
                theta_2, theta_3 = theta_2_up, theta_3_up
                chosen = 'elbow-up'

            # debug: which solution chosen
            self.get_logger().info(f"[DEBUG] chosen elbow configuration: {chosen}")

            # planar wrist for theta_4
            theta_4 = -(theta_2 + theta_3)

            # debug: angles
            deg = 180.0 / math.pi
            self.get_logger().info(
                f"[DEBUG] thetas (rad): t1={theta_1:.4f}, t2={theta_2:.4f}, t3={theta_3:.4f}, t4={theta_4:.4f}"
            )
            self.get_logger().info(
                f"[DEBUG] thetas (deg): t1={theta_1*deg:.1f}°, t2={theta_2*deg:.1f}°, t3={theta_3*deg:.1f}°, t4={theta_4*deg:.1f}°"
            )

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

        # debug: raw pose
        self.get_logger().info(f"[DEBUG] raw Pose: x={x:.4f}, y={y:.4f}, z={z:.4f}  (units: inches?)")

        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg. orientation.w]
        R = quat_to_matrix(q)   #convert quaternion to rotation matrix R
        z_ee = R[:, 2]  #end-effector z-axis
                        #the third column of R is the ee local z-axis expressed in the world/base frame
        # debug: orientation z axis
        self.get_logger().info(f"[DEBUG] z_ee (world frame): [{z_ee[0]:.4f}, {z_ee[1]:.4f}, {z_ee[2]:.4f}]")

        dist_to_wrist_center = 4.887  #distance from wrist center to EE tip

        #move target orientation from the ee tip to the wrist center
        #x_w, y_w, z_w is now your wrist orientation to solve for
        x_w = x - dist_to_wrist_center * z_ee[0]
        y_w = y - dist_to_wrist_center * z_ee[1]
        z_w = z - dist_to_wrist_center * z_ee[2]

        theta_1 = math.atan2(y_w, x_w)
        theta_2, theta_3, theta_4 = solve_remaining(x_w, y_w, z_w)
    
        joint_msg = JointState()
        joint_msg.name = ["joint_0", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
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
