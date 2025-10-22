import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState

class TrajectoryControl(Node):
    def __init__(self):
        super().__init__('trajectory_control_node')
        
        self.subscription = self.create_subscription(
            JointState,
            '/joint_commands',
            self.target_callback,
            10
        )

        self.publisher_ = self.create_publisher(JointState, '/trajectory_commands', 10)
    
        self.current_position = None

        self.rate = 50.0 

    def target_callback(self, msg):
        target = np.array(msg.position) #convert to a numpy array for math

        if target.size == 0:
            self.get_logger().warning('Received JointState with empty position; ignoring')
            return
        
        if self.current_position is None:
            # initialize to zeros matching incoming joint count
            self.current_position = np.zeros_like(target)
            self.joint_names = [f"joint_{i}" for i in range(target.size)]
        
        elif target.shape != self.current_position.shape:
            self.get_logger().error(f'Target length {target.shape[0]} does not match current_position length {self.current_position.shape[0]}')
            return
        
        self.move_to_target(target)     #call function move to target for interpolation

    def move_to_target(self, target): #when the target pose is received, linearly interpolate between the that and current position
        steps = 150                 #100 steps between target and current pos
        for i in np.linspace(0, 1, steps):
            step = self.current_position + i * (target - self.current_position) #linear interpolation
            msg = JointState()              #create joint state message
            msg.name = self.joint_names
            msg.position = step.tolist()    #change from numpy array to list to satisfy ros message type
            self.publisher_.publish(msg)
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.0/self.rate))
        self.current_position = target      # set the target to be the new current position at the end
        self.get_logger().info("Reached target position")


def main(args=None):
    rclpy.init(args=args)

    trajectory_control_node = TrajectoryControl()

    rclpy.spin(trajectory_control_node)

    trajectory_control_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
