from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='inverse_kinematics',
            namespace='inverse_kinematics',
            executable='ik_solver_node',
            name='ik_solver_node',
        ),
        Node(
            package='joint_command',
            namespace='joint_command',
            executable='joint_command_node',
            name='joint_command_node',
        ),
        Node(
            package='servo_driver',
            namespace='servo_driver',
            executable='servo_node',
            name='servo_node',
        ),
        Node(
            package='gripper_control',
            namespace='gripper_control',
            executable='gripper_node',
            name='gripper_node',
        ),
        Node(
            package='trajectory_control',
            namespace='trajectory_control',
            executable='trajectory_control_node',
            name='trajectory_control_node',
        )
    ]
)
