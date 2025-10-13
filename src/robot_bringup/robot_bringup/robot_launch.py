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
            executable='servo_driver_node',
            name='servo_driver_node',
        )
    ]
)
