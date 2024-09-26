from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    urdf_path = os.path.join(get_package_share_path('mycobot_arm_controller'),
                             'urdf', 'my_robot.urdf.xacro')
    rviz_config_path = os.path.join(
         get_package_share_path('mycobot_arm_controller'),
         'rviz', 'rviz_config.rviz')
    
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    return LaunchDescription([
         # Launch robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        
        # Launch joint controller script
        Node(
            package='mycobot_arm_controller',
            executable='control_joint',
            name='control_joint',
            output='screen',
        ),

        # Launch RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        ),
    ])
