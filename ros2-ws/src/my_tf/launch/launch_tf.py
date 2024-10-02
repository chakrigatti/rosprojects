from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    
    rviz_config_path = os.path.join(
         get_package_share_path('my_tf'),
         'rviz', 'tf_rviz_config.rviz')
    

    return LaunchDescription([
         # Launch robot state publisher
        # Node(
        #     package='my_tf',
        #     executable='publish_tf',
        #     name='publish_tf'
        # ),

        # Launch RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        ),
    ])
