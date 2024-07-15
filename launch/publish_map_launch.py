from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node 

import os

def generate_launch_description():

   
    config = os.path.join(
        get_package_share_directory("publish_map"),
        "config",
        "publish_map.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package='publish_map',
                executable='publish_map',
                name='publish_map',
		        output='screen',
                parameters=[config]
            )
        ]
    )