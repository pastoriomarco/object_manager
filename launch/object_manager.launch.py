# launch/object_manager_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the objects.yaml file
    config_dir = os.path.join(get_package_share_directory('object_manager'), 'config')
    objects_yaml = os.path.join(config_dir, 'objects.yaml')

    return LaunchDescription([
        # Launch the object_manager_node with a configurable frame_id
        Node(
            package='object_manager',
            executable='object_manager_node',
            name='object_manager_node',
            output='screen',
            parameters=[{'frame_id': 'world'}]  # You can change 'world' to your desired frame
        ),
        # Launch the collision_spawner node with object specifications from YAML
        Node(
            package='object_manager',
            executable='collision_spawner',
            name='collision_spawner',
            output='screen',
            parameters=[
                {'graspable_is_random': False},
                {'config_file': objects_yaml}
            ]
        )
    ])
