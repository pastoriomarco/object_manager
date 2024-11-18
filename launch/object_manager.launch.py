# launch/object_manager_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the object_manager_node with a configurable frame_id
        Node(
            package='object_manager',
            executable='object_manager_node',
            name='object_manager_node',
            output='screen',
            parameters=[{'frame_id': 'world'}]  # You can change 'world' to your desired frame
        ),
        # Launch the collision_spawner node
        Node(
            package='object_manager',
            executable='collision_spawner',
            name='collision_spawner',
            output='screen',
            parameters=[{'graspable_is_random': False}]
        )
    ])
