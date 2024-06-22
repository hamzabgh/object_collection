from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('object_collection_py'),
        'urdf',
        'drone.urdf'
    )

    box_urdf_file = os.path.join(
        get_package_share_directory('object_collection_py'),
        'urdf',
        'box.urdf'
    )

    model_sdf_file = os.path.join(
        get_package_share_directory('object_collection_py'),
        'urdf',
        'model.sdf'
    )

    controller_config = os.path.join(
        get_package_share_directory('object_collection_py'),
        'config',
        'controller.yaml'
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=[
                '-entity', 'Dron_gemini',
                '-x', '0',
                '-y', '0',
                '-z', '0.25',  # Adjust the drone's Z position as needed
                '-file', urdf_file,
                '-robot_namespace', '/',
            ]
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_box',
            output='screen',
            arguments=[
                '-entity', 'box',
                '-x', '2',  # Adjust the box's X position as needed
                '-y', '0',
                '-z', '0.5',  # Adjust the box's Z position as needed
                '-file', box_urdf_file
            ]
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_model_sdf',
            output='screen',
            arguments=[
                '-entity', 'custom_model',  # Adjust entity name as needed
                '-x', '4',
                '-y', '0',
                '-z', '0',  # Adjust position as needed
                '-file', model_sdf_file
            ]
        ),
        Node(
            package='object_collection_py',
            executable='object_collection_node',
            name='object_collection_node',
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controller_config],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        ),
    ])
