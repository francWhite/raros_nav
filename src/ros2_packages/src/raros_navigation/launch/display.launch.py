import launch
import launch_ros
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rviz_path = os.path.join(get_package_share_directory('raros_navigation'), 'rviz')
    rviz_config_path = os.path.join(rviz_path, 'urdf_config.rviz')

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return launch.LaunchDescription([
        rviz_node
    ])
