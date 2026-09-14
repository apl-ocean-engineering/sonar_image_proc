import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LifecycleNode, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

def generate_launch_description():

    draw_sonar_config = os.path.join(get_package_share_directory('sonar_image_proc'),'config','draw_sonar_params.yaml')

    draw_sonar_node = LifecycleNode(
        package='sonar_image_proc',
        executable= 'draw_sonar_node',
        name='draw_sonar_node',
        namespace='oculus',
        parameters=[draw_sonar_config],

    )

    return LaunchDescription(
        [
          draw_sonar_node
        ]
    )


