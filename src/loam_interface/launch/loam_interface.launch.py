import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='', description='Namespace for the node')

    start_loam_interface = Node(
        package='loam_interface',
        executable='loamInterface',
        name='loamInterface',
        namespace=namespace,
        remappings=remappings,
        output='screen',
        parameters=[{
            'stateEstimationTopic': 'aft_mapped_to_init',
            'registeredScanTopic': 'cloud_registered',
            'flipStateEstimation': False,
            'flipRegisteredScan': False,
            'sendTF': False,
            'reverseTF': False,
        }]
    )

    ld = LaunchDescription()

    # Add the actions
    ld.add_action(declare_namespace)
    ld.add_action(start_loam_interface)

    return ld