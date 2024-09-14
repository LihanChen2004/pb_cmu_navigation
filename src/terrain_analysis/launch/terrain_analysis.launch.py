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

    start_terrain_analysis = Node(
        package='terrain_analysis',
        executable='terrainAnalysis',
        name='terrainAnalysis',
        namespace=namespace,
        output='screen',
        remappings=remappings,
        parameters=[{
            'scanVoxelSize': 0.05,
            'decayTime': 2.0,
            'noDecayDis': 4.0,
            'clearingDis': 8.0,
            'useSorting': False,
            'quantileZ': 0.25,
            'considerDrop': True,
            'limitGroundLift': False,
            'maxGroundLift': 0.15,
            'clearDyObs': True,
            'minDyObsDis': 0.3,
            'minDyObsAngle': 0.0,
            'minDyObsRelZ': -0.5,
            'absDyObsRelZThre': 0.2,
            'minDyObsVFOV': -16.0,
            'maxDyObsVFOV': 16.0,
            'minDyObsPointNum': 1,
            'noDataObstacle': False,
            'noDataBlockSkipNum': 0,
            'minBlockPointNum': 10,
            'vehicleHeight': 0.4,
            'voxelPointUpdateThre': 100,
            'voxelTimeUpdateThre': 2.0,
            'minRelZ': -2.5,
            'maxRelZ': 1.0,
            'disRatioZ': 0.2,
        }]
    )

    ld = LaunchDescription()

    # Add the actions
    ld.add_action(declare_namespace)
    ld.add_action(start_terrain_analysis)

    return ld