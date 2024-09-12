import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    checkTerrainConn = LaunchConfiguration('checkTerrainConn')

    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='', description='Namespace for the node')
    declare_checkTerrainConn = DeclareLaunchArgument(
        'checkTerrainConn', default_value='false', description='Check terrain connectivity')
    
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    start_terrain_analysis_ext = Node(
        package='terrain_analysis_ext',
        executable='terrainAnalysisExt',
        name='terrainAnalysisExt',
        namespace=namespace,
        output='screen',
        remappings=remappings,
        parameters=[{
            'scanVoxelSize': 0.1,
            'decayTime': 10.0,
            'noDecayDis': 0.0,
            'clearingDis': 30.0,
            'useSorting': False,
            'quantileZ': 0.1,
            'vehicleHeight': 1.5,
            'voxelPointUpdateThre': 100,
            'voxelTimeUpdateThre': 2.0,
            'lowerBoundZ': -2.5,
            'upperBoundZ': 1.0,
            'disRatioZ': 0.1,
            'checkTerrainConn': checkTerrainConn,
            'terrainConnThre': 0.5,
            'terrainUnderVehicle': -0.75,
            'ceilingFilteringThre': 2.0,
            'localTerrainMapRadius': 4.0,
        }]
    )

    ld = LaunchDescription()

    # Add the actions
    ld.add_action(declare_namespace)
    ld.add_action(declare_checkTerrainConn)
    ld.add_action(start_terrain_analysis_ext)

    return ld