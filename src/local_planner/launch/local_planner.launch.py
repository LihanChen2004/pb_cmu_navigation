import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    sensorOffsetX = LaunchConfiguration('sensorOffsetX')
    sensorOffsetY = LaunchConfiguration('sensorOffsetY')
    cameraOffsetZ = LaunchConfiguration('cameraOffsetZ')
    twoWayDrive = LaunchConfiguration('twoWayDrive')
    maxSpeed = LaunchConfiguration('maxSpeed')
    autonomyMode = LaunchConfiguration('autonomyMode')
    autonomySpeed = LaunchConfiguration('autonomySpeed')
    joyToSpeedDelay = LaunchConfiguration('joyToSpeedDelay')
    goalX = LaunchConfiguration('goalX')
    goalY = LaunchConfiguration('goalY')

    declare_namespace = DeclareLaunchArgument('namespace', default_value='', description='Namespace for the node')
    declare_sensorOffsetX = DeclareLaunchArgument('sensorOffsetX', default_value='0.0', description='Sensor offset X')
    declare_sensorOffsetY = DeclareLaunchArgument('sensorOffsetY', default_value='0.0', description='Sensor offset Y')
    declare_cameraOffsetZ = DeclareLaunchArgument('cameraOffsetZ', default_value='0.0', description='Camera offset Z')
    declare_twoWayDrive = DeclareLaunchArgument('twoWayDrive', default_value='true', description='Two way drive')
    declare_maxSpeed = DeclareLaunchArgument('maxSpeed', default_value='2.0', description='Maximum speed')
    declare_autonomyMode = DeclareLaunchArgument('autonomyMode', default_value='true', description='Autonomy mode')
    declare_autonomySpeed = DeclareLaunchArgument('autonomySpeed', default_value='2.0', description='Autonomy speed')
    declare_joyToSpeedDelay = DeclareLaunchArgument('joyToSpeedDelay', default_value='2.0', description='Joy to speed delay')
    declare_goalX = DeclareLaunchArgument('goalX', default_value='0.0', description='Goal X')
    declare_goalY = DeclareLaunchArgument('goalY', default_value='0.0', description='Goal Y')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    start_local_planner = Node(
        package='local_planner',
        executable='localPlanner',
        name='localPlanner',
        namespace=namespace,
        output='screen',
        remappings=remappings,
        parameters=[{
            'pathFolder': os.path.join(get_package_share_directory('local_planner'), 'paths'),
            'vehicleLength': 0.6,
            'vehicleWidth': 0.45,
            'sensorOffsetX': sensorOffsetX,
            'sensorOffsetY': sensorOffsetY,
            'twoWayDrive': twoWayDrive,
            'laserVoxelSize': 0.05,
            'terrainVoxelSize': 0.2,
            'useTerrainAnalysis': True,
            'checkObstacle': True,
            'checkRotObstacle': False,
            'adjacentRange': 4.25,
            'obstacleHeightThre': 0.15,
            'groundHeightThre': 0.1,
            'costHeightThre': 0.1,
            'costScore': 0.02,
            'useCost': False,
            'pointPerPathThre': 2,
            'minRelZ': -0.5,
            'maxRelZ': 0.25,
            'maxSpeed': maxSpeed,
            'dirWeight': 0.02,
            'dirThre': 90.0,
            'dirToVehicle': False,
            'pathScale': 1.25,
            'minPathScale': 0.75,
            'pathScaleStep': 0.25,
            'pathScaleBySpeed': True,
            'minPathRange': 1.0,
            'pathRangeStep': 0.5,
            'pathRangeBySpeed': True,
            'pathCropByGoal': True,
            'autonomyMode': autonomyMode,
            'autonomySpeed': autonomySpeed,
            'joyToSpeedDelay': joyToSpeedDelay,
            'joyToCheckObstacleDelay': 5.0,
            'goalClearRange': 0.5,
            'goalX': goalX,
            'goalY': goalY,
        }]
    )

    start_path_follower = Node(
        package='local_planner',
        executable='pathFollower',
        name='pathFollower',
        namespace=namespace,
        output='screen',
        remappings=remappings,
        parameters=[{
            'sensorOffsetX': sensorOffsetX,
            'sensorOffsetY': sensorOffsetY,
            'pubSkipNum': 1,
            'twoWayDrive': twoWayDrive,
            'lookAheadDis': 0.5,
            'yawRateGain': 7.5,
            'stopYawRateGain': 7.5,
            'maxYawRate': 90.0,
            'maxSpeed': maxSpeed,
            'maxAccel': 2.5,
            'switchTimeThre': 1.0,
            # 'dirDiffThre': 0.1,
            'stopDisThre': 0.2,
            'slowDwnDisThre': 0.85,
            'useInclRateToSlow': False,
            'inclRateThre': 120.0,
            'slowRate1': 0.25,
            'slowRate2': 0.5,
            'slowTime1': 2.0,
            'slowTime2': 2.0,
            'useInclToStop': False,
            'inclThre': 45.0,
            'stopTime': 5.0,
            # 'noRotAtStop': False,
            # 'noRotAtGoal': True,
            'autonomyMode': autonomyMode,
            'autonomySpeed': autonomySpeed,
            'joyToSpeedDelay': joyToSpeedDelay,
        }]
    )

    start_vehicle_trans_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='vehicleTransPublisher',
        namespace=namespace,
        remappings=remappings,
        arguments=[
            '--x', sensorOffsetX,
            '--y', sensorOffsetY,
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', '/sensor',
            '--child-frame-id', '/vehicle'
        ],
    )

    start_sensor_trans_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sensorTransPublisher',
        namespace=namespace,
        remappings=remappings,
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', cameraOffsetZ,
            '--roll', '-1.5707963',
            '--pitch', '0.0',
            '--yaw', '-1.5707963',
            '--frame-id', '/sensor',
            '--child-frame-id', '/vehicle'
        ],
    )

    ld = LaunchDescription()

    # Add the actions
    ld.add_action(declare_namespace)
    ld.add_action(declare_sensorOffsetX)
    ld.add_action(declare_sensorOffsetY)
    ld.add_action(declare_cameraOffsetZ)
    ld.add_action(declare_twoWayDrive)
    ld.add_action(declare_maxSpeed)
    ld.add_action(declare_autonomyMode)
    ld.add_action(declare_autonomySpeed)
    ld.add_action(declare_joyToSpeedDelay)
    ld.add_action(declare_goalX)
    ld.add_action(declare_goalY)

    ld.add_action(start_local_planner)
    ld.add_action(start_path_follower)
    ld.add_action(start_vehicle_trans_publisher)
    ld.add_action(start_sensor_trans_publisher)

    return ld