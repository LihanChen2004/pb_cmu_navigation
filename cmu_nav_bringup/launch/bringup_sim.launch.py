import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import ReplaceString


def generate_launch_description():
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    cmu_nav_bringup_dir = get_package_share_directory("cmu_nav_bringup")

    namespace = LaunchConfiguration("namespace")
    world_name = LaunchConfiguration("world_name")
    cameraOffsetZ = LaunchConfiguration("cameraOffsetZ")
    vehicleX = LaunchConfiguration("vehicleX")
    vehicleY = LaunchConfiguration("vehicleY")
    checkTerrainConn = LaunchConfiguration("checkTerrainConn")

    point_lio_params_dir = os.path.join(
        cmu_nav_bringup_dir, "config", "simulation", "point_lio.yaml"
    )

    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="red_standard_robot1", description=""
    )
    declare_world_name = DeclareLaunchArgument(
        "world_name", default_value="garage", description=""
    )
    declare_cameraOffsetZ = DeclareLaunchArgument(
        "cameraOffsetZ", default_value="0.0", description=""
    )
    declare_vehicleX = DeclareLaunchArgument(
        "vehicleX", default_value="0.0", description=""
    )
    declare_vehicleY = DeclareLaunchArgument(
        "vehicleY", default_value="0.0", description=""
    )
    declare_checkTerrainConn = DeclareLaunchArgument(
        "checkTerrainConn", default_value="true", description=""
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("vehicle_simulator"),
        "rviz",
        "vehicle_simulator.rviz",
    )

    namespaced_rviz_config_file = ReplaceString(
        source_file=rviz_config_file,
        replacements={"<robot_namespace>": ("/", namespace)},
    )

    start_velodyne_convert_tool = Node(
        package="ign_sim_pointcloud_tool",
        executable="point_cloud_converter_node",
        namespace=namespace,
        remappings=remappings,
        parameters=[
            {
                "pcd_topic": "livox/lidar",
                "n_scan": 32,
                "horizon_scan": 1875,
                "ang_bottom": 7.0,
                "ang_res_y": 1.0,
            }
        ],
    )

    start_point_lio_node = Node(
        package="point_lio",
        executable="pointlio_mapping",
        namespace=namespace,
        parameters=[point_lio_params_dir],
        remappings=remappings,
        output="screen",
    )

    start_loam_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("loam_interface"),
                "launch",
                "loam_interface.launch.py",
            )
        )
    )

    start_local_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("local_planner"),
                "launch",
                "local_planner.launch.py",
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "cameraOffsetZ": cameraOffsetZ,
            "goalX": vehicleX,
            "goalY": vehicleY,
        }.items(),
    )

    start_terrain_analysis = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("terrain_analysis"),
                "launch",
                "terrain_analysis.launch.py",
            )
        ),
        launch_arguments={
            "namespace": namespace,
        }.items(),
    )

    start_terrain_analysis_ext = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("terrain_analysis_ext"),
                "launch",
                "terrain_analysis_ext.launch.py",
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "checkTerrainConn": checkTerrainConn,
        }.items(),
    )

    start_sensor_scan_generation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("sensor_scan_generation"),
                "launch",
                "sensor_scan_generation.launch.py",
            )
        ),
        launch_arguments={
            "namespace": namespace,
        }.items(),
    )

    # In the future, the transform will be provided by relocalization module
    start_map_trans_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="sensorTransPublisher",
        namespace=namespace,
        remappings=remappings,
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0.0",
            "--yaw",
            "0",
            "--frame-id",
            "map",
            "--child-frame-id",
            "odom",
        ],
    )

    start_visualization_tools = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("visualization_tools"),
                "launch",
                "visualization_tools.launch",
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "world_name": world_name,
        }.items(),
    )

    start_joy = Node(
        package="joy",
        executable="joy_node",
        name="ps3_joy",
        namespace=namespace,
        parameters=[
            {
                "dev": "/dev/input/js0",
                "deadzone": 0.12,
                "autorepeat_rate": 0.0,
            }
        ],
        output="screen",
    )

    start_rviz = Node(
        namespace=namespace,
        package="rviz2",
        executable="rviz2",
        arguments=["-d", namespaced_rviz_config_file],
        remappings=remappings,
        output="screen",
    )

    ld = LaunchDescription()

    # Add the actions
    ld.add_action(declare_namespace)
    ld.add_action(declare_world_name)
    ld.add_action(declare_cameraOffsetZ)
    ld.add_action(declare_vehicleX)
    ld.add_action(declare_vehicleY)
    ld.add_action(declare_checkTerrainConn)

    ld.add_action(start_point_lio_node)
    ld.add_action(start_velodyne_convert_tool)
    ld.add_action(TimerAction(period=5.0, actions=[start_loam_interface]))
    ld.add_action(TimerAction(period=5.0, actions=[start_sensor_scan_generation]))
    ld.add_action(TimerAction(period=5.0, actions=[start_terrain_analysis]))
    ld.add_action(TimerAction(period=5.0, actions=[start_terrain_analysis_ext]))
    ld.add_action(TimerAction(period=5.0, actions=[start_local_planner]))
    ld.add_action(TimerAction(period=5.0, actions=[start_map_trans_publisher]))

    # ld.add_action(start_visualization_tools)
    ld.add_action(start_joy)
    ld.add_action(start_rviz)

    return ld
