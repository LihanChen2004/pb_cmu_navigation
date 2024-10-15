import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from nav2_common.launch import ReplaceString, RewrittenYaml
from launch_ros.descriptions import ParameterFile


def generate_launch_description():
    bringup_dir = get_package_share_directory("cmu_nav_bringup")

    namespace = LaunchConfiguration("namespace")
    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    prior_pcd_file = (
        PathJoinSubstitution([bringup_dir, "pcd", "simulation", world]),
        ".pcd",
    )
    rviz_config_file = os.path.join(bringup_dir, "rviz", "cmu_nav.rviz")

    # Map fully qualified names to relative ones so the node"s namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn"t seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"use_sim_time": use_sim_time}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    namespaced_rviz_config_file = ReplaceString(
        source_file=rviz_config_file,
        replacements={"<robot_namespace>": ("/", namespace)},
    )

    point_lio_params_dir = os.path.join(
        bringup_dir, "config", "simulation", "point_lio.yaml"
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="red_standard_robot1", description=""
    )

    declare_world_name = DeclareLaunchArgument(
        "world", default_value="rmuc_2024", description=""
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="True", description=""
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            bringup_dir, "config", "simulation", "cmu_nav_params.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
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

    load_nodes = GroupAction(
        [
            PushRosNamespace(namespace=namespace),
            Node(
                package="loam_interface",
                executable="loamInterface",
                name="loam_interface",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="point_lio",
                executable="pointlio_mapping",
                name="point_lio",
                output="screen",
                parameters=[
                    point_lio_params_dir,
                    {"prior_pcd.prior_pcd_map_path": prior_pcd_file},
                ],
                remappings=remappings,
            ),
            Node(
                package="sensor_scan_generation",
                executable="sensor_scan_generation_node",
                name="sensor_scan_generation",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="terrain_analysis",
                executable="terrainAnalysis",
                name="terrain_analysis",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="terrain_analysis_ext",
                executable="terrainAnalysisExt",
                name="terrain_analysis_ext",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="local_planner",
                executable="localPlanner",
                name="local_planner",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="local_planner",
                executable="pathFollower",
                name="path_follower",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
        ]
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

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace)
    ld.add_action(declare_world_name)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the actions
    ld.add_action(start_velodyne_convert_tool)
    ld.add_action(load_nodes)
    ld.add_action(start_map_trans_publisher)

    # Options
    ld.add_action(start_joy)
    ld.add_action(start_rviz)

    return ld
