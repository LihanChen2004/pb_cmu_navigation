import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("cmu_nav_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    world = LaunchConfiguration("world")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    use_respawn = LaunchConfiguration("use_respawn")

    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_rviz = LaunchConfiguration("use_rviz")

    map_yaml_file = PathJoinSubstitution([bringup_dir, "map", world]), ".yaml"
    prior_pcd_file = PathJoinSubstitution([bringup_dir, "pcd", "simulation", world]), ".pcd"
    point_lio_config_file = os.path.join(bringup_dir, "config", "simulation", "point_lio.yaml")

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # Declare the launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value="rmul_2024",
        description="Select world: 'rmul_2024' or 'rmuc_2024' (map file share the same name as the this parameter)"
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="red_standard_robot1", description="Top-level namespace"
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        "use_namespace",
        default_value="true",
        description="Whether to apply a namespace to the navigation stack",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if True",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "config", "simulation", "cmu_nav_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(bringup_dir, "rviz", "cmu_nav.rviz"),
        description="Full path to the RVIZ config file to use",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RVIZ"
    )

    start_velodyne_convert_tool = Node(
        package="ign_sim_pointcloud_tool",
        executable="point_cloud_converter_node",
        name="point_cloud_converter",
        output="screen",
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
        name="point_lio",
        output="screen",
        namespace=namespace,
        parameters=[point_lio_config_file, {"prior_pcd.prior_pcd_map_path": prior_pcd_file}],
        remappings=remappings,
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "rviz_launch.py")),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "namespace": namespace,
            "use_namespace": use_namespace,
            "use_sim_time": use_sim_time,
            "rviz_config": rviz_config_file,
        }.items(),
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "bringup_launch.py")),
        launch_arguments={
            "namespace": namespace,
            "use_namespace": use_namespace,
            "map": map_yaml_file,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "use_respawn": use_respawn,
        }.items(),
    )

    start_joy = Node(
        package="joy",
        executable="joy_node",
        name="ps3_joy",
        output="screen",
        namespace=namespace,
        parameters=[
            {
                "dev": "/dev/input/js0",
                "deadzone": 0.12,
                "autorepeat_rate": 0.0,
            }
        ],
    )

    # In the future, the transform will be provided by relocalization module
    start_map_trans_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map2odom_trans_publisher",
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

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_respawn_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_velodyne_convert_tool)
    ld.add_action(start_point_lio_node)
    ld.add_action(start_map_trans_publisher)
    ld.add_action(bringup_cmd)
    ld.add_action(start_joy)
    ld.add_action(rviz_cmd)

    return ld