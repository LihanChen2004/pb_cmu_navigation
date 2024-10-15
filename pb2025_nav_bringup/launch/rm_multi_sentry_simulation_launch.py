import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from nav2_common.launch import ParseMultiRobotPose


def generate_launch_description():
    """
    Bring up the multi-robots with given launch arguments.

    Launch arguments consist of robot name(which is namespace) and pose for initialization.
    Keep general yaml format for pose information.
    ex) robots:="robot1={x: 1.0, y: 1.0, yaw: 1.5707}; robot2={x: 1.0, y: 1.0, yaw: 1.5707}"
    ex) robots:="robot3={x: 1.0, y: 1.0, z: 1.0, roll: 0.0, pitch: 1.5707, yaw: 1.5707};
                 robot4={x: 1.0, y: 1.0, z: 1.0, roll: 0.0, pitch: 1.5707, yaw: 1.5707}"
    """
    # Get the launch directory
    bringup_dir = get_package_share_directory('pb2025_nav_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Simulation settings
    world = LaunchConfiguration('world')

    # On this example all robots are launched with the same settings
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    rviz_config_file = LaunchConfiguration('rviz_config')
    use_rviz = LaunchConfiguration('use_rviz')
    log_settings = LaunchConfiguration('log_settings', default='true')

    # Declare the launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value="rmul_2024",
        description="Select world: 'rmul_2024' or 'rmuc_2024' (map file share the same name as the this parameter)")

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=[bringup_dir, "/map/", world, ".yaml"],
        description='Full path to map file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'simulation', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='True',
        description='Automatically startup the stacks')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_dir, 'rviz', 'nav2_namespaced_view.rviz'),
        description='Full path to the RVIZ config file to use.')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    robots_list = ParseMultiRobotPose('robots').value()

    # Define commands for launching the navigation instances
    bringup_cmd_group = []
    for robot_name in robots_list:
        init_pose = robots_list[robot_name]
        group = GroupAction([
            LogInfo(msg=['Launching namespace=', robot_name, ' init_pose=', str(init_pose)]),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'rviz_launch.py')),
                condition=IfCondition(use_rviz),
                launch_arguments={'namespace': TextSubstitution(text=robot_name),
                                  'use_namespace': 'True',
                                  'rviz_config': rviz_config_file}.items()),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bringup_dir,
                                                           'launch',
                                                           'rm_sentry_simulation_launch.py')),
                launch_arguments={'namespace': robot_name,
                                  'use_namespace': 'True',
                                  'map': map_yaml_file,
                                  'use_sim_time': 'True',
                                  'params_file': params_file,
                                  'autostart': autostart,
                                  'use_rviz': 'False',
                                  'headless': 'False',
                                  'x_pose': TextSubstitution(text=str(init_pose['x'])),
                                  'y_pose': TextSubstitution(text=str(init_pose['y'])),
                                  'z_pose': TextSubstitution(text=str(init_pose['z'])),
                                  'roll': TextSubstitution(text=str(init_pose['roll'])),
                                  'pitch': TextSubstitution(text=str(init_pose['pitch'])),
                                  'yaw': TextSubstitution(text=str(init_pose['yaw'])),
                                  'robot_name':TextSubstitution(text=robot_name), }.items())
        ])

        bringup_cmd_group.append(group)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    # Add the actions to start gazebo, robots and simulations
    ld.add_action(LogInfo(msg=['number_of_robots=', str(len(robots_list))]))

    ld.add_action(LogInfo(condition=IfCondition(log_settings),
                          msg=['map yaml: ', map_yaml_file]))
    ld.add_action(LogInfo(condition=IfCondition(log_settings),
                          msg=['params yaml: ', params_file]))
    ld.add_action(LogInfo(condition=IfCondition(log_settings),
                          msg=['rviz config file: ', rviz_config_file]))
    ld.add_action(LogInfo(condition=IfCondition(log_settings),
                          msg=['autostart: ', autostart]))

    for cmd in bringup_cmd_group:
        ld.add_action(cmd)

    return ld