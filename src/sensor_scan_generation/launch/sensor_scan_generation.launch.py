from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    lidar_frame = LaunchConfiguration("lidar_frame")
    vehicle_base_frame = LaunchConfiguration("vehicle_base_frame")
    vel_ref_frame = LaunchConfiguration("vel_ref_frame")

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="", description="Namespace for the node"
    )

    declare_lidar_frame = DeclareLaunchArgument(
        "lidar_frame",
        default_value="right_mid360",
        description="Frame ID for LiDAR sensor",
    )

    declare_vehicle_base_frame = DeclareLaunchArgument(
        "vehicle_base_frame",
        default_value="chassis",
        description="Frame ID for Vehicle Base",
    )

    declare_vel_ref_frame = DeclareLaunchArgument(
        "vel_ref_frame", default_value="gimbal_yaw", description="Frame ID for Gimbal"
    )

    start_sensor_scan_generation = Node(
        package="sensor_scan_generation",
        executable="sensor_scan_generation_node",
        namespace=namespace,
        output="screen",
        remappings=remappings,
        parameters=[
            {"lidar_frame": lidar_frame},
            {"vehicle_base_frame": vehicle_base_frame},
            {"vel_ref_frame": vel_ref_frame},
        ],
    )

    ld = LaunchDescription()

    # Add the actions
    ld.add_action(declare_namespace)
    ld.add_action(declare_lidar_frame)
    ld.add_action(declare_vehicle_base_frame)
    ld.add_action(declare_vel_ref_frame)
    ld.add_action(start_sensor_scan_generation)

    return ld
