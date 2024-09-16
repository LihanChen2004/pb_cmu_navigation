from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='', description='Namespace for the node')
    
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    start_sensor_scan_generation = Node(
        package='sensor_scan_generation',
        executable='sensor_scan_generation_node',
        namespace=namespace,
        output='screen',
        remappings=remappings,
    )

    ld = LaunchDescription()

    # Add the actions
    ld.add_action(declare_namespace)
    ld.add_action(start_sensor_scan_generation)

    return ld