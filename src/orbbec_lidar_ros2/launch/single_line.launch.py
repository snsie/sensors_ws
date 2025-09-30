from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get config file path
    config_file = os.path.join(get_package_share_directory('orbbec_lidar_ros2'), 'config', 'single_line_lidar.toml')

    # Declare arguments
    args = [
        DeclareLaunchArgument('lidar_name', default_value='orbbec_lidar'),
        DeclareLaunchArgument('type', default_value='single_line'),
        DeclareLaunchArgument('frame_id', default_value='scan'),
        DeclareLaunchArgument('data_type', default_value='laserscan'),
        DeclareLaunchArgument('data_qos', default_value='sensor_data'),
        DeclareLaunchArgument('use_intra_process', default_value='false'),
        DeclareLaunchArgument('min_angle', default_value='-135.0'),
        DeclareLaunchArgument('max_angle', default_value='135.0'),
        DeclareLaunchArgument('min_range', default_value='0.1'),
        DeclareLaunchArgument('max_range', default_value='30.0'),
        DeclareLaunchArgument('scan_frequency', default_value='30.0'),
        DeclareLaunchArgument('config_file', default_value=config_file),
        DeclareLaunchArgument('shared_container_name', default_value='orbbec_lidar_container'),
        DeclareLaunchArgument('attach_to_shared_container', default_value='false'),
        DeclareLaunchArgument('use_hardware_time', default_value='false'),
        DeclareLaunchArgument('filter_level', default_value='0'),
        DeclareLaunchArgument('enable_smoothing_filter', default_value='false'),
    ]

    # Node configuration
    parameters = [{arg.name: LaunchConfiguration(arg.name)} for arg in args]

    # Get ROS_DISTRO
    ros_distro = os.environ["ROS_DISTRO"]

    # If using Foxy, launch the normal node
    if ros_distro == "foxy":
        return LaunchDescription(
            args
            + [
                Node(
                    package="orbbec_lidar_ros2",
                    executable="orbbec_lidar_node",
                    name="orbbec_lidar",
                    namespace=LaunchConfiguration("lidar_name"),
                    parameters=parameters,
                    output="screen",
                )
            ]
        )

    # Define the ComposableNode
    compose_node = ComposableNode(
        package="orbbec_lidar_ros2",
        plugin="ob_lidar::OBLidarDriver",
        name=LaunchConfiguration("lidar_name"),
        namespace="orbbec_lidar",
        parameters=parameters,
    )

    # Logic to attach to an external container if 'attach_to_shared_container' is true
    load_composable_nodes = LoadComposableNodes(
        composable_node_descriptions=[compose_node],
        target_container=LaunchConfiguration('shared_container_name'),
        condition=IfCondition(LaunchConfiguration('attach_to_shared_container'))
    )

    # Define the ComposableNodeContainer if no external container is provided
    container = ComposableNodeContainer(
        name=LaunchConfiguration('shared_container_name'),
        namespace="orbbec_lidar",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[compose_node],
        output="screen",
        condition=UnlessCondition(LaunchConfiguration('attach_to_shared_container')),
    )

    # Launch description
    ld = LaunchDescription(
        args
        + [
            GroupAction(
                [
                    PushRosNamespace(LaunchConfiguration("lidar_name")),
                    load_composable_nodes,  # Load into an existing container if specified
                    container  # Otherwise, use a new container
                ]
            )
        ]
    )

    return ld
