"""盲人导航系统的一键启动编排。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare("blind_navigation_system")
    params_file = LaunchConfiguration("params_file")
    database_path = LaunchConfiguration("database_path")
    enable_perception = LaunchConfiguration("enable_perception")

    zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("zed_wrapper"), "launch", "zed_camera.launch.py"]
            )
        ),
        launch_arguments={
            "camera_model": "zed2i",
            "publish_tf": "true",
            "general.grab_resolution": "VGA",
            "general.grab_frame_rate": "15",
            "depth.point_cloud_freq": "5.0",
            "depth.publish_depth": "true",
            "pos_tracking.publish_map_tf": "false",
            "depth.point_cloud_organized": "false",
            "sensors.sensors_image_sync": "true",
        }.items(),
    )

    camera_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_to_base_transform",
        arguments=[
            "--x", "0", "--y", "0", "--z", "-0.8",
            "--yaw", "0", "--pitch", "0", "--roll", "0",
            "--frame-id", "zed_camera_link",
            "--child-frame-id", "base_link",
        ],
    )

    rosbridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge",
        output="screen",
    )

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("rtabmap_launch"), "launch", "rtabmap.launch.py"]
            )
        ),
        launch_arguments={
            "rtabmap_args": (
                "--Mem/IncrementalMemory false "
                "--Mem/InitWMWithAllNodes true "
                "--Vis/MinInliers 12 "
                "--RGBD/OptimizeMaxError 10.0 "
                "--RGBD/LoopClosureRecheck true"
            ),
            "database_path": database_path,
            "frame_id": "base_link",
            "rgb_topic": "/zed/zed_node/rgb/color/rect/image",
            "depth_topic": "/zed/zed_node/depth/depth_registered",
            "camera_info_topic": "/zed/zed_node/rgb/color/rect/camera_info",
            "odom_topic": "/zed/zed_node/odom",
            "approx_sync": "true",
            "wait_for_transform": "3.0",
            "qos": "1",
            "visual_odometry": "false",
            "rtabmap_viz": "false",
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": "false",
            "autostart": "true",
            "params_file": params_file,
        }.items(),
    )

    navigation_app = Node(
        package="blind_navigation_system",
        executable="blind-navigation",
        name="blind_guide_node",
        output="screen",
        parameters=[
            params_file,
            PathJoinSubstitution([package_share, "config", "navigation.yaml"]),
        ],
    )

    perception_app = Node(
        package="blind_navigation_system",
        executable="blind-perception",
        name="blind_perception",
        output="screen",
        condition=IfCondition(enable_perception),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "nav2_params.yaml"]
                ),
                description="Nav2 参数文件",
            ),
            DeclareLaunchArgument(
                "database_path",
                default_value=PathJoinSubstitution(
                    [EnvironmentVariable("HOME"), ".ros", "rtabmap.db"]
                ),
                description="RTAB-Map 定位数据库",
            ),
            DeclareLaunchArgument(
                "enable_perception",
                default_value="true",
                description="是否启动语音与视觉感知进程",
            ),
            zed,
            camera_transform,
            rosbridge,
            TimerAction(period=10.0, actions=[rtabmap]),
            TimerAction(period=25.0, actions=[nav2]),
            TimerAction(period=40.0, actions=[navigation_app]),
            TimerAction(period=45.0, actions=[perception_app]),
        ]
    )
