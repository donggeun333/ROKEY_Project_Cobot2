from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config = LaunchConfiguration("config")
    execution_mode = LaunchConfiguration("execution_mode")
    use_gui = LaunchConfiguration("use_gui")
    transform_config = LaunchConfiguration("transform_config")
    auto_scan_motion = LaunchConfiguration("auto_scan_motion")
    bird_raise_mm = LaunchConfiguration("bird_raise_mm")
    grasp_z_offset_mm = LaunchConfiguration("grasp_z_offset_mm")
    gripper_preopen_width_mm = LaunchConfiguration(
        "gripper_preopen_width_mm"
    )
    local_scan_standoff_mm = LaunchConfiguration(
        "local_scan_standoff_mm"
    )
    calibration_path = PathJoinSubstitution(
        [
            FindPackageShare("tool_sorter_core"),
            "models",
            "T_gripper2camera.npy",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("tool_sorter_core"),
                        "config",
                        "tool_sorter.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "execution_mode",
                default_value="inspect",
                description="inspect | point | pick_place",
            ),
            DeclareLaunchArgument("use_gui", default_value="true"),
            DeclareLaunchArgument(
                "auto_scan_motion",
                default_value="false",
                description=(
                    "Enable Home/Bird/local scan motion after cell validation"
                ),
            ),
            DeclareLaunchArgument(
                "bird_raise_mm",
                default_value="200.0",
                description="Base +Z rise from Home TCP to Bird view",
            ),
            DeclareLaunchArgument(
                "grasp_z_offset_mm",
                default_value="-15.0",
                description=(
                    "Measured Base-Z grasp correction; negative moves lower"
                ),
            ),
            DeclareLaunchArgument(
                "gripper_preopen_width_mm",
                default_value="50.0",
                description="RG2 opening before approach and after release",
            ),
            DeclareLaunchArgument(
                "local_scan_standoff_mm",
                default_value="0.0",
                description="Camera optical standoff for local re-scan",
            ),
            DeclareLaunchArgument(
                "transform_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("tool_sorter_core"),
                        "config",
                        "timestamped_rgbd_transform.yaml",
                    ]
                ),
            ),
            Node(
                package="tool_sorter_core",
                executable="tcp_pose_tf_broadcaster",
                name="m0609_tcp_pose_tf_broadcaster",
                output="screen",
                parameters=[transform_config],
            ),
            Node(
                package="tool_sorter_core",
                executable="hand_eye_tf_broadcaster",
                name="m0609_hand_eye_tf_broadcaster",
                output="screen",
                # Use the calibration installed with this sorter instead of an
                # absolute developer-PC path from the transform config.
                parameters=[
                    transform_config,
                    {"transform_path": calibration_path},
                ],
            ),
            Node(
                package="tool_sorter_core",
                executable="perception_node",
                name="tool_sorter_perception",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="tool_sorter_core",
                executable="task_manager",
                name="tool_sorter_task_manager",
                output="screen",
                parameters=[
                    config,
                    {
                        "execution_mode": execution_mode,
                        "auto_scan_motion": ParameterValue(
                            auto_scan_motion,
                            value_type=bool,
                        ),
                        "bird_raise_mm": ParameterValue(
                            bird_raise_mm,
                            value_type=float,
                        ),
                        "grasp_z_offset_mm": ParameterValue(
                            grasp_z_offset_mm,
                            value_type=float,
                        ),
                        "gripper_preopen_width_mm": ParameterValue(
                            gripper_preopen_width_mm,
                            value_type=float,
                        ),
                        "local_scan_standoff_mm": ParameterValue(
                            local_scan_standoff_mm,
                            value_type=float,
                        ),
                    },
                ],
            ),
            Node(
                package="tool_sorter_core",
                executable="dashboard",
                name="tool_sorter_dashboard",
                output="screen",
                parameters=[config],
                condition=IfCondition(use_gui),
            ),
        ]
    )
