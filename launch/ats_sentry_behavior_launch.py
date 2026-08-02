from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap


def generate_launch_description():
    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    log_level = LaunchConfiguration("log_level")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        description=(
            "Required full path to the behavior parameters. The formal profile passes "
            "ats_sentry_bringup/params/node_params.yaml; examples and tests must opt in explicitly."
        ),
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(namespace=namespace),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            Node(
                package="ats_sentry_behavior",
                executable="ats_sentry_behavior_server",
                name="ats_sentry_behavior_server",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="ats_sentry_behavior",
                executable="ats_sentry_behavior_client",
                name="ats_sentry_behavior_client",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
                arguments=["--ros-args", "--log-level", log_level],
            ),
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch the nodes
    ld.add_action(bringup_cmd_group)

    return ld
