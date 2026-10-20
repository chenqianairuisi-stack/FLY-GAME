"""Single launch for simulation, flight bridge, detector and full mission."""
import os
import atexit
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory
    config = LaunchConfiguration("config")
    actuator = Path(share("ican_delivery_throw"), "models/throw_actuator.sdf").read_text().replace(
        "libdelivery_throw.so", os.path.join(get_package_prefix("ican_delivery_throw"), "lib/libdelivery_throw.so"))
    with tempfile.NamedTemporaryFile(mode="w", suffix=".sdf", prefix="delivery_throw_", delete=False) as model_file:
        model_file.write(actuator)
    atexit.register(lambda: Path(model_file.name).unlink(missing_ok=True))
    mission = Node(package="ican_delivery_examples", executable="full_mission", output="screen",
        parameters=[config, {"use_sim_time": True,
            "drop_order": ParameterValue(LaunchConfiguration("drop_order"), value_type=str),
            "report_dir": LaunchConfiguration("report_dir")}])
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("drop_order", default_value="1234"),
        DeclareLaunchArgument("report_dir", default_value="demo_runs"),
        DeclareLaunchArgument("config", default_value=os.path.join(share("ican_delivery_examples"), "config/full_demo.yaml")),
        # The official launch evaluates its RViz condition in a delayed timer.
        # Preserve our option under another name, then disable its stock RViz.
        SetLaunchConfiguration("full_demo_rviz", LaunchConfiguration("rviz")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share("ican_delivery_sim"), "launch/delivery.launch.py")),
            launch_arguments={"gui": LaunchConfiguration("gui"), "rviz": "false"}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share("ican_delivery_flight_control"), "launch/flight_control.launch.py"))),
        Node(package="gazebo_ros", executable="spawn_entity.py", output="screen",
             arguments=["-entity", "throw_actuator", "-file", model_file.name]),
        Node(package="ican_delivery_examples", executable="window_detector", output="screen",
            parameters=[config, {"use_sim_time": True}]),
        TimerAction(period=6., actions=[Node(package="rviz2", executable="rviz2", output="screen",
            arguments=["-d", os.path.join(share("ican_delivery_examples"), "rviz/full_demo.rviz")],
            parameters=[{"use_sim_time": True}], condition=IfCondition(LaunchConfiguration("full_demo_rviz")))]),
        # Keep Gazebo/RViz open for inspection after completion.
        # The mission itself exits 0 on success, 1 on failure and saves a report.
        mission,
    ])
