import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ros2_ws = os.path.expanduser("~/ros2_ws")

    target_index = LaunchConfiguration("target_index")
    p_min = LaunchConfiguration("p_min")
    p_max = LaunchConfiguration("p_max")

    nodes_dir = os.path.join(
        ros2_ws,
        "src/crospi_application/ala_reaching_nodes/ala_reaching_nodes"
    )

    crospi_config = "$[crospi_application]/applications/application_ala_reaching/application_ala_reaching.setup.json"

    crospi_app_share = get_package_share_directory("crospi_application")

    urdf_file = os.path.join(
        crospi_app_share,
        "robot_models/urdf_models/robot_setups/ur10e/use_case_setup_ur10e.urdf"
    )

    rviz_file = os.path.join(
        crospi_app_share,
        "robot_models/urdf_models/robot_setups/ur10e/rviz_config.rviz"
    )

    with open(urdf_file, "r") as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument("target_index", default_value="1"),
        DeclareLaunchArgument("p_min", default_value="0.40"),
        DeclareLaunchArgument("p_max", default_value="0.50"),

        # RViz support
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_desc}],
            arguments=[urdf_file],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_file],
        ),

        # Crospi lifecycle node
        Node(
            package="crospi_core",
            executable="crospi_node",
            name="crospi_node",
            output="screen",
            parameters=[
                {"config_file": crospi_config},
                {"simulation": True},
            ],
        ),

        # ALA support nodes
        ExecuteProcess(
            cmd=[
                "python3",
                os.path.join(nodes_dir, "mock_static_hmi_node.py"),
                "--ros-args",
                "-p", ["target_index:=", target_index],
            ],
            output="screen",
        ),

        ExecuteProcess(
            cmd=[
                "python3",
                os.path.join(nodes_dir, "intent_estimator_node.py"),
            ],
            output="screen",
        ),

        ExecuteProcess(
            cmd=[
                "python3",
                os.path.join(nodes_dir, "autonomy_manager_node.py"),
                "--ros-args",
                "-p", ["p_min:=", p_min],
                "-p", ["p_max:=", p_max],
            ],
            output="screen",
        ),

        ExecuteProcess(
            cmd=[
                "python3",
                os.path.join(nodes_dir, "target_blender_node.py"),
            ],
            output="screen",
        ),

        ExecuteProcess(
            cmd=[
                "python3",
                os.path.join(nodes_dir, "target_latch_node.py"),
            ],
            output="screen",
        ),
    ])
