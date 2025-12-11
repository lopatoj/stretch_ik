from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    stretch_ik_path = get_package_share_path("stretch_ik")

    robot_description_content = Command(["xacro", str(stretch_ik_path / "urdf" / "stretch_ik.urdf")])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {
                "robot_description": ParameterValue(robot_description_content, value_type=str)
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", str(stretch_ik_path / "rviz" / "stretch_ik.rviz")],
    )

    # joint_state_publisher = Node(
    #     package="joint_state_publisher",
    #     executable="joint_state_publisher",
    #     output="both"
    # )

    ik_planner_node = Node(
        package="stretch_ik",
        executable="planner_node",
        output="screen",
        parameters=[
            {
                "ik_type": ParameterValue("KDL_NR_JL", value_type=str)
            }
        ]
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            rviz_node,
            # joint_state_publisher,
            ik_planner_node,
        ]
    )
