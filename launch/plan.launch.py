from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    stretch_ik_path = get_package_share_path("stretch_ik")

    robot_description_ik = Command(["xacro ", str(stretch_ik_path / "urdf" / "stretch_ik.urdf")])

    ik_planner_node = Node(
        package="stretch_ik",
        executable="planner_node",
        output="screen",
        parameters=[
            {
                "ik_type": ParameterValue("KDL_NR_JL", value_type=str),
                "ik_urdf": ParameterValue(robot_description_ik, value_type=str)
            }
        ]
    )

    return LaunchDescription(
        [
            ik_planner_node,
        ]
    )
