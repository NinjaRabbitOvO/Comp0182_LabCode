
"""Connect explore_lite to the Lab 7 navigation system."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    robot_name = LaunchConfiguration("robot_name")
    use_sim_time = LaunchConfiguration("use_sim_time")

    action_endpoints = [
        "send_goal",
        "get_result",
        "cancel_goal",
        "feedback",
        "status",
    ]

    remappings = [
        ("navigate_to_pose", "/navigate_to_pose"),
    ]

    remappings += [
        (
            f"navigate_to_pose/_action/{endpoint}",
            f"/navigate_to_pose/_action/{endpoint}",
        )
        for endpoint in action_endpoints
    ]

    return LaunchDescription([

        DeclareLaunchArgument(
            "robot_name",
            description="Full robot namespace",
        ),

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            choices=["true", "false"],
        ),

        Node(
            package="explore_lite",
            executable="explore",
            namespace=robot_name,
            name="explore_node",
            output="screen",

            parameters=[{
                "use_sim_time": use_sim_time,

                "costmap_topic": "/map",

                "costmap_updates_topic": "/map_updates",

                "robot_base_frame": [
                    robot_name,
                    "/base_footprint",
                ],

                "return_to_init": False,
            }],

            remappings=remappings,
        ),
    ])
