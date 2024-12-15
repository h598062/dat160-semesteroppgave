import launch

from launch_ros.actions import Node


def generate_launch_description():
    tb1 = "/tb3_0"
    tb2 = "/tb3_1"
    tb1_wf = Node(
        package="final_iteration",
        executable="wall_follower",
        namespace=tb1,
        parameters=[{"namespace": tb1}],
    )
    tb1_controller = Node(
        package="final_iteration",
        executable="final_iteration",
        namespace=tb1,
        parameters=[{"namespace": tb1}],
    )
    tb2_wf = Node(
        package="final_iteration",
        executable="wall_follower",
        namespace=tb2,
        parameters=[{"namespace": tb2}],
    )
    tb2_controller = Node(
        package="final_iteration",
        executable="final_iteration",
        namespace=tb2,
        parameters=[{"namespace": tb2}],
    )
    return launch.LaunchDescription([tb1_wf, tb2_wf, tb1_controller, tb2_controller])
