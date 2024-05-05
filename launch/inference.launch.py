import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    camera_name = "zed"
    camera_model = "zed2i"

    zed_wrapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("zed_wrapper"),
                "launch",
                "zed_camera.launch.py",
            ),
        ),
        launch_arguments={
            "camera_name": camera_name,
            "camera_model": camera_model,
            "ros_params_override_path": os.path.join(
                get_package_share_directory("viva_demo"),
                "config",
                "camera_override.yaml",
            ),
        }.items(),
    )

    ld.add_action(zed_wrapper)

    inference = Node(
        package="viva_demo",
        executable="inference",
        remappings=[("image_rect", f"/{camera_name}/zed_node/left/image_rect_color")],
    )

    ld.add_action(inference)

    return ld
