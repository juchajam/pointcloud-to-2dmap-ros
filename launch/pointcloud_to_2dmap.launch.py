import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    pkg_irop_databse = get_package_share_directory("irop_database")

    input_pcd = LaunchConfiguration("input_pcd")
    dest_directory = LaunchConfiguration("dest_directory")
    resolution = LaunchConfiguration("resolution")
    map_width = LaunchConfiguration("map_width")
    map_height = LaunchConfiguration("map_height")
    min_points_in_pix = LaunchConfiguration("min_points_in_pix")
    max_points_in_pix = LaunchConfiguration("max_points_in_pix")
    min_height = LaunchConfiguration("min_height")
    max_height = LaunchConfiguration("max_height")

    return LaunchDescription(
        [
            DeclareLaunchArgument("input_pcd", description="Input PCD file"),
            DeclareLaunchArgument(
                "dest_directory", description="Destination directory"
            ),
            DeclareLaunchArgument(
                "resolution",
                default_value="0.05",
                description="Pixel resolution (meters / pix)",
            ),
            DeclareLaunchArgument(
                "map_width", default_value="1024", description="Map width [pix]"
            ),
            DeclareLaunchArgument(
                "map_height", default_value="1024", description="Map height [pix]"
            ),
            DeclareLaunchArgument(
                "min_points_in_pix",
                default_value="2",
                description="Min points in a occupied pix",
            ),
            DeclareLaunchArgument(
                "max_points_in_pix",
                default_value="5",
                description="Max points in a pix for saturation",
            ),
            DeclareLaunchArgument(
                "min_height",
                default_value="0.5",
                description="Min height of clipping range",
            ),
            DeclareLaunchArgument(
                "max_height",
                default_value="1.0",
                description="Max height of clipping range",
            ),
            Node(
                package="pointcloud_to_2dmap",
                executable="pointcloud_to_2dmap",
                parameters=[
                    {"input_pcd": input_pcd},
                    {"dest_directory": dest_directory},
                    {"resolution": resolution},
                    {"map_width": map_width},
                    {"map_height": map_height},
                    {"min_points_in_pix": min_points_in_pix},
                    {"max_points_in_pix": max_points_in_pix},
                    {"min_height": min_height},
                    {"max_height": max_height},
                ],
                output="screen",
            ),
        ]
    )
