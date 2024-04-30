import rospy
import os
import cv2
import numpy as np
import pcl
from map_generater import MapGenerater


if __name__ == "__main__":
    rospy.init_node("pointcloud_to_2dmap_node")

    input_pcd = rospy.get_param("~input_pcd", "")
    if input_pcd == "":
        rospy.logerr("input_pcd param is not set")
        exit()

    dest_directory = rospy.get_param("~dest_directory", "")
    if dest_directory == "":
        rospy.logerr("dest_directory param is not set")
        exit()

    resolution = rospy.get_param("resolution", 0.1)
    map_width = rospy.get_param("map_width", 1024)
    map_height = rospy.get_param("map_height", 1024)
    min_points_in_pix = rospy.get_param("min_points_in_pix", 2)
    max_points_in_pix = rospy.get_param("max_points_in_pix", 5)
    min_height = rospy.get_param("min_height", 0.5)
    max_height = rospy.get_param("max_height", 1.0)

    print(f"input_pcd     :{input_pcd}")
    print(f"dest_directory:{dest_directory}")
    print(f"resolution    :{resolution}")

    cloud = pcl.load(input_pcd)

    generater = MapGenerater(input_pcd, dest_directory, resolution, map_width, map_height, min_points_in_pix, max_points_in_pix, min_height, max_height)
    generater.generate(cloud)
    generater.save_map(dest_directory)

    rospy.loginfo("Finish converting point cloud to 2d map.")