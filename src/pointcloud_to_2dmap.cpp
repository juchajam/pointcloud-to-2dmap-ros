#include "ros/ros.h"
#include <iostream>
#include <boost/filesystem.hpp>

#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


class MapGenerater
{
public:
  MapGenerater(double resolution, int map_width, int map_height, int min_points_in_pix, int max_points_in_pix, double min_height, double max_height)
  : resolution(resolution),
    m2pix(1.0 / resolution),
    map_width(map_width),
    map_height(map_height),
    min_points_in_pix(min_points_in_pix),
    max_points_in_pix(max_points_in_pix),
    min_height(min_height),
    max_height(max_height)
  {}

  cv::Mat generate(const pcl::PointCloud<pcl::PointXYZ>& cloud) const
  {
    cv::Mat map(map_height, map_width, CV_32SC1, cv::Scalar::all(0));

    for(const auto& point: cloud)
    {
      if(point.z < min_height || point.z > max_height)
      {
        continue;
      }

      int x = point.x * m2pix + map_width / 2;
      int y = -point.y * m2pix + map_width / 2;

      if(x < 0 || x >= map_width || y < 0 || y >= map_height)
      {
        continue;
      }

      map.at<int>(y, x) ++;
    }

    map -= min_points_in_pix;
    map.convertTo(map, CV_8UC1, - 255.0 / (max_points_in_pix - min_points_in_pix),  255);

    return map;
  }

public:
  const double resolution;    // meters per pixel
  const double m2pix;         // inverse resolution (pix/m)
  const int map_width;
  const int map_height;

  const int min_points_in_pix;
  const int max_points_in_pix;
  const double min_height;
  const double max_height;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_to_2dmap");

  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  double resolution;
  int map_width, map_height, min_points_in_pix, max_points_in_pix;
  double min_height, max_height;
  std::string input_pcd, dest_directory;
  
  if (!nh_priv.getParam("input_pcd", input_pcd))
  {
    ROS_ERROR("input_pcd param is not set");
    return 1;
  }

  if (!nh_priv.getParam("dest_directory", dest_directory))
  {
    ROS_ERROR("dest_directory param is not set");
    return 1;
  }

  nh_priv.param("resolution", resolution, 0.1);
  nh_priv.param("map_width", map_width, 1024);
  nh_priv.param("map_height", map_height, 1024);
  nh_priv.param("min_points_in_pix", min_points_in_pix, 2);
  nh_priv.param("max_points_in_pix", max_points_in_pix, 5);
  nh_priv.param("min_height", min_height, 0.5);
  nh_priv.param("max_height", max_height, 1.0);

  std::cout << "input_pcd     :" << input_pcd << std::endl;
  std::cout << "dest_directory:" << dest_directory << std::endl;
  std::cout << "resolution    :" << resolution << std::endl;

  auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if(pcl::io::loadPCDFile(input_pcd, *cloud))
  {
    ROS_ERROR("failed to open the input cloud");
    return 1;
  }

  MapGenerater generater(resolution, map_width, map_height, min_points_in_pix, max_points_in_pix, min_height, max_height);
  cv::Mat map = generater.generate(*cloud);

  if(!boost::filesystem::exists(dest_directory))
  {
    boost::filesystem::create_directories(dest_directory);
  }

  cv::imwrite(dest_directory + "/map.png", map);
  
  std::ofstream ofs(dest_directory + "/map.yaml");
  ofs << "image: map.png" << std::endl;
  ofs << "resolution: " << generater.resolution << std::endl;
  ofs << "origin: [" << -generater.resolution * generater.map_width / 2 << ", " << -generater.resolution * generater.map_height / 2 << ", 0.0]" << std::endl;
  ofs << "occupied_thresh: 0.5" << std::endl;
  ofs << "free_thresh: 0.2" << std::endl;
  ofs << "negate: 0" << std::endl;

  ROS_INFO("Finish converting point cloud to 2d map.");

  return 0;
}