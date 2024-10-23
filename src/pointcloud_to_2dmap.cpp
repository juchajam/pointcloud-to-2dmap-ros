#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <filesystem>

#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


class MapGenerater : rclcpp::Node
{
public:
  MapGenerater(rclcpp::NodeOptions options = rclcpp::NodeOptions())
  : Node("pointcloud_to_2dmap", options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true))
  {
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    
    this->declare_parameter("resolution", 0.1);
    this->declare_parameter("map_width", 1024);
    this->declare_parameter("map_height", 1024);
    this->declare_parameter("min_points_in_pix", 2);
    this->declare_parameter("max_points_in_pix", 5);
    this->declare_parameter("min_height", 0.5);
    this->declare_parameter("max_height", 1.0);
    this->declare_parameter("dest_directory", "");

    std::string input_pcd, dest_directory;

    if this->get_parameter("resolution", resolution)
    m2pix = 1.0 / resolution;
    map_width = this->get_parameter("map_width").as_int();
    map_height = this->get_parameter("map_height").as_int();
    min_points_in_pix = this->get_parameter("min_points_in_pix").as_int();
    max_points_in_pix = this->get_parameter("max_points_in_pix").as_int();
    min_height = this->get_parameter("min_height").as_double();
    max_height = this->get_parameter("max_height").as_double();
    dest_directory = this->get_parameter("").as_string();


    if(this->get_parameter("input_pcd", input_pcd)!=true)
    {
      RCLCPP_ERROR(this->get_logger(), "input_pcd param is not set.");
      rclcpp::shutdown();
    }
    

    if (!nh_priv.getParam("dest_directory", dest_directory))
    {
      RCLCPP_ERROR(this->get_logger(), "dest_directory param is not set.");
      rclcpp::shutdown();
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "input_pcd     :" << input_pcd << std::endl 
                                        << "dest_directory:" << dest_directory << std::endl
                                        << "resolution    :" << resolution << std::endl);

    
    cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if(pcl::io::loadPCDFile(input_pcd, *cloud))
    {
      RCLCPP_ERROR("failed to open the input cloud");
      rclcpp::shutdown();
    }
  }

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

  void run()
  {
    cv::Mat map = generater.generate(*cloud);

    if(!std::filesystem::exists(dest_directory))
    {
      std::filesystem::create_directories(dest_directory);
    }

    cv::imwrite(dest_directory + "/map.png", map);
    
    std::ofstream ofs(dest_directory + "/map.yaml");
    ofs << "image: map.png" << std::endl;
    ofs << "resolution: " << resolution << std::endl;
    ofs << "origin: [" << -resolution * map_width / 2 << ", " << -resolution * map_height / 2 << ", 0.0]" << std::endl;
    ofs << "occupied_thresh: 0.5" << std::endl;
    ofs << "free_thresh: 0.2" << std::endl;
    ofs << "negate: 0" << std::endl;

    RCLCPP_INFO("Finish converting point cloud to 2d map.");
  }

public:
  double resolution;    // meters per pixel
  double m2pix;         // inverse resolution (pix/m)
  int map_width;
  int map_height;

  int min_points_in_pix;
  int max_points_in_pix;
  double min_height;
  double max_height;

  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud;
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  MapGenerater generater();
  generater.run();

  rclcpp::shutdown();
  return 0;
}