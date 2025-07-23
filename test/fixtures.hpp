#pragma once

#include "cuda_blackboard/cuda_image.hpp"
#include "cuda_blackboard/cuda_pointcloud2.hpp"

#include <cv_bridge/cv_bridge.h>

namespace cuda_blackboard
{

inline std::unique_ptr<CudaPointCloud2> create_test_point_cloud()
{
  sensor_msgs::msg::PointCloud2 ros_pc;
  ros_pc.header.stamp.sec = 123;
  ros_pc.header.stamp.nanosec = 456789;
  ros_pc.header.frame_id = "lidar_frame";
  ros_pc.width = 10;
  ros_pc.height = 1;
  ros_pc.is_bigendian = false;
  ros_pc.is_dense = true;
  ros_pc.point_step = 3;  // Assuming 3 bytes per point (x, y, z)
  ros_pc.row_step = ros_pc.point_step * ros_pc.width;
  ros_pc.data.resize(ros_pc.row_step * ros_pc.height);

  std::fill(ros_pc.data.begin(), ros_pc.data.end(), 0x42);

  return std::make_unique<CudaPointCloud2>(ros_pc);
}

inline std::unique_ptr<CudaImage> create_test_image()
{
  cv_bridge::CvImage cv_image;
  cv_image.header.stamp.sec = 123;
  cv_image.header.stamp.nanosec = 456789;
  cv_image.header.frame_id = "camera_frame";
  cv_image.encoding = sensor_msgs::image_encodings::BGR8;

  cv::Scalar blue(255, 0, 0);
  cv_image.image = cv::Mat(20, 10, CV_8UC3, blue);

  sensor_msgs::msg::Image ros_image;
  cv_image.toImageMsg(ros_image);

  return std::make_unique<CudaImage>(ros_image);
}

}  // namespace cuda_blackboard