
#pragma once

#include "cuda_blackboard/cuda_unique_ptr.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstdint>
#include <memory>

namespace cuda_blackboard
{

class CudaPointCloud2 : public sensor_msgs::msg::PointCloud2
{
public:
  using ros_type = sensor_msgs::msg::PointCloud2;
  using SharedPtr = std::shared_ptr<CudaPointCloud2>;
  using ConstSharedPtr = std::shared_ptr<CudaPointCloud2 const>;

  template <typename Deleter = std::default_delete<CudaPointCloud2>>
  using UniquePtrWithDeleter = std::unique_ptr<CudaPointCloud2, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template <typename Deleter = std::default_delete<CudaPointCloud2>>
  using ConstUniquePtrWithDeleter = std::unique_ptr<CudaPointCloud2 const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  CudaPointCloud2();
  CudaPointCloud2(const CudaPointCloud2 & pointcloud);  // This is needed for ROS compliance
  CudaPointCloud2 & operator=(const CudaPointCloud2 &) = delete;

  CudaPointCloud2(CudaPointCloud2 && pointcloud);
  CudaPointCloud2 & operator=(CudaPointCloud2 &&);
  CudaPointCloud2(const sensor_msgs::msg::PointCloud2 & source);

  ~CudaPointCloud2();

  CudaUniquePtr<std::uint8_t[]> data;
};

}  // namespace cuda_blackboard
