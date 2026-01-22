

#include "cuda_blackboard/cuda_pointcloud2.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cuda_runtime_api.h>

namespace cuda_blackboard
{

CudaPointCloud2::CudaPointCloud2()
{
}

CudaPointCloud2::CudaPointCloud2(CudaPointCloud2 && source)
{
  header = source.header;
  fields = source.fields;
  height = source.height;
  width = source.width;
  row_step = source.row_step;
  point_step = source.point_step;
  is_dense = source.is_dense;
  is_bigendian = source.is_bigendian;
  data = std::move(source.data);

  source.data = nullptr;
}

CudaPointCloud2 & CudaPointCloud2::operator=(CudaPointCloud2 && other)
{
  if (this != &other) {
    header = other.header;
    fields = other.fields;
    height = other.height;
    width = other.width;
    row_step = other.row_step;
    point_step = other.point_step;
    is_dense = other.is_dense;
    is_bigendian = other.is_bigendian;
    data = std::move(other.data);

    other.data = nullptr;
  }

  return *this;
}

CudaPointCloud2::CudaPointCloud2(const CudaPointCloud2 & pointcloud)
: sensor_msgs::msg::PointCloud2(pointcloud)
{
  RCLCPP_WARN(
    rclcpp::get_logger("CudaPointCloud2"),
    "CudaPointCloud2 copy constructor called. This should be avoided and is most likely a design "
    "error.");

  data = make_unique<uint8_t[]>(
    pointcloud.height * pointcloud.width * pointcloud.point_step * sizeof(uint8_t));
  cudaMemcpy(
    data.get(), pointcloud.data.get(),
    pointcloud.height * pointcloud.width * pointcloud.point_step * sizeof(uint8_t),
    cudaMemcpyDeviceToDevice);
}

CudaPointCloud2::CudaPointCloud2(const sensor_msgs::msg::PointCloud2 & source)
{
  header = source.header;
  fields = source.fields;
  height = source.height;
  width = source.width;
  row_step = source.row_step;
  point_step = source.point_step;
  is_dense = source.is_dense;
  is_bigendian = source.is_bigendian;

  data = make_unique<uint8_t[]>(source.height * source.width * source.point_step * sizeof(uint8_t));
  cudaMemcpy(
    data.get(), source.data.data(),
    source.height * source.width * source.point_step * sizeof(uint8_t), cudaMemcpyHostToDevice);
}

CudaPointCloud2::~CudaPointCloud2()
{
}

}  // namespace cuda_blackboard
