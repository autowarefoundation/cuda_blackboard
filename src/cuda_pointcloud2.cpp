

#include "cuda_blackboard/cuda_pointcloud2.hpp"

#include "cuda_blackboard/cuda_error.hpp"
#include "cuda_blackboard/cuda_mem_pool_context.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cuda_runtime_api.h>

namespace cuda_blackboard
{

CudaPointCloud2::CudaPointCloud2()
{
  // NOTE: `cudaEventBlockingSync` flag is not set here so that cudaEventSynchronize()
  // will busy-wait until the event has been completed
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(
    cudaEventCreateWithFlags(&ready_event(), cudaEventDisableTiming));
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
  ready_event_ = std::move(source.ready_event_);

  source.data = nullptr;
  source.ready_event_ = nullptr;
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
    ready_event_ = std::move(other.ready_event_);

    other.data = nullptr;
    other.ready_event_ = nullptr;
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

  auto & ctx = CudaMemPoolContext::getInstance();

  // NOTE: `cudaEventBlockingSync` flag is not set here so that cudaEventSynchronize()
  // will busy-wait until the event has been completed
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(
    cudaEventCreateWithFlags(&ready_event(), cudaEventDisableTiming));

  data = make_unique<uint8_t[]>(
    pointcloud.height * pointcloud.width * pointcloud.point_step * sizeof(uint8_t));
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaMemcpyAsync(
    data.get(), pointcloud.data.get(),
    pointcloud.height * pointcloud.width * pointcloud.point_step * sizeof(uint8_t),
    cudaMemcpyDeviceToDevice, ctx.stream()));

  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaEventRecord(ready_event(), ctx.stream()));
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

  auto & ctx = CudaMemPoolContext::getInstance();

  // NOTE: `cudaEventBlockingSync` flag is not set here so that cudaEventSynchronize()
  // will busy-wait until the event has been completed
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(
    cudaEventCreateWithFlags(&ready_event(), cudaEventDisableTiming));

  data = make_unique<uint8_t[]>(source.height * source.width * source.point_step * sizeof(uint8_t));
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaMemcpyAsync(
    data.get(), source.data.data(),
    source.height * source.width * source.point_step * sizeof(uint8_t), cudaMemcpyHostToDevice,
    ctx.stream()));

  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaEventRecord(ready_event(), ctx.stream()));
}

CudaPointCloud2::~CudaPointCloud2()
{
  if (ready_event_) {
    cudaEventSynchronize(ready_event());
    cudaEventDestroy(ready_event());
  }
}

}  // namespace cuda_blackboard
