
#include "cuda_blackboard/cuda_image.hpp"

#include "cuda_blackboard/cuda_error.hpp"
#include "cuda_blackboard/cuda_mem_pool_context.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cuda_runtime_api.h>

#include <memory>

namespace cuda_blackboard
{
CudaImage::CudaImage()
{
  // NOTE: `cudaEventBlockingSync` flag is not set here so that cudaEventSynchronize()
  // will busy-wait until the event has been completed
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(
    cudaEventCreateWithFlags(&ready_event(), cudaEventDisableTiming));
}

CudaImage::CudaImage(const CudaImage & image) : sensor_msgs::msg::Image(image)
{
  RCLCPP_WARN(
    rclcpp::get_logger("CudaImage"),
    "CudaImage copy constructor called. This should be avoided and is most likely a design error.");

  auto & ctx = CudaMemPoolContext::getInstance();

  // NOTE: `cudaEventBlockingSync` flag is not set here so that cudaEventSynchronize()
  // will busy-wait until the event has been completed
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(
    cudaEventCreateWithFlags(&ready_event(), cudaEventDisableTiming));

  data = make_unique<uint8_t[]>(image.height * image.step * sizeof(uint8_t));
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaMemcpyAsync(
    data.get(), image.data.get(), image.height * image.step * sizeof(uint8_t),
    cudaMemcpyDeviceToDevice, ctx.stream()));

  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaEventRecord(ready_event(), ctx.stream()));
}

CudaImage::CudaImage(const sensor_msgs::msg::Image & source)
{
  header = source.header;
  encoding = source.encoding;
  height = source.height;
  width = source.width;
  step = source.step;
  is_bigendian = source.is_bigendian;

  auto & ctx = CudaMemPoolContext::getInstance();

  // NOTE: `cudaEventBlockingSync` flag is not set here so that cudaEventSynchronize()
  // will busy-wait until the event has been completed
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(
    cudaEventCreateWithFlags(&ready_event(), cudaEventDisableTiming));

  data = make_unique<uint8_t[]>(source.height * source.step * sizeof(uint8_t));
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaMemcpyAsync(
    data.get(), source.data.data(), source.height * source.step * sizeof(uint8_t),
    cudaMemcpyHostToDevice, ctx.stream()));

  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaEventRecord(ready_event(), ctx.stream()));
}

CudaImage::~CudaImage()
{
  if (ready_event_) {
    cudaEventSynchronize(ready_event());
    cudaEventDestroy(ready_event());
  }
}

}  // namespace cuda_blackboard
