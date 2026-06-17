
#include "cuda_blackboard/cuda_image.hpp"

#include "cuda_blackboard/cuda_error.hpp"
#include "cuda_blackboard/cuda_mem_pool_context.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cuda_runtime_api.h>

#include <memory>

namespace cuda_blackboard
{
namespace
{

template <typename T>
const void * getDataPointer(const CudaUniquePtr<T[]> & data)
{
  return data.get();
}

template <typename T, typename Allocator>
const void * getDataPointer(const std::vector<T, Allocator> & data)
{
  return data.data();
}

template <typename T>
void copyDataToDevice(CudaImage & image, const T & source, const cudaMemcpyKind copy_kind)
{
  auto & ctx = CudaMemPoolContext::getInstance();

  // NOTE: `cudaEventBlockingSync` flag is not set here so that cudaEventSynchronize()
  // will busy-wait until the event has been completed
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(
    cudaEventCreateWithFlags(&image.ready_event(), cudaEventDisableTiming));

  auto size = source.height * source.step * sizeof(uint8_t);
  image.data = make_unique<uint8_t[]>(size);
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(
    cudaMemcpyAsync(image.data.get(), getDataPointer(source.data), size, copy_kind, ctx.stream()));

  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaEventRecord(image.ready_event(), ctx.stream()));
}

}  // namespace

CudaImage::CudaImage()
{
  // NOTE: `cudaEventBlockingSync` flag is not set here so that cudaEventSynchronize()
  // will busy-wait until the event has been completed
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaEventCreateWithFlags(&ready_event_, cudaEventDisableTiming));
}

CudaImage::CudaImage(const CudaImage & image) : sensor_msgs::msg::Image(image)
{
  RCLCPP_WARN(
    rclcpp::get_logger("CudaImage"),
    "CudaImage copy constructor called. This should be avoided and is most likely a design error.");

  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaEventCreateWithFlags(&ready_event_, cudaEventDisableTiming));
  copyDataToDevice(*this, image, cudaMemcpyDeviceToDevice);
}

CudaImage::CudaImage(const sensor_msgs::msg::Image & source)
{
  header = source.header;
  encoding = source.encoding;
  height = source.height;
  width = source.width;
  step = source.step;
  is_bigendian = source.is_bigendian;

  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaEventCreateWithFlags(&ready_event_, cudaEventDisableTiming));
  copyDataToDevice(*this, source, cudaMemcpyHostToDevice);
}

CudaImage::~CudaImage()
{
  if (ready_event_) {
    auto & ctx = CudaMemPoolContext::getInstance();
    cudaStreamWaitEvent(ctx.free_stream(), ready_event_, cudaEventWaitDefault);
    cudaEventDestroy(ready_event_);
  }
}

}  // namespace cuda_blackboard
