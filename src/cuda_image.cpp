
#include "cuda_blackboard/cuda_image.hpp"

#include "cuda_blackboard/cuda_error.hpp"
#include "cuda_blackboard/cuda_mem_pool_context.hpp"
#include "cuda_message_impl_detail.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cuda_runtime_api.h>

namespace cuda_blackboard
{

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

  impl_detail::copyDataToDevice(
    data, ready_event_, image.data.get(), image.height * image.step * sizeof(uint8_t),
    cudaMemcpyDeviceToDevice);
}

CudaImage::CudaImage(const sensor_msgs::msg::Image & source)
{
  header = source.header;
  encoding = source.encoding;
  height = source.height;
  width = source.width;
  step = source.step;
  is_bigendian = source.is_bigendian;

  impl_detail::copyDataToDevice(
    data, ready_event_, source.data.data(), source.height * source.step * sizeof(uint8_t),
    cudaMemcpyHostToDevice);
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
