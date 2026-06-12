

#include "cuda_blackboard/cuda_adaptation.hpp"
#include "cuda_blackboard/cuda_blackboard_subscriber.hpp"
#include "cuda_blackboard/cuda_error.hpp"
#include "cuda_blackboard/cuda_image.hpp"
#include "cuda_blackboard/cuda_mem_pool_context.hpp"
#include "cuda_blackboard/cuda_unique_ptr.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <nvtx3/nvToolsExt.h>

#include <memory>
#include <string>
#include <utility>

namespace
{
__global__ void dummy_copy_kernel(
  uint8_t * __restrict__ src, uint8_t * __restrict__ dest, const size_t width, const size_t height,
  const size_t step)
{
  auto x = blockIdx.x * blockDim.x + threadIdx.x;
  auto y = blockIdx.y * blockDim.y + threadIdx.y;
  if (width <= x || height <= y) {
    return;
  }

  dest[y * step + x] = src[y * step + x];
}
}  // namespace

namespace cuda_blackboard
{

class CudaBlackboardSubscriberNodeWithStream final : public rclcpp::Node
{
public:
  explicit CudaBlackboardSubscriberNodeWithStream(const rclcpp::NodeOptions & options)
  : rclcpp::Node("cuda_blackboard_subscriber_node", options)
  {
    CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));

    auto callback = [this](std::shared_ptr<const CudaImage> cuda_msg) {
      nvtxRangePushA("CudaBlackboardSubscriberNodeWithStream::callback");
      RCLCPP_INFO(
        this->get_logger(), "Received message with resolution: %u x %u", cuda_msg->height,
        cuda_msg->width);
      if (!dummy_image_dev_) {
        dummy_image_dev_ =
          cuda_blackboard::make_unique<uint8_t[]>(cuda_msg->height * cuda_msg->step);
      }

      dim3 thread_per_block(
        8, 8);  // intentionally use small block to make the kernel take long time execution
      dim3 num_blocks(
        (cuda_msg->width + thread_per_block.x - 1) / thread_per_block.x,
        (cuda_msg->height + thread_per_block.y - 1) / thread_per_block.y);
      dummy_copy_kernel<<<num_blocks, thread_per_block, 0, stream_>>>(
        cuda_msg->data.get(), dummy_image_dev_.get(), cuda_msg->width, cuda_msg->height,
        cuda_msg->step);

      nvtxRangePop();
      // cudaStreamSynchronize(stream_) can be omit since we passed stream_ to the
      // CudaBlackboardSubscriber
    };

    // Pass user side stream to the CudaBlackboardSubscriber ctor
    sub_ = std::make_shared<CudaBlackboardSubscriber<CudaImage>>(
      *this, "image_raw", false, callback, stream_);
  }

private:
  std::shared_ptr<CudaBlackboardSubscriber<CudaImage>> sub_;
  CudaUniquePtr<uint8_t[]> dummy_image_dev_{nullptr};
  cudaStream_t stream_{nullptr};
};

}  // namespace cuda_blackboard

RCLCPP_COMPONENTS_REGISTER_NODE(cuda_blackboard::CudaBlackboardSubscriberNodeWithStream)
