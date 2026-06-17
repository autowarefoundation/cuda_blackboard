

#include "cuda_blackboard/cuda_adaptation.hpp"
#include "cuda_blackboard/cuda_blackboard_subscriber.hpp"
#include "cuda_blackboard/cuda_error.hpp"
#include "cuda_blackboard/cuda_image.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <utility>

namespace cuda_blackboard
{

class CudaBlackboardSubscriberNode final : public rclcpp::Node
{
public:
  explicit CudaBlackboardSubscriberNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("cuda_blackboard_subscriber_node", options)
  {
    CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));

    auto callback = [this](std::shared_ptr<const CudaImage> cuda_msg) {
      RCLCPP_INFO(
        this->get_logger(), "Received message with resolution: %u x %u", cuda_msg->height,
        cuda_msg->width);

      auto ros_image = std::make_unique<sensor_msgs::msg::Image>();
      ros_image->encoding = cuda_msg->encoding;
      ros_image->height = cuda_msg->height;
      ros_image->width = cuda_msg->width;
      ros_image->step = cuda_msg->step;
      ros_image->is_bigendian = cuda_msg->is_bigendian;
      ros_image->data.resize(cuda_msg->height * ros_image->step);
      cudaMemcpy(
        ros_image->data.data(), cuda_msg->data.get(), ros_image->height * ros_image->step,
        cudaMemcpyDeviceToHost);

      pub_->publish(std::move(ros_image));
    };

    sub_ = std::make_shared<CudaBlackboardSubscriber<CudaImage>>(
      *this, "image_raw", false, callback, stream_);

    pub_ = this->create_publisher<sensor_msgs::msg::Image>("output_image", 10);
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  std::shared_ptr<CudaBlackboardSubscriber<CudaImage>> sub_;
  cudaStream_t stream_{nullptr};
};

}  // namespace cuda_blackboard

RCLCPP_COMPONENTS_REGISTER_NODE(cuda_blackboard::CudaBlackboardSubscriberNode)
