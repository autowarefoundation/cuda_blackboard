

#pragma once

#include <negotiated/negotiated_subscription.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/u_int64.hpp>

#include <cuda_runtime_api.h>

#include <memory>

namespace cuda_blackboard
{

template <typename T>
class CudaBlackboardSubscriber
{
public:
  [[deprecated]] CudaBlackboardSubscriber(
    rclcpp::Node & node, const std::string & topic_name, bool add_compatible_sub,
    std::function<void(std::shared_ptr<const T>)> callback, cudaStream_t user_stream);

  [[deprecated(
    "Use the constructor that takes cudaStream_t to avoid legacy-default-stream synchronization, "
    "which can degrade performance.")]] CudaBlackboardSubscriber(rclcpp::Node & node, const std::string & topic_name, bool add_compatible_sub, std::function<void(std::shared_ptr<const T>)> callback)
  : CudaBlackboardSubscriber(node, topic_name, add_compatible_sub, callback, cudaStreamLegacy) {};

  CudaBlackboardSubscriber(
    rclcpp::Node & node, const std::string & topic_name,
    std::function<void(std::shared_ptr<const T>)> callback, cudaStream_t user_stream);

  [[deprecated(
    "Use the constructor that takes cudaStream_t to avoid legacy-default-stream synchronization, "
    "which can degrade performance.")]] CudaBlackboardSubscriber(rclcpp::Node & node, const std::string & topic_name, std::function<void(std::shared_ptr<const T>)> callback)
  : CudaBlackboardSubscriber(node, topic_name, callback, cudaStreamLegacy) {};

private:
  void instanceIdCallback(const std_msgs::msg::UInt64 & instance_id_msg);

  void compatibleCallback(const std::shared_ptr<const typename T::ros_type> & ros_msg_ptr);

  inline bool needFallbackToLegacyDefaultStream(const cudaStream_t & stream) const;

  void addConsumerDependencyToFreeStream() const;

  std::function<void(std::shared_ptr<const T> cuda_msg)> callback_{};

  rclcpp::Node & node_;
  std::shared_ptr<negotiated::NegotiatedSubscription> negotiated_sub_;
  typename rclcpp::Subscription<typename T::ros_type>::SharedPtr compatible_sub_;
  cudaStream_t user_stream_;
};

}  // namespace cuda_blackboard
