

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
    std::function<void(std::shared_ptr<const T>)> callback);

  CudaBlackboardSubscriber(
    rclcpp::Node & node, const std::string & topic_name,
    std::function<void(std::shared_ptr<const T>)> callback);

  // Stream-aware subscriber: `stream` is the stream this node uses to read the
  // received buffers. Before the user callback runs, the stream is ordered
  // after the producer's fill; after it returns, a dependency is registered so
  // the buffer's async free waits for this stream's queued reads. The stream
  // must outlive this subscriber and every message delivered through it.
  CudaBlackboardSubscriber(
    rclcpp::Node & node, const std::string & topic_name,
    std::function<void(std::shared_ptr<const T>)> callback, cudaStream_t stream);

private:
  void initialize(const std::string & topic_name);

  void instanceIdCallback(const std_msgs::msg::UInt64 & instance_id_msg);

  void compatibleCallback(const std::shared_ptr<const typename T::ros_type> & ros_msg_ptr);

  std::function<void(std::shared_ptr<const T> cuda_msg)> callback_{};

  rclcpp::Node & node_;
  cudaStream_t stream_{nullptr};
  std::shared_ptr<negotiated::NegotiatedSubscription> negotiated_sub_;
  typename rclcpp::Subscription<typename T::ros_type>::SharedPtr compatible_sub_;
};

}  // namespace cuda_blackboard
