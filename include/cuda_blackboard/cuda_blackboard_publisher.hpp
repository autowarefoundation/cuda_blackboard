
#pragma once

#include "cuda_blackboard/negotiated_types.hpp"

#include <negotiated/negotiated_publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cuda_runtime_api.h>

#include <memory>
#include <string>
#include <utility>

namespace cuda_blackboard
{

template <typename T>
class CudaBlackboardPublisher
{
public:
  CudaBlackboardPublisher(rclcpp::Node & node, const std::string & topic_name);
  void publish(std::unique_ptr<const T> msg);

  // Stream-ordered publish: `stream` is the stream the message buffer was
  // allocated and filled on. The buffer is not host-synchronized before
  // publishing; consumers using the stream-aware subscriber are ordered after
  // the fill via CUDA events. The buffer must outlive every subscriber's use,
  // which is guaranteed because the stream (owned by the producing node)
  // outlives the messages it produces.
  void publish(std::unique_ptr<const T> msg, cudaStream_t stream);

  std::size_t get_subscription_count() const;
  std::size_t get_intra_process_subscription_count() const;

private:
  // Shared implementation. `stream` is nullptr for the legacy synchronous path.
  void publishImpl(std::unique_ptr<const T> msg, cudaStream_t stream);

  rclcpp::Node & node_;
  std::shared_ptr<negotiated::NegotiatedPublisher> negotiated_pub_;
  typename rclcpp::Publisher<typename T::ros_type>::SharedPtr compatible_pub_;
};

}  // namespace cuda_blackboard
