

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

  /**
   * @brief Check whether the given CUDA stream requires fallback to the legacy default stream.
   *
   * @return True if the stream is nullptr or the legacy default stream.
   */
  inline bool needFallbackToLegacyDefaultStream(const cudaStream_t & stream) const;

  /**
   * @brief Make the CUDA memory-pool free stream held by CudaMemPoolContext wait for the
   * callback's consumer stream.
   *
   * @details Records an event on the consumer stream, or on the legacy default stream as a
   * fallback, and makes CudaMemPoolContext::free_stream() wait on it. Call this right after the
   * callback returns, while the message is still alive, so the cudaFreeAsync issued by CudaDeleter
   * on the same free stream is ordered strictly after consumption. See
   * CudaMemPoolContext::free_stream() for why the two streams must match.
   *
   * @note The legacy-default-stream fallback does not work when all of the following hold: the data
   * is consumed in the callback on a stream created with the cudaStreamNonBlocking flag, no proper
   * stream synchronization is performed in the callback, and that stream is not passed as the
   * user_stream argument of the constructor. Such a non-blocking stream does not synchronize with
   * the legacy default stream, so the event recorded here cannot capture the consumption and the
   * memory may be freed too early.
   */
  void addConsumerDependencyToFreeStream() const;

  std::function<void(std::shared_ptr<const T> cuda_msg)> callback_{};

  rclcpp::Node & node_;
  std::shared_ptr<negotiated::NegotiatedSubscription> negotiated_sub_;
  typename rclcpp::Subscription<typename T::ros_type>::SharedPtr compatible_sub_;
  cudaStream_t user_stream_;
};

}  // namespace cuda_blackboard
