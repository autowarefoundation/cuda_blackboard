
#pragma once

#include "cuda_blackboard/negotiated_types.hpp"

#include <negotiated/negotiated_publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace cuda_blackboard
{

template <typename T>
class CudaBlackboardPublisher
{
public:
  CudaBlackboardPublisher(rclcpp::Node & node, const std::string & topic_name);
  void publish(std::unique_ptr<const T> msg);

  std::size_t get_subscription_count() const;
  std::size_t get_intra_process_subscription_count() const;

private:
  // Holds the Agnocast publisher of the ROS message when the node runs on Agnocast; defined in the
  // source file so that this header is the same however the code including it was built.
  struct AgnocastCompatiblePublisher;

  void publish_compatible(const T & cuda_msg);
  // publish() and the subscription counts may run on any thread: they read the negotiation state
  // from a cache refreshed on the executor thread.
  void refreshNegotiationCache();

  rclcpp::Node & node_;

  struct NegotiatedCache
  {
    /// A subscriber negotiated the plain ROS message type.
    bool negotiated_ros{false};
    /// A subscriber negotiated the CUDA type, delivered through the blackboard.
    bool negotiated_cuda{false};
    /// Publisher of the blackboard instance ids.
    std::shared_ptr<rclcpp::PublisherBase> blackboard_pub;
    /// Every live publisher, negotiated and compatible.
    std::vector<std::shared_ptr<rclcpp::PublisherBase>> negotiated_pubs;
  };

  // Both declared before negotiated_pub_: its timer may still call refreshNegotiationCache() while
  // it is being destroyed.
  mutable std::mutex cache_mutex_;
  NegotiatedCache negotiated_cache_;

  std::shared_ptr<negotiated::NegotiatedPublisher> negotiated_pub_;
  typename rclcpp::Publisher<typename T::ros_type>::SharedPtr compatible_pub_;
  std::shared_ptr<AgnocastCompatiblePublisher> agnocast_compatible_pub_;
};

}  // namespace cuda_blackboard
