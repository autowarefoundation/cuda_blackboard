

#pragma once

#include <negotiated/negotiated_subscription.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/u_int64.hpp>

#include <memory>

namespace cuda_blackboard
{

template <typename T>
class CudaBlackboardSubscriber
{
public:
  CudaBlackboardSubscriber(
    rclcpp::Node & node, const std::string & topic_name, bool add_compatible_sub,
    std::function<void(std::shared_ptr<const T>)> callback);

private:
  void instanceIdCallback(const std_msgs::msg::UInt64 & instance_id_msg);

  void compatibleCallback(const std::shared_ptr<const typename T::ros_type> & ros_msg_ptr);

  std::function<void(std::shared_ptr<const T> cuda_msg)> callback_{};

  rclcpp::Node & node_;
  std::shared_ptr<negotiated::NegotiatedSubscription> negotiated_sub_;
  typename rclcpp::Subscription<typename T::ros_type>::SharedPtr compatible_sub_;
};

}  // namespace cuda_blackboard
