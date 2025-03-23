
#include "cuda_blackboard/cuda_blackboard_subscriber.hpp"

#include "cuda_blackboard/cuda_blackboard.hpp"
#include "cuda_blackboard/negotiated_types.hpp"

#include <functional>

namespace cuda_blackboard
{

template <typename T>
CudaBlackboardSubscriber<T>::CudaBlackboardSubscriber(
  rclcpp::Node & node, const std::string & topic_name, [[maybe_unused]] bool add_compatible_sub,
  std::function<void(std::shared_ptr<const T>)> callback)
: node_(node)
{
  using std::placeholders::_1;

  negotiated::NegotiatedSubscriptionOptions negotiation_options;
  negotiation_options.disconnect_on_negotiation_failure = false;

  callback_ = callback;
  negotiated_sub_ = std::make_shared<negotiated::NegotiatedSubscription>(
    node, topic_name + "/cuda", negotiation_options);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

  negotiated_sub_->add_supported_callback<NegotiationStruct<T>>(
    1.0, rclcpp::QoS(1), std::bind(&CudaBlackboardSubscriber<T>::instanceIdCallback, this, _1),
    sub_options);

  std::string ros_type_name = NegotiationStruct<typename T::ros_type>::supported_type_name;

  compatible_sub_ = node.create_subscription<typename T::ros_type>(
    topic_name, rclcpp::SensorDataQoS(),
    std::bind(&CudaBlackboardSubscriber<T>::compatibleCallback, this, _1), sub_options);

  negotiated_sub_->add_compatible_subscription(compatible_sub_, ros_type_name, 0.1);

  negotiated_sub_->start();
}

template <typename T>
CudaBlackboardSubscriber<T>::CudaBlackboardSubscriber(
  rclcpp::Node & node, const std::string & topic_name,
  std::function<void(std::shared_ptr<const T>)> callback)
: node_(node)
{
  using std::placeholders::_1;

  negotiated::NegotiatedSubscriptionOptions negotiation_options;
  negotiation_options.disconnect_on_negotiation_failure = false;

  callback_ = callback;
  negotiated_sub_ = std::make_shared<negotiated::NegotiatedSubscription>(
    node, topic_name + "/cuda", negotiation_options);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

  negotiated_sub_->add_supported_callback<NegotiationStruct<T>>(
    1.0, rclcpp::QoS(1), std::bind(&CudaBlackboardSubscriber<T>::instanceIdCallback, this, _1),
    sub_options);

  std::string ros_type_name = NegotiationStruct<typename T::ros_type>::supported_type_name;

  compatible_sub_ = node.create_subscription<typename T::ros_type>(
    topic_name, rclcpp::SensorDataQoS(),
    std::bind(&CudaBlackboardSubscriber<T>::compatibleCallback, this, _1), sub_options);

  negotiated_sub_->add_compatible_subscription(compatible_sub_, ros_type_name, 0.1);

  negotiated_sub_->start();
}

template <typename T>
void CudaBlackboardSubscriber<T>::instanceIdCallback(const std_msgs::msg::UInt64 & instance_id_msg)
{
  if (compatible_sub_ && negotiated_sub_->get_negotiated_topic_publisher_count() > 0) {
    const std::string ros_type_name = NegotiationStruct<typename T::ros_type>::supported_type_name;
    negotiated_sub_->remove_compatible_subscription<typename T::ros_type>(
      compatible_sub_, ros_type_name);
    compatible_sub_ = nullptr;

    RCLCPP_INFO(
      node_.get_logger(),
      "A negotiated message has been received, so the compatible callback will be disabled");
  }

  auto & blackboard = CudaBlackboard<T>::getInstance();
  auto data = blackboard.queryData(instance_id_msg.data);
  if (data) {
    callback_(data);
  } else {
    RCLCPP_ERROR_STREAM(
      node_.get_logger(), "There was not data with the requested instance id= "
                            << instance_id_msg.data << " in the blackboard.");
  }
}

template <typename T>
void CudaBlackboardSubscriber<T>::compatibleCallback(
  const std::shared_ptr<const typename T::ros_type> & ros_msg_ptr)
{
  const std::string ros_type_name = NegotiationStruct<typename T::ros_type>::supported_type_name;

  if (compatible_sub_ && negotiated_sub_->get_negotiated_topic_publisher_count() > 0) {
    negotiated_sub_->remove_compatible_subscription<typename T::ros_type>(
      compatible_sub_, ros_type_name);
    compatible_sub_ = nullptr;

    RCLCPP_INFO(
      node_.get_logger(),
      "A negotiation type succeeded, so the compatible callback will be disabled");

    return;
  }

  RCLCPP_WARN_ONCE(
    node_.get_logger(),
    "The compatible callback was called. This results in a performance loss. This behavior is "
    "probably not intended or a temporal measure");
  callback_(std::make_shared<T>(*ros_msg_ptr));
}

}  // namespace cuda_blackboard

template class cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>;
template class cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaImage>;
