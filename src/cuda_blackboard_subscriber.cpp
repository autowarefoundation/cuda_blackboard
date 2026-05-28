
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
  callback_ = callback;
  initialize(topic_name);
}

template <typename T>
CudaBlackboardSubscriber<T>::CudaBlackboardSubscriber(
  rclcpp::Node & node, const std::string & topic_name,
  std::function<void(std::shared_ptr<const T>)> callback)
: node_(node)
{
  callback_ = callback;
  initialize(topic_name);
}

template <typename T>
CudaBlackboardSubscriber<T>::CudaBlackboardSubscriber(
  rclcpp::Node & node, const std::string & topic_name,
  std::function<void(std::shared_ptr<const T>)> callback, cudaStream_t stream)
: node_(node), stream_(stream)
{
  callback_ = callback;
  initialize(topic_name);
}

template <typename T>
void CudaBlackboardSubscriber<T>::initialize(const std::string & topic_name)
{
  using std::placeholders::_1;

  negotiated::NegotiatedSubscriptionOptions negotiation_options;
  negotiation_options.disconnect_on_negotiation_failure = false;

  negotiated_sub_ = std::make_shared<negotiated::NegotiatedSubscription>(
    node_, topic_name + "/cuda", negotiation_options);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

  negotiated_sub_->add_supported_callback<NegotiationStruct<T>>(
    1.0, rclcpp::QoS(1), std::bind(&CudaBlackboardSubscriber<T>::instanceIdCallback, this, _1),
    sub_options);

  std::string ros_type_name = NegotiationStruct<typename T::ros_type>::supported_type_name;

  compatible_sub_ = node_.create_subscription<typename T::ros_type>(
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
  cudaEvent_t ready_event = nullptr;
  auto data = blackboard.queryData(instance_id_msg.data, &ready_event);
  if (data) {
    // Order this node's stream after the producer's fill (no host stall), then
    // run the user callback, which queues its reads on stream_.
    if (stream_ != nullptr && ready_event != nullptr) {
      CUDA_BLACKBOARD_CHECK_CUDA_ERROR(::cudaStreamWaitEvent(stream_, ready_event, 0));
    }

    callback_(data);

    // Free the buffer on this subscriber's stream: point its deleter at stream_
    // so the eventual cudaFreeAsync is queued after the reads above (same
    // stream), with no host synchronization. set_stream() is a no-op for a
    // synchronously allocated buffer. The free itself happens when the last
    // reference to the message is dropped.
    if (stream_ != nullptr) {
      data->data.get_deleter().set_stream(stream_);
    }
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
  // Construct synchronously even on the stream-aware path: the source ROS
  // message buffer is owned by the subscription and released right after this
  // callback returns, so an async host-to-device copy could outlive it. This is
  // the rare degraded fallback (negotiation not yet established), so the extra
  // synchronization is acceptable.
  callback_(std::make_shared<T>(*ros_msg_ptr));
}

}  // namespace cuda_blackboard

template class cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>;
template class cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaImage>;
