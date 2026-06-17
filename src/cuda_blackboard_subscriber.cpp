
#include "cuda_blackboard/cuda_blackboard_subscriber.hpp"

#include "cuda_blackboard/cuda_blackboard.hpp"
#include "cuda_blackboard/cuda_error.hpp"
#include "cuda_blackboard/cuda_mem_pool_context.hpp"
#include "cuda_blackboard/negotiated_types.hpp"

#include <rclcpp/rclcpp.hpp>

#include <functional>

namespace cuda_blackboard
{

template <typename T>
CudaBlackboardSubscriber<T>::CudaBlackboardSubscriber(
  rclcpp::Node & node, const std::string & topic_name, bool,
  std::function<void(std::shared_ptr<const T>)> callback, cudaStream_t user_stream)
: CudaBlackboardSubscriber(node, topic_name, callback, user_stream)
{
}

template <typename T>
CudaBlackboardSubscriber<T>::CudaBlackboardSubscriber(
  rclcpp::Node & node, const std::string & topic_name,
  std::function<void(std::shared_ptr<const T>)> callback, cudaStream_t user_stream)
: node_(node), user_stream_(user_stream)
{
  using std::placeholders::_1;

  if (needFallbackToLegacyDefaultStream(user_stream_)) {
    RCLCPP_WARN_STREAM(
      node_.get_logger(),
      "`user_stream` was not set or the legacy default stream was specified for the "
      "CudaBlackboardSubscriber of "
        << topic_name
        << ". This causes process-wide synchronization after the callback executes, which may "
           "degrade performance. Moreover, if the subscribed data is consumed on a CUDA stream "
           "created with the `cudaStreamNonBlocking` flag, this may cause a use-after-free of the "
           "pointer. To avoid this, consider the following:"
        << std::endl
        << "  - pass the stream to the CudaBlackboardSubscriber's constructor, and/or " << std::endl
        << "  - perform proper synchronization on the stream in the callback");
  }

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
    CUDA_BLACKBOARD_CHECK_CUDA_ERROR(
      cudaStreamWaitEvent(user_stream_, data->ready_event_, cudaEventWaitDefault));

    callback_(data);

    // Make the free stream wait until the consumer is done with `data` before it is freed.
    addConsumerDependencyToFreeStream();
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
  auto cuda_msg_ptr = std::make_shared<T>(*ros_msg_ptr);
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(
    cudaStreamWaitEvent(user_stream_, cuda_msg_ptr->ready_event_, cudaEventWaitDefault));

  RCLCPP_WARN_ONCE(
    node_.get_logger(),
    "The compatible callback was called. This results in a performance loss. This behavior is "
    "probably not intended or a temporal measure");
  callback_(cuda_msg_ptr);

  // Make the free stream wait until the consumer is done with the message before it is freed.
  addConsumerDependencyToFreeStream();
}

template <typename T>
void CudaBlackboardSubscriber<T>::addConsumerDependencyToFreeStream() const
{
  const cudaStream_t record_stream =
    needFallbackToLegacyDefaultStream(user_stream_) ? cudaStreamLegacy : user_stream_;
  auto & ctx = CudaMemPoolContext::getInstance();
  cudaEvent_t event;
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaEventCreateWithFlags(&event, cudaEventDisableTiming));
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaEventRecord(event, record_stream));
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(
    cudaStreamWaitEvent(ctx.free_stream(), event, cudaEventWaitDefault));
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaEventDestroy(event));
}

template <typename T>
inline bool CudaBlackboardSubscriber<T>::needFallbackToLegacyDefaultStream(
  const cudaStream_t & stream) const

{
  return (stream == nullptr) || (stream == cudaStreamLegacy);
}

}  // namespace cuda_blackboard

template class cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>;
template class cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaImage>;
