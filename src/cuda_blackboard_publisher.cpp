
#include "cuda_blackboard/cuda_blackboard_publisher.hpp"

#include "cuda_blackboard/cuda_adaptation.hpp"
#include "cuda_blackboard/cuda_blackboard.hpp"
#include "cuda_blackboard/negotiated_types.hpp"

#ifdef USE_AGNOCAST_ENABLED
#include <autoware/agnocast_wrapper/publisher.hpp>
#include <autoware/agnocast_wrapper/runtime.hpp>
#endif

namespace cuda_blackboard
{

template <typename T>
struct CudaBlackboardPublisher<T>::AgnocastCompatiblePublisher
{
#ifdef USE_AGNOCAST_ENABLED
  typename autoware::agnocast_wrapper::Publisher<typename T::ros_type>::SharedPtr publisher;
#endif
};

template <typename T>
CudaBlackboardPublisher<T>::CudaBlackboardPublisher(
  rclcpp::Node & node, const std::string & topic_name)
: node_(node)
{
  negotiated::NegotiatedPublisherOptions negotiation_options;
  negotiation_options.disconnect_publishers_on_failure = false;
  negotiation_options.successful_negotiation_cb =
    [this](const negotiated_interfaces::msg::NegotiatedTopicsInfo &) { refreshNegotiationCache(); };

  negotiated_pub_ = std::make_shared<negotiated::NegotiatedPublisher>(
    node, topic_name + "/cuda", negotiation_options);
  rclcpp::PublisherOptions pub_options;
  pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

  negotiated_pub_->add_supported_type<NegotiationStruct<T>>(
    1.0,
    rclcpp::QoS(1),  //.durability_volatile(),
    pub_options);

#ifdef USE_AGNOCAST_ENABLED
  // On Agnocast, publish the ROS message through Agnocast so that subscribers in other Agnocast
  // processes do not need a ROS 2 -> Agnocast bridge for it.
  if (autoware::agnocast_wrapper::use_agnocast()) {
    agnocast_compatible_pub_ = std::make_shared<AgnocastCompatiblePublisher>();
    agnocast_compatible_pub_->publisher =
      autoware::agnocast_wrapper::create_publisher<typename T::ros_type>(
        &node, topic_name, rclcpp::SensorDataQoS());
    negotiated_pub_->start();
    refreshNegotiationCache();
    return;
  }
#endif

  std::string ros_type_name = NegotiationStruct<typename T::ros_type>::supported_type_name;
  compatible_pub_ =
    node.create_publisher<typename T::ros_type>(topic_name, rclcpp::SensorDataQoS(), pub_options);
  negotiated_pub_->add_compatible_publisher(compatible_pub_, ros_type_name, 0.1);

  negotiated_pub_->start();
  // the callback runs after every successful negotiation; without any NegotiatedSubscription
  // negotiation never runs, so seed the cache with the compatible publisher here
  refreshNegotiationCache();
}

template <typename T>
void CudaBlackboardPublisher<T>::refreshNegotiationCache()
{
  using ROSMessageType = typename NegotiationStruct<T>::MsgT;
  const std::string key_name = negotiated::detail::generate_key(
    rosidl_generator_traits::name<ROSMessageType>(), NegotiationStruct<T>::supported_type_name);
  const auto & map = negotiated_pub_->get_supported_types();
  NegotiatedCache cache;
  cache.negotiated_ros =
    negotiated_pub_->type_was_negotiated<NegotiationStruct<typename T::ros_type>>();
  cache.negotiated_cuda = negotiated_pub_->type_was_negotiated<NegotiationStruct<T>>();
  cache.blackboard_pub = map.count(key_name) != 0 ? map.at(key_name).publisher : nullptr;
  for (const auto & [key, info] : map) {
    if (info.publisher) {
      cache.negotiated_pubs.push_back(info.publisher);
    }
  }

  std::lock_guard<std::mutex> lock(cache_mutex_);
  negotiated_cache_ = std::move(cache);
}

template <typename T>
void CudaBlackboardPublisher<T>::publish(std::unique_ptr<const T> cuda_msg_ptr)
{
  using ROSMessageType = typename NegotiationStruct<T>::MsgT;

  bool negotiated_ros{false};
  bool negotiated_cuda{false};
  std::shared_ptr<rclcpp::PublisherBase> publisher;
  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    negotiated_ros = negotiated_cache_.negotiated_ros;
    negotiated_cuda = negotiated_cache_.negotiated_cuda;
    publisher = negotiated_cache_.blackboard_pub;
  }

  uint64_t instance_id{0};
  bool publish_ros_msg = false;
#ifdef USE_AGNOCAST_ENABLED
  if (agnocast_compatible_pub_) {
    publish_ros_msg =
      agnocast_compatible_pub_->publisher->get_subscription_count() > 0 ||
      agnocast_compatible_pub_->publisher->get_intra_process_subscription_count() > 0;
  } else
#endif
  {
    publish_ros_msg =
      negotiated_ros && (compatible_pub_->get_subscription_count() > 0 ||
                         compatible_pub_->get_intra_process_subscription_count() > 0);
  }

  // tickets are only given to intra process subscribers
  std::size_t tickets =
    publisher != nullptr ? publisher->get_intra_process_subscription_count() : 0;

  bool publish_blackboard_msg = negotiated_cuda && tickets > 0;

  auto & blackboard = CudaBlackboard<T>::getInstance();

  // When we want to publish cuda data, we instead use the blackboard
  if (publish_blackboard_msg) {
    if (publish_ros_msg) {
      tickets++;
    }

    // TODO: Once CudaBlackboardPublisher accepts a `user_stream`, record the buffer's
    // ready_event on it here (before the std::move below), e.g.
    // `cudaEventRecord(cuda_msg_ptr->ready_event_, user_stream_)`.
    // This is the producer-side mirror of the consumer ordering in CudaBlackboardSubscriber: it
    // lets a subscriber wait on ready_event (via cudaStreamWaitEvent) until the producer's stream
    // work is actually done, so correctness no longer relies on the producer synchronizing its
    // stream inside the data-producer callback before publishing.

    instance_id = blackboard.registerData(
      std::string(node_.get_fully_qualified_name()) + "_" + publisher->get_topic_name(),
      std::move(cuda_msg_ptr), tickets);

    RCLCPP_DEBUG(
      node_.get_logger(), "Publishing instance id %lu with %ld tickets", instance_id, tickets);

    auto instance_msg = std_msgs::msg::UInt64();
    instance_msg.data = static_cast<uint64_t>(instance_id);
    // the same cast NegotiatedPublisher::publish() makes
    static_cast<rclcpp::Publisher<ROSMessageType> *>(publisher.get())->publish(instance_msg);
  } else if (publish_ros_msg) {
    publish_compatible(*cuda_msg_ptr);
  }

  // When we want to publish ros data, we need to use type adaptation
  if (publish_blackboard_msg && publish_ros_msg) {
    auto data = blackboard.queryData(instance_id);
    publish_compatible(*data);
  }
}

template <typename T>
void CudaBlackboardPublisher<T>::publish_compatible(const T & cuda_msg)
{
#ifdef USE_AGNOCAST_ENABLED
  if (agnocast_compatible_pub_) {
    auto & publisher = agnocast_compatible_pub_->publisher;
    auto ros_msg_ptr = publisher->allocate_output_message_unique();
    rclcpp::TypeAdapter<T>::convert_to_ros_message(cuda_msg, *ros_msg_ptr);
    publisher->publish(std::move(ros_msg_ptr));
    return;
  }
#endif
  auto ros_msg_ptr = std::make_unique<typename T::ros_type>();
  rclcpp::TypeAdapter<T>::convert_to_ros_message(cuda_msg, *ros_msg_ptr);
  compatible_pub_->publish(std::move(ros_msg_ptr));
}

template <typename T>
std::size_t CudaBlackboardPublisher<T>::get_subscription_count() const
{
  std::lock_guard<std::mutex> lock(cache_mutex_);
  std::size_t total = std::accumulate(
    negotiated_cache_.negotiated_pubs.begin(), negotiated_cache_.negotiated_pubs.end(), 0,
    [](int count, const auto & p) { return count + p->get_subscription_count(); });
#ifdef USE_AGNOCAST_ENABLED
  if (agnocast_compatible_pub_) {
    total += agnocast_compatible_pub_->publisher->get_subscription_count();
  }
#endif
  return total;
}

template <typename T>
std::size_t CudaBlackboardPublisher<T>::get_intra_process_subscription_count() const
{
  std::lock_guard<std::mutex> lock(cache_mutex_);
  std::size_t total = std::accumulate(
    negotiated_cache_.negotiated_pubs.begin(), negotiated_cache_.negotiated_pubs.end(), 0,
    [](int count, const auto & p) { return count + p->get_intra_process_subscription_count(); });
#ifdef USE_AGNOCAST_ENABLED
  if (agnocast_compatible_pub_) {
    total += agnocast_compatible_pub_->publisher->get_intra_process_subscription_count();
  }
#endif
  return total;
}

}  // namespace cuda_blackboard

template class cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>;
template class cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaImage>;
