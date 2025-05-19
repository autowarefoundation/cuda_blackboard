
#include "cuda_blackboard/cuda_blackboard_publisher.hpp"

#include "cuda_blackboard/cuda_adaptation.hpp"
#include "cuda_blackboard/cuda_blackboard.hpp"
#include "cuda_blackboard/negotiated_types.hpp"

namespace cuda_blackboard
{

template <typename T>
CudaBlackboardPublisher<T>::CudaBlackboardPublisher(
  rclcpp::Node & node, const std::string & topic_name)
: node_(node)
{
  negotiated::NegotiatedPublisherOptions negotiation_options;
  negotiation_options.disconnect_publishers_on_failure = false;

  negotiated_pub_ = std::make_shared<negotiated::NegotiatedPublisher>(
    node, topic_name + "/cuda", negotiation_options);
  rclcpp::PublisherOptions pub_options;
  pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

  negotiated_pub_->add_supported_type<NegotiationStruct<T>>(
    1.0,
    rclcpp::QoS(1),  //.durability_volatile(),
    pub_options);

  std::string ros_type_name = NegotiationStruct<typename T::ros_type>::supported_type_name;
  compatible_pub_ =
    node.create_publisher<typename T::ros_type>(topic_name, rclcpp::SensorDataQoS(), pub_options);
  negotiated_pub_->add_compatible_publisher(compatible_pub_, ros_type_name, 0.1);

  negotiated_pub_->start();
}

template <typename T>
void CudaBlackboardPublisher<T>::publish(std::unique_ptr<const T> cuda_msg_ptr)
{
  auto & map = negotiated_pub_->get_supported_types();

  using ROSMessageType = typename NegotiationStruct<T>::MsgT;
  std::string ros_type_name = rosidl_generator_traits::name<ROSMessageType>();
  std::string key_name =
    negotiated::detail::generate_key(ros_type_name, NegotiationStruct<T>::supported_type_name);

  uint64_t instance_id{0};
  bool publish_ros_msg =
    negotiated_pub_->type_was_negotiated<NegotiationStruct<typename T::ros_type>>() &&
    (compatible_pub_->get_subscription_count() > 0 ||
     compatible_pub_->get_intra_process_subscription_count() > 0);
  bool publish_blackboard_msg =
    negotiated_pub_->type_was_negotiated<NegotiationStruct<T>>() && map.count(key_name) > 0;

  auto & blackboard = CudaBlackboard<T>::getInstance();
  std::shared_ptr<const T> blackboard_data = nullptr;

  // When we want to publish cuda data, we instead use the blackboard
  if (publish_blackboard_msg) {
    auto & publisher = map.at(key_name).publisher;
    std::size_t tickets =
      publisher->get_intra_process_subscription_count();  // tickets are only given to intra process
                                                          // subscribers

    if (tickets == 0 && !publish_ros_msg) {
      RCLCPP_WARN(node_.get_logger(), "there is no intra process subscription");
      return;
    }

    if (publish_ros_msg) {
      tickets++;
    }

    instance_id = blackboard.registerData(
      std::string(node_.get_fully_qualified_name()) + "_" + publisher->get_topic_name(),
      std::move(cuda_msg_ptr), tickets);

    RCLCPP_DEBUG(
      node_.get_logger(), "Publishing instance id %lu with %ld tickets", instance_id, tickets);

    RCLCPP_WARN(node_.get_logger(), "publish blackboard msg");
    auto instance_msg = std_msgs::msg::UInt64();
    instance_msg.data = static_cast<uint64_t>(instance_id);
    negotiated_pub_->publish<NegotiationStruct<T>>(instance_msg);

    if (publish_ros_msg) {
      blackboard_data = blackboard.queryData(instance_id);  // for ROS conversion if needed
    }
  }

  if (publish_ros_msg) {
    std::unique_ptr<typename T::ros_type> ros_msg_ptr = std::make_unique<typename T::ros_type>();

    if (blackboard_data) {
      rclcpp::TypeAdapter<T>::convert_to_ros_message(*blackboard_data, *ros_msg_ptr);
    } else {
      rclcpp::TypeAdapter<T>::convert_to_ros_message(*cuda_msg_ptr, *ros_msg_ptr);
    }

    RCLCPP_WARN(node_.get_logger(), "publish ros msg");
    compatible_pub_->publish(std::move(ros_msg_ptr));
  }

  if (!publish_blackboard_msg && !publish_ros_msg) {
    RCLCPP_WARN(node_.get_logger(), "No blackboard or ROS subscribers");
  }
  RCLCPP_WARN(node_.get_logger(), "end publish");
}

template <typename T>
std::size_t CudaBlackboardPublisher<T>::get_subscription_count() const
{
  auto & map = negotiated_pub_->get_supported_types();
  return std::accumulate(map.begin(), map.end(), 0, [](int count, const auto & p) {
    std::size_t sub_count = p.second.publisher ? p.second.publisher->get_subscription_count() : 0;
    return count + sub_count;
  });
}

template <typename T>
std::size_t CudaBlackboardPublisher<T>::get_intra_process_subscription_count() const
{
  auto & map = negotiated_pub_->get_supported_types();
  return std::accumulate(map.begin(), map.end(), 0, [](int count, const auto & p) {
    std::size_t sub_count =
      p.second.publisher ? p.second.publisher->get_intra_process_subscription_count() : 0;
    return count + sub_count;
  });
}

}  // namespace cuda_blackboard

template class cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>;
template class cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaImage>;
