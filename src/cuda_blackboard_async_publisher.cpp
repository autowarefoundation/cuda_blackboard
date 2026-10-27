
#include "cuda_blackboard/cuda_blackboard_async_publisher.hpp"

#ifdef USE_AGNOCAST_ENABLED
#include <agnocast_cie_thread_configurator/cie_thread_configurator.hpp>
#include <autoware/agnocast_wrapper/runtime.hpp>
#endif

#include <functional>
#include <string>
#include <thread>
#include <utility>

namespace cuda_blackboard::detail
{

std::thread start_worker_thread(
  [[maybe_unused]] rclcpp::Node & node, [[maybe_unused]] const std::string & topic_name,
  std::function<void()> body)
{
#ifdef USE_AGNOCAST_ENABLED
  // The configurator matches the name exactly against its configuration, so it must be unique
  // among the threads it manages and the same in every run.
  if (autoware::agnocast_wrapper::use_agnocast()) {
    const std::string thread_name =
      std::string(node.get_fully_qualified_name()) + "@" +
      node.get_node_topics_interface()->resolve_topic_name(topic_name);
    return agnocast_cie_thread_configurator::spawn_non_ros2_thread(
      thread_name.c_str(), std::move(body));
  }
#endif
  return std::thread(std::move(body));
}

}  // namespace cuda_blackboard::detail
