
#pragma once

#include "cuda_blackboard/cuda_blackboard_publisher.hpp"
#include "cuda_blackboard/publish_mailbox.hpp"

#include <rclcpp/rclcpp.hpp>

#include <pthread.h>

#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <utility>

namespace cuda_blackboard
{

namespace detail
{
/// Runs body on a new thread. On Agnocast, the thread is announced to the CIE thread configurator
/// as "<fully qualified node name>@<resolved topic name>", so that the configurator can set its
/// scheduling.
std::thread start_worker_thread(
  rclcpp::Node & node, const std::string & topic_name, std::function<void()> body);
}  // namespace detail

/// A CudaBlackboardPublisher whose publish() returns at once: the message goes into a single-slot
/// mailbox and a worker thread owned by this object does the publish (D2H copy and serialization
/// for ROS subscribers). A message the worker has not taken yet is replaced by the newer one and
/// counted in dropped(). The destructor stops and joins the worker, so the thread never outlives
/// the publisher. Subscriber counts are the wrapped publisher's.
template <typename T>
class CudaBlackboardAsyncPublisher
{
public:
  CudaBlackboardAsyncPublisher(rclcpp::Node & node, const std::string & topic_name)
  : logger_(node.get_logger()), publisher_(node, topic_name)
  {
    // Last, so that a throwing constructor never leaves a joinable thread behind.
    worker_ =
      detail::start_worker_thread(node, topic_name, [this, topic_name] { run(topic_name); });
  }

  ~CudaBlackboardAsyncPublisher()
  {
    mailbox_.stop();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  CudaBlackboardAsyncPublisher(const CudaBlackboardAsyncPublisher &) = delete;
  CudaBlackboardAsyncPublisher & operator=(const CudaBlackboardAsyncPublisher &) = delete;

  /// Hands the message to the worker and returns; a message still waiting there is replaced.
  void publish(std::unique_ptr<const T> msg) { mailbox_.deposit(std::move(msg)); }

  std::size_t get_subscription_count() const { return publisher_.get_subscription_count(); }
  std::size_t get_intra_process_subscription_count() const
  {
    return publisher_.get_intra_process_subscription_count();
  }

  /// Messages replaced before the worker published them (cumulative).
  std::uint64_t dropped() const { return mailbox_.dropped(); }

private:
  void run(const std::string & topic_name)
  {
    // Linux limits thread names to 15 characters: keep the last topic segment.
    const std::string name = topic_name.substr(topic_name.rfind('/') + 1).substr(0, 15);
    pthread_setname_np(pthread_self(), name.c_str());
    while (auto msg = mailbox_.take()) {
      try {
        publisher_.publish(std::move(*msg));
      } catch (const std::exception & e) {
        RCLCPP_ERROR(logger_, "Publishing %s failed: %s", topic_name.c_str(), e.what());
      }
    }
  }

  rclcpp::Logger logger_;
  CudaBlackboardPublisher<T> publisher_;
  PublishMailbox<std::unique_ptr<const T>> mailbox_;
  std::thread worker_;  // declared last: joined in the destructor before the members above go
};

}  // namespace cuda_blackboard
