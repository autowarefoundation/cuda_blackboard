// Integration-style unit tests for CudaBlackboardPublisher / CudaBlackboardSubscriber.
//
// These spin real rclcpp nodes so the underlying `negotiated` publisher/subscription can discover
// each other and negotiate the CUDA (intra-process blackboard) transport, then verify that a
// published CudaImage is delivered to the subscriber callback.

#include "cuda_blackboard/cuda_blackboard_publisher.hpp"
#include "cuda_blackboard/cuda_blackboard_subscriber.hpp"
#include "cuda_blackboard/cuda_image.hpp"
#include "cuda_test_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace cuda_blackboard::test
{

using namespace std::chrono_literals;

// Spin the executor until `predicate` returns true or `timeout` elapses.
// Returns the final value of `predicate`.
template <typename Predicate>
bool SpinUntil(
  rclcpp::Executor & executor, Predicate predicate, std::chrono::milliseconds timeout_ms)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout_ms;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    executor.spin_some();
    std::this_thread::sleep_for(2ms);
  }
  return predicate();
}

// Fixture for the publisher/subscriber integration tests: requires a CUDA device (via
// CudaDeviceTest), initializes/shuts down rclcpp around each test, and gathers the ROS-image
// factory shared by the cases.
class CudaBlackboardPubSubTest : public CudaDeviceTest
{
protected:
  void SetUp() override
  {
    CudaDeviceTest::SetUp();
    if (IsSkipped()) {
      return;
    }
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  static sensor_msgs::msg::Image MakeRosImage()
  {
    sensor_msgs::msg::Image image;
    image.header.frame_id = "camera";
    image.header.stamp.sec = 12;
    image.header.stamp.nanosec = 34;
    image.height = 2;
    image.width = 3;
    image.encoding = "rgb8";
    image.is_bigendian = false;
    image.step = image.width * 3;
    image.data.resize(image.height * image.step);
    for (std::size_t i = 0; i < image.data.size(); ++i) {
      image.data[i] = static_cast<uint8_t>(i + 1);
    }
    return image;
  }
};

// The negotiated subscription must discover the negotiated publisher on the same topic; once it
// does, the publisher reports a (intra-process) subscription. This mirrors the discovery-waiting
// done in qos.cpp.
TEST_F(CudaBlackboardPubSubTest, SubscriberDiscoversPublisher)
{
  auto pub_node = std::make_shared<rclcpp::Node>("cuda_blackboard_pub_node");
  auto sub_node = std::make_shared<rclcpp::Node>("cuda_blackboard_sub_node");

  const std::string topic = "test_image";

  CudaBlackboardPublisher<CudaImage> publisher(*pub_node, topic);
  CudaBlackboardSubscriber<CudaImage> subscriber(
    *sub_node, topic, [](std::shared_ptr<const CudaImage>) {}, cudaStreamLegacy);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pub_node);
  executor.add_node(sub_node);

  const bool connected =
    SpinUntil(executor, [&]() { return publisher.get_subscription_count() > 0; }, 10s);

  EXPECT_TRUE(connected) << "publisher never observed the negotiated subscription";
  EXPECT_GT(publisher.get_subscription_count(), 0U);

  executor.remove_node(pub_node);
  executor.remove_node(sub_node);
}

// Full path: after negotiation, publishing a CudaImage should reach the subscriber callback with
// the device payload intact (delivered via the blackboard, the intra-process CUDA transport).
TEST_F(CudaBlackboardPubSubTest, PublishDeliversImageToSubscriber)
{
  cudaStream_t consumer_stream = nullptr;
  ASSERT_TRUE(CudaSucceeded(cudaStreamCreate(&consumer_stream)));

  auto pub_node = std::make_shared<rclcpp::Node>("cuda_blackboard_pub_node");
  auto sub_node = std::make_shared<rclcpp::Node>("cuda_blackboard_sub_node");

  const std::string topic = "test_image";
  const auto ros_image = MakeRosImage();

  std::vector<uint8_t> received_data;
  sensor_msgs::msg::Image received_meta;
  std::size_t received_count = 0;

  auto callback = [&](std::shared_ptr<const CudaImage> msg) {
    received_meta.header = msg->header;
    received_meta.height = msg->height;
    received_meta.width = msg->width;
    received_meta.encoding = msg->encoding;
    received_meta.is_bigendian = msg->is_bigendian;
    received_meta.step = msg->step;

    received_data.resize(msg->height * msg->step);
    // The subscriber already injected a wait on `consumer_stream` for the buffer's ready_event,
    // so this copy on the same stream is correctly ordered after the producer's work.
    EXPECT_TRUE(CudaSucceeded(cudaMemcpyAsync(
      received_data.data(), msg->data.get(), received_data.size() * sizeof(uint8_t),
      cudaMemcpyDeviceToHost, consumer_stream)));
    EXPECT_TRUE(CudaSucceeded(cudaStreamSynchronize(consumer_stream)));
    ++received_count;
  };

  CudaBlackboardPublisher<CudaImage> publisher(*pub_node, topic);
  CudaBlackboardSubscriber<CudaImage> subscriber(*sub_node, topic, callback, consumer_stream);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pub_node);
  executor.add_node(sub_node);

  // Negotiation can briefly flap before it settles, and publish() is a no-op until the CUDA type
  // is negotiated with an intra-process subscriber. So we publish on every spin iteration (a fresh
  // buffer each time, since publish() takes ownership) until the callback fires or we time out.
  const auto deadline = std::chrono::steady_clock::now() + 15s;
  while (received_count == 0 && std::chrono::steady_clock::now() < deadline) {
    if (publisher.get_intra_process_subscription_count() > 0) {
      publisher.publish(std::make_unique<const CudaImage>(ros_image));
    }
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }

  ASSERT_GT(received_count, 0U) << "subscriber callback was never invoked";
  EXPECT_EQ(received_meta.header.frame_id, ros_image.header.frame_id);
  EXPECT_EQ(received_meta.height, ros_image.height);
  EXPECT_EQ(received_meta.width, ros_image.width);
  EXPECT_EQ(received_meta.encoding, ros_image.encoding);
  EXPECT_EQ(received_meta.step, ros_image.step);
  EXPECT_EQ(received_data, ros_image.data);

  executor.remove_node(pub_node);
  executor.remove_node(sub_node);
  EXPECT_TRUE(CudaSucceeded(cudaStreamDestroy(consumer_stream)));
}

}  // namespace cuda_blackboard::test
