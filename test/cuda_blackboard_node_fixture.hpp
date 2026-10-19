// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "cuda_blackboard/cuda_pointcloud2.hpp"
#include "cuda_test_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

namespace cuda_blackboard::test
{

// Fixture for publisher tests that spin real nodes: a publisher node and a subscriber node, so
// that the underlying `negotiated` pair can negotiate the CUDA (intra-process blackboard)
// transport. Requires a CUDA device (via CudaDeviceTest) and initializes/shuts down rclcpp around
// each test. The nodes keep intra-process comms off: the publisher and subscriber enable it per
// entity, and node-wide intra-process would reject negotiated's transient_local topic.
class CudaBlackboardNodeTest : public CudaDeviceTest
{
protected:
  void SetUp() override
  {
    CudaDeviceTest::SetUp();
    if (IsSkipped()) {
      return;
    }
    rclcpp::init(0, nullptr);
    pub_node_ = std::make_shared<rclcpp::Node>("cuda_blackboard_pub_node");
    sub_node_ = std::make_shared<rclcpp::Node>("cuda_blackboard_sub_node");
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(pub_node_);
    executor_->add_node(sub_node_);
    ASSERT_TRUE(CudaSucceeded(cudaStreamCreate(&consumer_stream_)));
  }

  void TearDown() override
  {
    if (consumer_stream_ != nullptr) {
      EXPECT_TRUE(CudaSucceeded(cudaStreamDestroy(consumer_stream_)));
    }
    if (executor_) {
      executor_->remove_node(pub_node_);
      executor_->remove_node(sub_node_);
      executor_.reset();
    }
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  // Spin both nodes, calling `step` once per iteration, until `predicate` returns true or
  // `timeout` elapses. Returns the final value of `predicate`. publish() is a no-op until the
  // CUDA type is negotiated, so a test publishes from `step` on every iteration, as a node does,
  // until the first message arrives.
  bool SpinUntil(
    const std::function<bool()> & predicate, const std::function<void()> & step = [] {},
    std::chrono::milliseconds timeout = std::chrono::seconds(10))
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (predicate()) {
        return true;
      }
      step();
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return predicate();
  }

  static std::unique_ptr<CudaPointCloud2> MakeCloud(const std::string & frame_id)
  {
    sensor_msgs::msg::PointCloud2 ros_cloud;
    ros_cloud.header.frame_id = frame_id;
    ros_cloud.height = 1;
    ros_cloud.width = 8;
    ros_cloud.point_step = 16;
    ros_cloud.row_step = ros_cloud.point_step * ros_cloud.width;
    ros_cloud.data.assign(ros_cloud.row_step, 0x42);
    return std::make_unique<CudaPointCloud2>(ros_cloud);
  }

  rclcpp::Node::SharedPtr pub_node_;
  rclcpp::Node::SharedPtr sub_node_;
  // Created in SetUp(), after rclcpp::init(): an executor needs the context.
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  cudaStream_t consumer_stream_{nullptr};
};

}  // namespace cuda_blackboard::test
