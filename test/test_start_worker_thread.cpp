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

// The listener runs in this process, so the test needs neither a CUDA device nor the Agnocast
// kernel module.

#include "cuda_blackboard/cuda_blackboard_async_publisher.hpp"

#include <agnocast_cie_thread_configurator/non_ros_thread_ipc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <memory>
#include <system_error>
#include <utility>

namespace cuda_blackboard::test
{

using agnocast_cie_thread_configurator::NonRosThreadInfo;
using agnocast_cie_thread_configurator::NonRosThreadInfoListener;

TEST(StartWorkerThreadTest, AnnouncesNodeAtResolvedTopic)
{
  std::promise<NonRosThreadInfo> announced;
  auto announced_future = announced.get_future();
  std::unique_ptr<NonRosThreadInfoListener> listener;
  try {
    listener = std::make_unique<NonRosThreadInfoListener>(
      [&](NonRosThreadInfo info) { announced.set_value(std::move(info)); },
      rclcpp::get_logger("test_start_worker_thread"));
  } catch (const std::system_error &) {
    GTEST_SKIP() << "another listener (prerun_node or the configurator) is bound on this host";
  }

  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("pub_node", "/ns");
  detail::start_worker_thread(*node, "cloud", [] {}).join();

  ASSERT_EQ(announced_future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  const NonRosThreadInfo info = announced_future.get();
  EXPECT_EQ(info.name, "/ns/pub_node@/ns/cloud");
  rclcpp::shutdown();
}

}  // namespace cuda_blackboard::test
