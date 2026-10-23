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

// Tests for CudaBlackboardAsyncPublisher: publish() only deposits into the mailbox, the worker
// thread does the publishing, and the destructor joins the worker.

#include "cuda_blackboard/cuda_blackboard_async_publisher.hpp"
#include "cuda_blackboard/cuda_blackboard_subscriber.hpp"
#include "cuda_blackboard/cuda_pointcloud2.hpp"
#include "cuda_blackboard_node_fixture.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <memory>
#include <string>

namespace cuda_blackboard::test
{

using CudaBlackboardAsyncPublisherTest = CudaBlackboardNodeTest;

TEST_F(CudaBlackboardAsyncPublisherTest, DeliversUntilDestroyed)
{
  std::atomic<int> received{0};
  std::string received_frame;
  {
    CudaBlackboardAsyncPublisher<CudaPointCloud2> publisher(*pub_node_, "test_cloud");
    CudaBlackboardSubscriber<CudaPointCloud2> subscriber(
      *sub_node_, "test_cloud",
      [&](std::shared_ptr<const CudaPointCloud2> cloud) {
        received_frame = cloud->header.frame_id;
        ++received;
      },
      consumer_stream_);

    ASSERT_TRUE(
      SpinUntil([&]() { return received > 0; }, [&]() { publisher.publish(MakeCloud("async")); }))
      << "subscriber callback was never invoked";
    EXPECT_EQ(received_frame, "async");
    EXPECT_EQ(publisher.get_intra_process_subscription_count(), 1U);
  }
  // Destroyed with its worker joined: nothing may arrive afterwards.
  const int after_destruction = received;
  executor_->spin_some();
  EXPECT_EQ(received, after_destruction);
}

}  // namespace cuda_blackboard::test
