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

// Tests for CudaBlackboardPublisher's negotiation cache: publish() and the subscription counts
// may be called from any thread and read the state that negotiation refreshes on the executor.

#include "cuda_blackboard/cuda_blackboard_publisher.hpp"
#include "cuda_blackboard/cuda_blackboard_subscriber.hpp"
#include "cuda_blackboard/cuda_pointcloud2.hpp"
#include "cuda_blackboard_node_fixture.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <memory>
#include <string>
#include <thread>

namespace cuda_blackboard::test
{

using CudaBlackboardPublisherTest = CudaBlackboardNodeTest;

TEST_F(CudaBlackboardPublisherTest, CountsAreZeroAndPublishIsANoOpBeforeNegotiation)
{
  CudaBlackboardPublisher<CudaPointCloud2> publisher(*pub_node_, "test_cloud");

  EXPECT_EQ(publisher.get_subscription_count(), 0U);
  EXPECT_EQ(publisher.get_intra_process_subscription_count(), 0U);
  EXPECT_NO_THROW(publisher.publish(MakeCloud("none")));
}

TEST_F(CudaBlackboardPublisherTest, WorkerThreadPublishReachesTheSubscriberAfterNegotiation)
{
  CudaBlackboardPublisher<CudaPointCloud2> publisher(*pub_node_, "test_cloud");

  std::atomic<int> received{0};
  std::string received_frame;
  CudaBlackboardSubscriber<CudaPointCloud2> subscriber(
    *sub_node_, "test_cloud",
    [&](std::shared_ptr<const CudaPointCloud2> cloud) {
      received_frame = cloud->header.frame_id;
      ++received;
    },
    consumer_stream_);

  // publish() from a thread that is not the executor's, as a node's worker would. The clouds
  // sent before negotiation has completed are dropped, so keep sending until one arrives.
  std::atomic<bool> stop{false};
  std::thread worker([&]() {
    while (!stop) {
      publisher.publish(MakeCloud("worker"));
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  });
  const bool delivered = SpinUntil([&]() { return received > 0; });
  stop = true;
  worker.join();

  ASSERT_TRUE(delivered) << "subscriber callback was never invoked";
  EXPECT_EQ(received_frame, "worker");
  EXPECT_EQ(publisher.get_subscription_count(), 1U);
  EXPECT_EQ(publisher.get_intra_process_subscription_count(), 1U);
}

}  // namespace cuda_blackboard::test
