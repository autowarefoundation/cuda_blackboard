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

// Tests for PublishMailbox, the single-slot newest-wins handover between a node's callback and
// the async publisher's worker. Header-only and CUDA-free, so it runs without a device.

#include "cuda_blackboard/publish_mailbox.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <thread>
#include <utility>

namespace cuda_blackboard::test
{

using Item = std::unique_ptr<int>;

TEST(PublishMailboxTest, TakeReturnsTheDepositedItem)
{
  PublishMailbox<Item> mailbox;
  mailbox.deposit(std::make_unique<int>(1));

  std::optional<Item> item = mailbox.take();
  ASSERT_TRUE(item.has_value());
  EXPECT_EQ(**item, 1);
  EXPECT_EQ(mailbox.dropped(), 0U);
}

TEST(PublishMailboxTest, NewerDepositReplacesTheWaitingItemAndCountsIt)
{
  PublishMailbox<Item> mailbox;
  mailbox.deposit(std::make_unique<int>(1));
  mailbox.deposit(std::make_unique<int>(2));
  mailbox.deposit(std::make_unique<int>(3));

  std::optional<Item> item = mailbox.take();
  ASSERT_TRUE(item.has_value());
  EXPECT_EQ(**item, 3);
  EXPECT_EQ(mailbox.dropped(), 2U);
}

TEST(PublishMailboxTest, TakeBlocksUntilADepositArrives)
{
  PublishMailbox<Item> mailbox;
  std::atomic<bool> taken{false};
  std::optional<Item> item;
  std::thread consumer([&]() {
    item = mailbox.take();
    taken = true;
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_FALSE(taken);

  mailbox.deposit(std::make_unique<int>(7));
  consumer.join();
  EXPECT_TRUE(taken);
  ASSERT_TRUE(item.has_value());
  EXPECT_EQ(**item, 7);
}

TEST(PublishMailboxTest, StopWakesTheConsumerAndDiscardsTheWaitingItem)
{
  PublishMailbox<Item> mailbox;
  mailbox.deposit(std::make_unique<int>(1));
  std::optional<Item> item;
  std::thread consumer([&]() {
    item = mailbox.take();  // the deposited item
    item = mailbox.take();  // blocks until stop()
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  mailbox.stop();
  consumer.join();
  EXPECT_FALSE(item.has_value());

  // Nothing is handed out or kept after stop().
  mailbox.deposit(std::make_unique<int>(2));
  EXPECT_FALSE(mailbox.take().has_value());
  EXPECT_EQ(mailbox.dropped(), 0U);
}

}  // namespace cuda_blackboard::test
