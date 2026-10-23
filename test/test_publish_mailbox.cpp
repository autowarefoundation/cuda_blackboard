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

#include <chrono>
#include <future>
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
  std::promise<void> started;
  std::promise<void> taken;
  std::future<void> taken_future = taken.get_future();
  std::optional<Item> item;
  std::thread consumer([&]() {
    started.set_value();
    item = mailbox.take();
    taken.set_value();
  });

  started.get_future().wait();
  EXPECT_EQ(taken_future.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);

  mailbox.deposit(std::make_unique<int>(7));
  consumer.join();
  ASSERT_TRUE(item.has_value());
  EXPECT_EQ(**item, 7);
}

TEST(PublishMailboxTest, StopWakesTheConsumerAndDiscardsTheWaitingItem)
{
  PublishMailbox<Item> mailbox;
  mailbox.deposit(std::make_unique<int>(1));
  std::promise<void> first_taken;
  std::optional<Item> first;
  std::optional<Item> second;
  std::thread consumer([&]() {
    first = mailbox.take();
    first_taken.set_value();
    second = mailbox.take();  // blocks until stop()
  });

  first_taken.get_future().wait();
  mailbox.stop();
  consumer.join();
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(**first, 1);
  EXPECT_FALSE(second.has_value());

  // Nothing is handed out or kept after stop().
  mailbox.deposit(std::make_unique<int>(2));
  EXPECT_FALSE(mailbox.take().has_value());
  EXPECT_EQ(mailbox.dropped(), 0U);
}

}  // namespace cuda_blackboard::test
