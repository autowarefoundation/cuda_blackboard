
#pragma once

#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <utility>

namespace cuda_blackboard
{

/// Single-slot, newest-wins hand-over from a producer to one consumer thread.
///
/// deposit() never blocks; it replaces an untaken item and counts it in dropped(). take() blocks
/// until an item arrives or stop() is called. stop() discards the pending item; afterwards take()
/// returns nothing and deposit() discards its argument. Items are destroyed outside the lock.
///
/// @tparam T Any movable type.
template <typename T>
class PublishMailbox
{
public:
  void deposit(T && item)
  {
    std::optional<T> discarded;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopped_) {
        discarded.emplace(std::move(item));
      } else {
        if (item_.has_value()) {
          discarded = std::move(item_);
          ++dropped_;
        }
        item_.emplace(std::move(item));
      }
    }
    cv_.notify_one();
  }

  std::optional<T> take()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this] { return stopped_ || item_.has_value(); });
    if (stopped_) {
      return std::nullopt;
    }
    std::optional<T> item = std::move(item_);
    item_.reset();
    return item;
  }

  void stop()
  {
    std::optional<T> discarded;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopped_ = true;
      discarded = std::move(item_);
      item_.reset();
    }
    cv_.notify_all();
  }

  /// Number of items replaced before the consumer took them (cumulative).
  std::uint64_t dropped() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return dropped_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::optional<T> item_;
  bool stopped_{false};
  std::uint64_t dropped_{0};
};

}  // namespace cuda_blackboard
