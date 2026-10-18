// Fixed-capacity single-producer / single-consumer lock-free queue (TD-7).
//
// Used to pass small POD commands (mode changes) from the aux executor to the
// fast tick, which drains it at the start of each cycle. No allocation after
// construction; push fails rather than blocks when full.
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <type_traits>

namespace pat_common
{

template<typename T, std::size_t Capacity>
class SpscQueue
{
  static_assert(std::is_trivially_copyable_v<T>, "SpscQueue requires trivially copyable T");
  static_assert((Capacity & (Capacity - 1)) == 0, "Capacity must be a power of two");

public:
  bool push(const T & v) noexcept
  {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    const std::size_t next = (head + 1) & kMask;
    if (next == tail_.load(std::memory_order_acquire)) {return false;}
    buf_[head] = v;
    head_.store(next, std::memory_order_release);
    return true;
  }

  bool pop(T & out) noexcept
  {
    const std::size_t tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) {return false;}
    out = buf_[tail];
    tail_.store((tail + 1) & kMask, std::memory_order_release);
    return true;
  }

  bool empty() const noexcept
  {
    return head_.load(std::memory_order_acquire) == tail_.load(std::memory_order_acquire);
  }

  static constexpr std::size_t capacity() noexcept {return Capacity - 1;}

private:
  static constexpr std::size_t kMask = Capacity - 1;
  alignas(64) std::atomic<std::size_t> head_{0};
  alignas(64) std::atomic<std::size_t> tail_{0};
  std::array<T, Capacity> buf_{};
};

}  // namespace pat_common
