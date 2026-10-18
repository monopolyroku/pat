// Single-writer / single-reader seqlock-protected double buffer (TD-7).
//
// The writer (slow path, e.g. camera correction on E2) never blocks. The
// reader (fast path, E1) copies a snapshot and retries if a write was in
// progress or completed during the copy. T must be trivially copyable so the
// racy copy of a half-written back buffer is benign and simply discarded.
#pragma once

#include <atomic>
#include <cstdint>
#include <type_traits>

namespace pat_common
{

template<typename T>
class SeqlockBuffer
{
  static_assert(std::is_trivially_copyable_v<T>, "SeqlockBuffer requires trivially copyable T");

public:
  SeqlockBuffer() = default;
  explicit SeqlockBuffer(const T & initial)
  : value_(initial) {}

  // Writer side. Sequence is odd while a write is in progress.
  void write(const T & v) noexcept
  {
    const uint32_t s = seq_.load(std::memory_order_relaxed);
    seq_.store(s + 1, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    value_ = v;
    std::atomic_thread_fence(std::memory_order_release);
    seq_.store(s + 2, std::memory_order_release);
  }

  // Reader side. Returns a consistent snapshot; bounded retries in practice
  // because writes are rare and short relative to reads.
  T read() const noexcept
  {
    T out;
    for (;;) {
      const uint32_t s1 = seq_.load(std::memory_order_acquire);
      if (s1 & 1u) {continue;}
      out = value_;
      std::atomic_thread_fence(std::memory_order_acquire);
      const uint32_t s2 = seq_.load(std::memory_order_relaxed);
      if (s1 == s2) {return out;}
    }
  }

  // Sequence number of the last completed write (always even); lets the
  // reader detect whether anything changed since it last looked.
  uint32_t sequence() const noexcept {return seq_.load(std::memory_order_acquire) & ~1u;}

private:
  alignas(64) std::atomic<uint32_t> seq_{0};
  alignas(64) T value_{};
};

}  // namespace pat_common
