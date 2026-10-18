#include <atomic>
#include <cstdint>
#include <thread>

#include <gtest/gtest.h>

#include "pat_common/seqlock_buffer.hpp"
#include "pat_common/spsc_queue.hpp"
#include "pat_common/units.hpp"

namespace
{

// Payload whose fields are always written consistently, so a torn read is
// detectable: every field must equal the same generation counter.
struct Snapshot
{
  uint64_t a;
  uint64_t b;
  uint64_t c;
  uint64_t d;
};

}  // namespace

TEST(SeqlockBuffer, ReadReturnsLastWrite)
{
  pat_common::SeqlockBuffer<Snapshot> buf(Snapshot{1, 1, 1, 1});
  EXPECT_EQ(buf.read().a, 1u);
  buf.write(Snapshot{7, 7, 7, 7});
  EXPECT_EQ(buf.read().d, 7u);
  EXPECT_EQ(buf.sequence(), 2u);
}

TEST(SeqlockBuffer, NeverObservesTornState)
{
  pat_common::SeqlockBuffer<Snapshot> buf(Snapshot{0, 0, 0, 0});
  std::atomic<bool> stop{false};
  std::atomic<uint64_t> torn{0};
  std::atomic<uint64_t> reads{0};

  std::thread writer([&] {
      for (uint64_t g = 1; !stop.load(std::memory_order_relaxed); ++g) {
        buf.write(Snapshot{g, g, g, g});
      }
    });
  std::thread reader([&] {
      while (!stop.load(std::memory_order_relaxed)) {
        const Snapshot s = buf.read();
        reads.fetch_add(1, std::memory_order_relaxed);
        if (s.a != s.b || s.b != s.c || s.c != s.d) {torn.fetch_add(1, std::memory_order_relaxed);}
      }
    });
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  stop = true;
  writer.join();
  reader.join();

  EXPECT_GT(reads.load(), 1000u);
  EXPECT_EQ(torn.load(), 0u);
}

TEST(SpscQueue, FifoAndCapacity)
{
  pat_common::SpscQueue<int, 8> q;
  EXPECT_TRUE(q.empty());
  for (int i = 0; i < 7; ++i) {EXPECT_TRUE(q.push(i));}
  EXPECT_FALSE(q.push(99)) << "queue must refuse rather than block when full";
  for (int i = 0; i < 7; ++i) {
    int v = -1;
    ASSERT_TRUE(q.pop(v));
    EXPECT_EQ(v, i);
  }
  int v;
  EXPECT_FALSE(q.pop(v));
  EXPECT_TRUE(q.empty());
}

TEST(SpscQueue, CrossThreadOrdering)
{
  pat_common::SpscQueue<uint32_t, 64> q;
  constexpr uint32_t kN = 200000;
  std::thread producer([&] {
      for (uint32_t i = 0; i < kN; ) {
        if (q.push(i)) {++i;}
      }
    });
  uint32_t expected = 0;
  while (expected < kN) {
    uint32_t v;
    if (q.pop(v)) {
      ASSERT_EQ(v, expected);
      ++expected;
    }
  }
  producer.join();
  EXPECT_TRUE(q.empty());
}

TEST(Units, Conversions)
{
  EXPECT_NEAR(pat_common::deg_to_urad(1.0), 17453.29, 0.01);
  EXPECT_NEAR(pat_common::urad_to_deg(pat_common::deg_to_urad(0.5)), 0.5, 1e-12);
  const auto t = pat_common::from_ns(1'500'000'123LL);
  EXPECT_EQ(t.sec, 1);
  EXPECT_EQ(t.nanosec, 500'000'123u);
  EXPECT_EQ(pat_common::to_ns(t), 1'500'000'123LL);
}
