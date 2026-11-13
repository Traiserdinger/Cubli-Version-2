// Lock-free primitives for passing data between the control task (core 1) and
// the low-priority tasks (core 0) without blocking the control loop.
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace cubli {

/// Single-producer / single-consumer ring of trivially-copyable items.
/// One slot is kept empty, so capacity is N - 1. N must be a power of two.
template <typename T, size_t N>
class SpscRing {
    static_assert((N & (N - 1)) == 0 && N >= 2, "N must be a power of two");
    static_assert(std::is_trivially_copyable<T>::value, "T must be trivially copyable");

public:
    /// Producer side. Returns false (item dropped) when full.
    bool push(const T& item) {
        const size_t head = head_.load(std::memory_order_relaxed);
        const size_t next = (head + 1) & (N - 1);
        if (next == tail_.load(std::memory_order_acquire)) {
            dropped_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        buf_[head] = item;
        head_.store(next, std::memory_order_release);
        return true;
    }
    /// Consumer side.
    bool pop(T& out) {
        const size_t tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire)) return false;
        out = buf_[tail];
        tail_.store((tail + 1) & (N - 1), std::memory_order_release);
        return true;
    }
    bool empty() const {
        return tail_.load(std::memory_order_acquire) == head_.load(std::memory_order_acquire);
    }
    uint32_t dropped() const { return dropped_.load(std::memory_order_relaxed); }

private:
    T buf_[N]{};
    std::atomic<size_t> head_{0};
    std::atomic<size_t> tail_{0};
    std::atomic<uint32_t> dropped_{0};
};

/// Sequence lock: one writer that never blocks, any number of readers that retry.
/// The payload is copied word-wise through relaxed atomics so concurrent access is
/// well defined.
template <typename T>
class SeqLock {
    static_assert(std::is_trivially_copyable<T>::value, "T must be trivially copyable");
    static constexpr size_t kWords = (sizeof(T) + 3) / 4;

public:
    void write(const T& v) {
        uint32_t tmp[kWords] = {};
        std::memcpy(tmp, &v, sizeof(T));
        const uint32_t s = seq_.load(std::memory_order_relaxed);
        seq_.store(s + 1, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);
        for (size_t i = 0; i < kWords; ++i) words_[i].store(tmp[i], std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);
        seq_.store(s + 2, std::memory_order_relaxed);
    }
    /// Returns false only if a consistent copy could not be taken (writer busy).
    bool read(T& out, int max_tries = 100) const {
        for (int attempt = 0; attempt < max_tries; ++attempt) {
            const uint32_t s0 = seq_.load(std::memory_order_acquire);
            if (s0 & 1u) continue;
            uint32_t tmp[kWords];
            for (size_t i = 0; i < kWords; ++i) tmp[i] = words_[i].load(std::memory_order_relaxed);
            std::atomic_thread_fence(std::memory_order_acquire);
            if (seq_.load(std::memory_order_relaxed) == s0) {
                std::memcpy(&out, tmp, sizeof(T));
                return true;
            }
        }
        return false;
    }
    uint32_t version() const { return seq_.load(std::memory_order_acquire); }

private:
    std::atomic<uint32_t> seq_{0};
    std::atomic<uint32_t> words_[kWords]{};
};

}  // namespace cubli
