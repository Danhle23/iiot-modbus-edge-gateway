#pragma once
// ============================================================================
// ring_buffer.hpp - Lock-Free SPSC Static Ring Buffer for UART DMA / ISR
// Zero dynamic memory allocation (MISRA / Embedded C++17 compliant)
// Author: Le Thanh Danh (Danhle23)
// ============================================================================
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace iiot {

template <typename T, std::size_t CapacityPow2>
class SpscRingBuffer {
    static_assert((CapacityPow2 >= 2) && ((CapacityPow2 & (CapacityPow2 - 1)) == 0),
                  "CapacityPow2 must be a power of 2 for single-cycle bitmask wrap");

public:
    constexpr SpscRingBuffer() noexcept = default;

    [[nodiscard]] bool push(const T& item) noexcept {
        const std::size_t head = head_.load(std::memory_order_relaxed);
        const std::size_t next = (head + 1U) & kMask;
        if (next == tail_.load(std::memory_order_acquire)) {
            return false; // Buffer full
        }
        storage_[head] = item;
        head_.store(next, std::memory_order_release);
        return true;
    }

    [[nodiscard]] bool pop(T& out) noexcept {
        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire)) {
            return false; // Buffer empty
        }
        out = storage_[tail];
        tail_.store((tail + 1U) & kMask, std::memory_order_release);
        return true;
    }

    [[nodiscard]] std::size_t size() const noexcept {
        const std::size_t head = head_.load(std::memory_order_acquire);
        const std::size_t tail = tail_.load(std::memory_order_acquire);
        return (head - tail) & kMask;
    }

    [[nodiscard]] bool empty() const noexcept {
        return head_.load(std::memory_order_acquire) == tail_.load(std::memory_order_acquire);
    }

    void clear() noexcept {
        tail_.store(head_.load(std::memory_order_relaxed), std::memory_order_release);
    }

private:
    static constexpr std::size_t kMask = CapacityPow2 - 1U;
    std::array<T, CapacityPow2> storage_{};
    std::atomic<std::size_t> head_{0};
    std::atomic<std::size_t> tail_{0};
};

} // namespace iiot
