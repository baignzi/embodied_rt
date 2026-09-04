// lock_free_ring_buffer.hpp — SPSC无锁环形缓冲区
#pragma once
#include <atomic>
#include <array>
#include <cstddef>
#include <optional>

template<typename T, std::size_t Capacity>
class LockFreeRingBuffer {
static_assert((Capacity & (Capacity - 1)) == 0,
    "Capacity must be power of 2 for branchless modulo");

public:
    LockFreeRingBuffer() : head_(0), tail_(0) {}

    // 生产者调用（VLA线程，10Hz）
    bool push(const T& item) {
        const std::size_t curr = head_.load(std::memory_order_relaxed);
        const std::size_t next = (curr + 1) & mask_;
        // 如果buffer满，采用overwrite策略（丢旧动作）
        if (next == tail_.load(std::memory_order_acquire)) {
            // 推进tail，丢弃最旧的动作
            tail_.store((tail_.load(std::memory_order_relaxed) + 1) & mask_,
                        std::memory_order_release);
        }
        buffer_[curr] = item;
        head_.store(next, std::memory_order_release);
        return true;
    }

    // 消费者调用（控制线程，1000Hz）
    std::optional<T> pop() {
        const std::size_t curr = tail_.load(std::memory_order_relaxed);
        if (curr == head_.load(std::memory_order_acquire)) {
            return std::nullopt;  // buffer空，保持上一个动作
        }
        const T item = buffer_[curr];
        tail_.store((curr + 1) & mask_, std::memory_order_release);
        return item;
    }

    // 查看最新动作但不弹出（控制回路在没有新动作时持续追踪上一目标）
    std::optional<T> peek_latest() const {
        const std::size_t h = head_.load(std::memory_order_acquire);
        const std::size_t t = tail_.load(std::memory_order_acquire);
        if (h == t) return std::nullopt;
        return buffer_[(h - 1) & mask_];
    }

private:
    static constexpr std::size_t mask_ = Capacity - 1;
    std::array<T, Capacity> buffer_;
    alignas(64) std::atomic<std::size_t> head_;  // false sharing隔离
    alignas(64) std::atomic<std::size_t> tail_;
};
