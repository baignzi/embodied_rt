/**
 * @file lock_free_ring_buffer.hpp
 * @brief SPSC（单生产者单消费者）无锁环形缓冲区
 *
 * 用于 VLA 线程（10Hz 生产者）与控制线程（1000Hz 消费者）之间的
 * 无锁通信。Capacity 必须为 2 的幂以实现无分支取模。buffer 满时
 * 采用 overwrite 策略丢弃最旧的动作。
 */
#pragma once
#include <atomic>
#include <array>
#include <cstddef>
#include <optional>

/**
 * @class LockFreeRingBuffer
 * @brief 单生产者单消费者无锁环形缓冲区
 * @tparam T 元素类型
 * @tparam Capacity 缓冲区容量，必须为 2 的幂
 */
template<typename T, std::size_t Capacity>
class LockFreeRingBuffer {
static_assert((Capacity & (Capacity - 1)) == 0,
    "Capacity must be power of 2 for branchless modulo");

public:
    /**
     * @brief 默认构造，初始化 head/tail 指针为 0
     */
    LockFreeRingBuffer() : head_(0), tail_(0) {}

    /**
     * @brief 生产者调用（VLA 线程，10Hz），将 item 写入缓冲区
     *
     * 如果缓冲区已满，采用 overwrite 策略：推进 tail 丢弃最旧的动作。
     *
     * @param item 待写入的元素
     * @return 始终返回 true
     */
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

    /**
     * @brief 消费者调用（控制线程，1000Hz），弹出并返回最早写入的元素
     *
     * 如果缓冲区为空，返回 std::nullopt，调用方应保持上一个动作。
     *
     * @return 被弹出的元素，或 std::nullopt 表示缓冲区为空
     */
    std::optional<T> pop() {
        const std::size_t curr = tail_.load(std::memory_order_relaxed);
        if (curr == head_.load(std::memory_order_acquire)) {
            return std::nullopt;  // buffer空，保持上一个动作
        }
        const T item = buffer_[curr];
        tail_.store((curr + 1) & mask_, std::memory_order_release);
        return item;
    }

    /**
     * @brief 查看最新写入的动作但不弹出
     *
     * 控制回路在没有新动作时持续追踪上一目标，调用此方法即可
     * 在不改变缓冲区状态的情况下获取最新值。
     *
     * @return 最新写入的元素，或 std::nullopt 表示缓冲区为空
     */
    std::optional<T> peek_latest() const {
        const std::size_t h = head_.load(std::memory_order_acquire);
        const std::size_t t = tail_.load(std::memory_order_acquire);
        if (h == t) return std::nullopt;
        return buffer_[(h - 1) & mask_];
    }

private:
    static constexpr std::size_t mask_ = Capacity - 1;  ///< 位掩码，用于无分支取模
    std::array<T, Capacity> buffer_;                   ///< 底层存储数组
    alignas(64) std::atomic<std::size_t> head_;  ///< 写入位置（false sharing 隔离）
    alignas(64) std::atomic<std::size_t> tail_;  ///< 读取位置（false sharing 隔离）
};
