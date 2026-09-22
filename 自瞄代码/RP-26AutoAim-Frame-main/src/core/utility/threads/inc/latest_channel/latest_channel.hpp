#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <type_traits>

#include "communication_tool.hpp"

namespace latest_channel_detail
{
    template <typename T>
    struct is_shared_ptr : std::false_type
    {
    };

    template <typename T>
    struct is_shared_ptr<std::shared_ptr<T>> : std::true_type
    {
    };

    template <typename T>
    inline constexpr bool is_shared_ptr_v = is_shared_ptr<std::remove_cv_t<T>>::value;
}


template <typename T>
class LatestChannel : public CommunicationToolBase
{
    static_assert(!latest_channel_detail::is_shared_ptr_v<T>,
                  "LatestChannel<T> 的 T 应该是消息类型本身，不要使用 LatestChannel<std::shared_ptr<U>>；"
                  "请使用 LatestChannel<U>，publish() 可以接收 std::shared_ptr<const U>。");

public:
    using DataPtr = std::shared_ptr<const T>;
    class Publisher;
    class Subscriber;

    LatestChannel() = default;
    LatestChannel(const LatestChannel&) = delete;
    LatestChannel& operator=(const LatestChannel&) = delete;

    Subscriber create_subscriber(bool is_receive_existing_msg = false);

    void publish(const DataPtr &data)
    {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_latest_data = data;
        }

        // 向其它线程建立 Synchronizes-With 关系，保证暴露 release 之前的内存操作
        m_seq.fetch_add(1, std::memory_order_release);
        m_seq.notify_all(); // 同一个对象不可能被指令重排，
        // 不可能死锁——无论是发布线程还是订阅线程，都没在同一作用域内同时占用 m_mutex 和 futex，
        // 循环等待不可能成立
    }

    DataPtr wait_next(uint64_t &received_seq)
    {
        while (true)
        {
            // 响应某个 release，保证 acquire 之后的内存操作不会被提前执行
            uint64_t current = m_seq.load(std::memory_order_acquire);

            if (current != received_seq)
            {
                received_seq = current;

                std::lock_guard<std::mutex> lock(m_mutex);
                return m_latest_data;
            }

            // 不会因为 wait_next() 读到旧序号而错过新数据，原因是 atomic::wait 会在真正睡眠前再次检查值，
            // 即这里有一次“复检”，m_seq 比 current 更新的时候不会进入等待（自旋/休眠），
            // wait 内部有对内存的 read 的操作，肯定要用 std::memory_order_acquire
            m_seq.wait(current, std::memory_order_acquire);
        }
    }

    DataPtr try_get_new(uint64_t &seen)
    {
        uint64_t current = m_seq.load(std::memory_order_acquire);
        if (current == seen)
            return nullptr;

        seen = current;

        std::lock_guard<std::mutex> lock(m_mutex);
        return m_latest_data;
    }

    uint64_t current_seq() const
    {
        return m_seq.load(std::memory_order_acquire);
    }

private:
    mutable std::mutex m_mutex;
    DataPtr m_latest_data;
    std::atomic_uint64_t m_seq{0};
};

template <typename T>
class LatestChannel<T>::Publisher
{
public:
    using DataPtr = typename LatestChannel<T>::DataPtr;

    explicit Publisher(LatestChannel &channel)
        : m_channel(channel)
    {
    }

    void publish(const DataPtr &data)
    {
        m_channel.publish(data);
    }

private:
    LatestChannel &m_channel;
};

template <typename T>
class LatestChannel<T>::Subscriber
{
public:
    using DataPtr = typename LatestChannel<T>::DataPtr;

    explicit Subscriber(LatestChannel<T> &channel, bool is_receive_existing_msg = false)
        : m_channel(channel),
          m_received_seq(is_receive_existing_msg ? 0 : channel.current_seq())
    {
    }

    DataPtr wait_next()
    {
        return m_channel.wait_next(m_received_seq);
    }

    DataPtr try_get_new()
    {
        return m_channel.try_get_new(m_received_seq);
    }

    [[nodiscard]] uint64_t received_seq() const
    {
        return m_received_seq;
    }

private:
    LatestChannel<T> &m_channel;
    uint64_t m_received_seq;
};

template <typename T>
typename LatestChannel<T>::Subscriber LatestChannel<T>::create_subscriber(bool is_receive_existing_msg)
{
    return Subscriber(*this, is_receive_existing_msg);
}

template <typename T>
using Subscriber = typename LatestChannel<T>::Subscriber;
