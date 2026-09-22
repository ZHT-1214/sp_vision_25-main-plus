#pragma once

#include <condition_variable>
#include <mutex>
#include <optional>
#include <utility>

#include "communication_tool.hpp"


template<typename T>
class LatestBuffer : public CommunicationToolBase
{
public:
    class Publisher;
    class Subscriber;

    LatestBuffer() = default;
    LatestBuffer(const LatestBuffer&) = delete;
    LatestBuffer& operator=(const LatestBuffer&) = delete;

    template<typename U>
    void push(U&& value)
    {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_value.emplace(std::forward<U>(value));
        }
        m_cv.notify_one();
    }

    T wait_pop()
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_cv.wait(lock, [this]{
            return m_value.has_value();
        });

        T value = std::move(*m_value);
        m_value.reset();
        return value;
    }

    std::optional<T> try_pop()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_value.has_value())
            return std::nullopt;

        T value = std::move(*m_value);
        m_value.reset();
        return value;
    }

    void clear()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_value.reset();
    }

    [[nodiscard]] bool has_value() const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_value.has_value();
    }

private:
    mutable std::mutex m_mutex;
    std::condition_variable m_cv;
    std::optional<T> m_value;
};

template<typename T>
class LatestBuffer<T>::Publisher
{
public:
    explicit Publisher(LatestBuffer &buffer)
        : m_buffer(buffer)
    {
    }

    template<typename U>
    void push(U&& value)
    {
        m_buffer.push(std::forward<U>(value));
    }

private:
    LatestBuffer &m_buffer;
};

template<typename T>
class LatestBuffer<T>::Subscriber
{
public:
    explicit Subscriber(LatestBuffer &buffer)
        : m_buffer(buffer)
    {
    }

    T wait_pop()
    {
        return m_buffer.wait_pop();
    }

    std::optional<T> try_pop()
    {
        return m_buffer.try_pop();
    }

    void clear()
    {
        m_buffer.clear();
    }

    [[nodiscard]] bool has_value() const
    {
        return m_buffer.has_value();
    }

private:
    LatestBuffer &m_buffer;
};
