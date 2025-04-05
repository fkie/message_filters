/****************************************************************************
 *
 * fkie_message_filters
 * Copyright © 2018-2025 Fraunhofer FKIE
 * Author: Timo Röhling
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_BUFFER_IMPL_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_BUFFER_IMPL_HPP_
#pragma once

// IWYU pragma: private; include "buffer.hpp"

#ifndef FKIE_MF_IGNORE_RCLCPP_OK
#    ifdef FKIE_MESSAGE_FILTERS_IGNORE_ROS_OK
#        define FKIE_MF_IGNORE_RCLCPP_OK FKIE_MESSAGE_FILTERS_IGNORE_ROS_OK
#    endif
#endif

#include "buffer.hpp"
#include "helpers/tuple.hpp"
#ifndef FKIE_MF_IGNORE_RCLCPP_OK
#    include <rclcpp/utilities.hpp>
#endif

#include <condition_variable>
#include <deque>
#include <mutex>

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

template<class... Inputs>
struct Buffer<Inputs...>::Impl
{
    Impl(Buffer* parent, BufferPolicy policy, std::size_t max_queue_size) noexcept
        : parent_(parent), policy_(policy), max_queue_size_(max_queue_size)
    {
    }

    bool wait_for_queue_element(std::unique_lock<std::mutex>& lock) noexcept
    {
#ifndef FKIE_MF_IGNORE_RCLCPP_OK
        using namespace std::chrono_literals;
        while (rclcpp::ok() && policy_ == BufferPolicy::Queue && queue_.empty())
        {
            cond_.wait_for(lock, 100ms);
        }
        return rclcpp::ok() && !queue_.empty();
#else
        while (policy_ == BufferPolicy::Queue && queue_.empty())
            cond_.wait(lock);
        return !queue_.empty();
#endif
    }
    template<class Rep, class Period>
    bool wait_for_queue_element(std::unique_lock<std::mutex>& lock,
                                const std::chrono::duration<Rep, Period>& timeout) noexcept
    {
        std::chrono::system_clock::time_point deadline = std::chrono::system_clock::now() + timeout;
#ifndef FKIE_MF_IGNORE_RCLCPP_OK
        using namespace std::chrono_literals;
        while (rclcpp::ok() && policy_ == BufferPolicy::Queue && queue_.empty())
        {
            std::chrono::system_clock::duration remaining = deadline - std::chrono::system_clock::now();
            if (remaining > 100ms)
                cond_.wait_for(lock, 100ms);
            else
                cond_.wait_until(lock, deadline);
        }
        return rclcpp::ok() && !queue_.empty();
#else
        while (policy_ == BufferPolicy::Queue && queue_.empty())
            cond_.wait_until(lock, deadline);
        return !queue_.empty();
#endif
    }

    void rclcpp_timer_callback()
    {
        std::unique_lock<std::mutex> lock{mutex_};
        if (!queue_.empty())
        {
            QueueElement e{std::move(queue_.front())};
            queue_.pop_front();
            if (queue_.empty())
                timer_.reset();
            lock.unlock();
            parent_->send_queue_element(e);
            lock.lock();
        }
        else
        {
            timer_.reset();
        }
    }

    void arm_rclcpp_timer(std::unique_lock<std::mutex>& lock)
    {
        using namespace std::chrono_literals;
        if (node_ && !timer_)
        {
            timer_ = node_->create_wall_timer(
                0ns, static_cast<rclcpp::VoidCallbackType>([this]() { this->rclcpp_timer_callback(); }),
                callback_group_);
        }
    }

    void adjust_capacity(std::size_t max_queue_size)
    {
        std::lock_guard<std::mutex> lock{mutex_};
        max_queue_size_ = max_queue_size;
        while (queue_.size() > max_queue_size)
            queue_.pop_front();
    }

    void insert_queue_element(std::unique_lock<std::mutex>& lock, QueueElement& e)
    {
        queue_.push_back(std::move(e));
        while (queue_.size() > max_queue_size_)
            queue_.pop_front();
        arm_rclcpp_timer(lock);
    }

    void set_node(std::unique_lock<std::mutex>& lock, const rclcpp::Node::SharedPtr& node)
    {
        if (node_ != node)
        {
            timer_.reset();
            callback_group_.reset();
            node_ = node;
            if (node_)
            {
                callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                if (!queue_.empty())
                    arm_rclcpp_timer(lock);
            }
        }
    }

    Buffer* parent_;
    BufferPolicy policy_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::WallTimer<rclcpp::VoidCallbackType>::SharedPtr timer_;
    std::deque<QueueElement> queue_;
    std::size_t max_queue_size_;
    std::mutex mutex_;
    std::condition_variable cond_;
};

template<class... Inputs>
Buffer<Inputs...>::Buffer(BufferPolicy policy, std::size_t max_queue_size) noexcept
    : impl_(std::make_shared<Impl>(this, policy, max_queue_size))
{
    // impl_->queue_.set_capacity(max_queue_size);
}

template<class... Inputs>
Buffer<Inputs...>::Buffer(const rclcpp::Node::SharedPtr& node, std::size_t max_queue_size) noexcept
    : impl_(std::make_shared<Impl>(this, BufferPolicy::Queue, max_queue_size))
{
    std::unique_lock<std::mutex> lock{impl_->mutex_};
    impl_->set_node(lock, node);
}

template<class... Inputs>
Buffer<Inputs...>::~Buffer()
{
    set_policy(BufferPolicy::Discard);
}

template<class... Inputs>
void Buffer<Inputs...>::set_policy(BufferPolicy policy, std::size_t max_queue_size)
{
    std::unique_lock<std::mutex> lock{impl_->mutex_};
    impl_->policy_ = policy;
    switch (policy)
    {
        case BufferPolicy::Discard:
            impl_->queue_.clear();
            impl_->timer_.reset();
            lock.unlock();
            break;
        case BufferPolicy::Queue:
            if (max_queue_size > 0)
                impl_->adjust_capacity(max_queue_size);
            lock.unlock();
            break;
        case BufferPolicy::Passthru:
            impl_->timer_.reset();
            process_some(lock);
            // lock is unlocked now
            break;
    }
    impl_->cond_.notify_all();
}

template<class... Inputs>
void Buffer<Inputs...>::set_node(const rclcpp::Node::SharedPtr& node) noexcept
{
    std::unique_lock<std::mutex> lock{impl_->mutex_};
    impl_->set_node(lock, node);
}

template<class... Inputs>
void Buffer<Inputs...>::spin_once()
{
    std::unique_lock<std::mutex> lock{impl_->mutex_};
    process_some(lock);
}

template<class... Inputs>
void Buffer<Inputs...>::reset() noexcept
{
    std::lock_guard<std::mutex> lock{impl_->mutex_};
    impl_->queue_.clear();
    impl_->timer_.reset();
}

template<class... Inputs>
void Buffer<Inputs...>::process_some(std::unique_lock<std::mutex>& lock)
{
    std::vector<QueueElement> tmp;
    tmp.reserve(impl_->queue_.size());
    for (QueueElement& e : impl_->queue_)
    {
        tmp.push_back(std::move(e));
    }
    impl_->queue_.clear();
    lock.unlock();
    for (QueueElement& e : tmp)
    {
        send_queue_element(e);
    }
    // lock stays unlocked
}

template<class... Inputs>
bool Buffer<Inputs...>::has_some() const noexcept
{
    std::lock_guard<std::mutex> lock{impl_->mutex_};
    return !impl_->queue_.empty();
}

template<class... Inputs>
bool Buffer<Inputs...>::wait() noexcept
{
    std::unique_lock<std::mutex> lock{impl_->mutex_};
    return impl_->wait_for_queue_element(lock);
}

template<class... Inputs>
template<class Rep, class Period>
bool Buffer<Inputs...>::wait_for(const std::chrono::duration<Rep, Period>& timeout) noexcept
{
    std::unique_lock<std::mutex> lock{impl_->mutex_};
    return impl_->wait_for_queue_element(lock, timeout);
}

template<class... Inputs>
bool Buffer<Inputs...>::process_one()
{
    std::unique_lock<std::mutex> lock{impl_->mutex_};
    if (impl_->wait_for_queue_element(lock))
    {
        QueueElement e{std::move(impl_->queue_.front())};
        impl_->queue_.pop_front();
        lock.unlock();
        send_queue_element(e);
        return true;
    }
    return false;
}

template<class... Inputs>
template<class Rep, class Period>
bool Buffer<Inputs...>::process_one(const std::chrono::duration<Rep, Period>& timeout)
{
    std::unique_lock<std::mutex> lock{impl_->mutex_};
    if (impl_->wait_for_queue_element(lock, timeout))
    {
        QueueElement e{std::move(impl_->queue_.front())};
        impl_->queue_.pop_front();
        lock.unlock();
        send_queue_element(e);
        return true;
    }
    return false;
}

template<class... Inputs>
void Buffer<Inputs...>::receive(helpers::argument_t<Inputs>... in)
{
    std::unique_lock<std::mutex> lock{impl_->mutex_};
    switch (impl_->policy_)
    {
        case BufferPolicy::Discard:
            break;
        case BufferPolicy::Queue:
        {
            QueueElement e{helpers::maybe_move(in)...};
            impl_->insert_queue_element(lock, e);
            lock.unlock();
            impl_->cond_.notify_one();
        }
        break;
        case BufferPolicy::Passthru:
            lock.unlock();
            this->send(helpers::maybe_move(in)...);
            break;
    }
}

template<class... Inputs>
void Buffer<Inputs...>::spin()
{
    std::unique_lock<std::mutex> lock{impl_->mutex_};
    while (impl_->wait_for_queue_element(lock))
    {
        QueueElement e{std::move(impl_->queue_.front())};
        impl_->queue_.pop_front();
        lock.unlock();
        send_queue_element(e);
        lock.lock();
    }
}

template<class... Inputs>
void Buffer<Inputs...>::send_queue_element(QueueElement& e)
{
    helpers::index_apply<sizeof...(Inputs)>([this, &e](auto... is)
                                            { this->send(helpers::maybe_move(std::get<is>(e))...); });
}

FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_BUFFER_IMPL_HPP_ */
