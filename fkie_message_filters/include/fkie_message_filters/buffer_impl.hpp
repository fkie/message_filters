/****************************************************************************
 *
 * fkie_message_filters
 * Copyright © 2018-2025 Fraunhofer FKIE
 * Author: Timo Röhling
 * SPDX-License-Identifier: Apache-2.0
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
#include "version.hpp"
#ifndef FKIE_MF_IGNORE_RCLCPP_OK
#    include <rclcpp/utilities.hpp>
#endif

#include <rclcpp/node_interfaces/get_node_base_interface.hpp>
#include <rclcpp/node_interfaces/get_node_waitables_interface.hpp>
#include <rclcpp/waitable.hpp>

#include <condition_variable>
#include <deque>
#include <mutex>

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

template<class... Inputs>
struct Buffer<Inputs...>::Impl
{
    class BufferCB : public rclcpp::Waitable
    {
    public:
        using SharedPtr = std::shared_ptr<BufferCB>;

        explicit BufferCB(Impl* impl) : impl_(impl), cond_(rcl_get_zero_initialized_guard_condition())
        {
            rclcpp::Context::SharedPtr ctx = rclcpp::contexts::get_global_default_context();
            rcl_guard_condition_options_t options = rcl_guard_condition_get_default_options();
            if (rcl_guard_condition_init(&cond_, ctx->get_rcl_context().get(), options) != RCL_RET_OK)
                throw std::runtime_error("failed to initialize rcl guard condition");
        }

        ~BufferCB()
        {
            if (rcl_guard_condition_fini(&cond_) != RCL_RET_OK)
            { /* ignore */
            }
        }

        std::size_t get_number_of_ready_guard_conditions() override
        {
            return 1;
        }

        void trigger()
        {
            if (rcl_trigger_guard_condition(&cond_) != RCL_RET_OK)
                throw std::runtime_error("failed to trigger rcl guard condition");
        }

#if FKIE_MF_RCLCPP_VERSION >= FKIE_MF_VERSION_TUPLE(29, 4, 0)
        void add_to_wait_set(rcl_wait_set_t& wait_set) override
        {
            if (rcl_wait_set_add_guard_condition(&wait_set, &cond_, nullptr) != RCL_RET_OK)
                throw std::runtime_error("failed to add rcl guard condition to wait set");
        }

        bool is_ready(const rcl_wait_set_t&) override
        {
            std::lock_guard<std::mutex> lock{impl_->mutex_};
            return !impl_->queue_.empty();
        }

        void execute(const std::shared_ptr<void>&) override
        {
            std::unique_lock<std::mutex> lock{impl_->mutex_};
            impl_->parent_->process_some(lock);
        }

        std::vector<std::shared_ptr<rclcpp::TimerBase>> get_timers() const override
        {
            return {};
        }
#else
        void add_to_wait_set(rcl_wait_set_t* wait_set) override
        {
            if (rcl_wait_set_add_guard_condition(wait_set, &cond_, nullptr) != RCL_RET_OK)
                throw std::runtime_error("failed to add rcl guard condition to wait set");
        }

        bool is_ready(rcl_wait_set_t*) override
        {
            std::lock_guard<std::mutex> lock{impl_->mutex_};
            return !impl_->queue_.empty();
        }

        void execute(std::shared_ptr<void>&) override
        {
            std::unique_lock<std::mutex> lock{impl_->mutex_};
            impl_->parent_->process_some(lock);
        }
#endif

        std::shared_ptr<void> take_data() override
        {
            return nullptr;
        }

        std::shared_ptr<void> take_data_by_entity_id(std::size_t id) override
        {
            return nullptr;
        }

        void set_on_ready_callback(std::function<void(std::size_t, int)>) override {}

        void clear_on_ready_callback() override {}

    private:
        Impl* impl_;
        rcl_guard_condition_t cond_;
    };

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
        if (callback_)
            callback_->trigger();
    }

    template<class NodeT>
    void set_node(std::unique_lock<std::mutex>& lock, NodeT&& node)
    {
        if constexpr (!std::is_same_v<NodeT, std::nullptr_t>)
        {
            if (node)
            {
                auto node_base = rclcpp::node_interfaces::get_node_base_interface(node);
                auto node_waitable = rclcpp::node_interfaces::get_node_waitables_interface(node);
                if (node_waitable_ != node_waitable || node_base_ != node_base)
                {
                    callback_.reset();
                    callback_group_.reset();
                    node_base_ = node_base;
                    node_waitable_ = node_waitable;
                    callback_group_ = node_base_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                    callback_ = std::make_shared<BufferCB>(this);
                    node_waitable_->add_waitable(callback_, callback_group_);
                }
                return;
            }
        }
        callback_.reset();
        callback_group_.reset();
        node_base_.reset();
        node_waitable_.reset();
    }

    Buffer* parent_;
    BufferPolicy policy_;
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
    rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitable_;
    typename BufferCB::SharedPtr callback_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
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
template<class NodeT>
Buffer<Inputs...>::Buffer(NodeT&& node, std::size_t max_queue_size) noexcept
    : impl_(std::make_shared<Impl>(this, BufferPolicy::Queue, max_queue_size))
{
    std::unique_lock<std::mutex> lock{impl_->mutex_};
    impl_->template set_node<NodeT>(lock, std::forward<NodeT&&>(node));
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
            lock.unlock();
            break;
        case BufferPolicy::Queue:
            if (max_queue_size > 0)
                impl_->adjust_capacity(max_queue_size);
            lock.unlock();
            break;
        case BufferPolicy::Passthru:
            process_some(lock);
            // lock is unlocked now
            break;
    }
    impl_->cond_.notify_all();
}

template<class... Inputs>
template<class NodeT>
void Buffer<Inputs...>::set_node(NodeT&& node)
{
    std::unique_lock<std::mutex> lock{impl_->mutex_};
    impl_->template set_node<NodeT>(lock, std::forward<NodeT&&>(node));
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
