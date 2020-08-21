/****************************************************************************
 *
 * fkie_message_filters
 * Copyright © 2018-2020 Fraunhofer FKIE
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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_BUFFER_IMPL_H_
#define INCLUDE_FKIE_MESSAGE_FILTERS_BUFFER_IMPL_H_

#include "buffer.h"
#include "helpers/tuple.h"
#include <ros/init.h>

namespace fkie_message_filters
{

template<class... Inputs>
struct Buffer<Inputs...>::Impl
{
    Impl(Buffer* parent, BufferPolicy policy) noexcept
    : parent_(parent), policy_(policy), epoch_(0)
    {}

    bool wait_for_queue_element(std::unique_lock<std::mutex>& lock) noexcept
    {
#ifndef FKIE_MESSAGE_FILTERS_IGNORE_ROS_OK
        while (ros::ok() && policy_ == BufferPolicy::Queue && queue_.empty())
        {
            cond_.wait_for(lock, std::chrono::milliseconds(100));
        }
        return ros::ok() && !queue_.empty();
#else
        while (policy_ == BufferPolicy::Queue && queue_.empty()) cond_.wait(lock);
        return !queue_.empty();
#endif
    }
    template<class Rep, class Period>
    bool wait_for_queue_element(std::unique_lock<std::mutex>& lock, const std::chrono::duration<Rep, Period>& timeout) noexcept
    {
        std::chrono::system_clock::time_point deadline = std::chrono::system_clock::now() + timeout;
    #ifndef FKIE_MESSAGE_FILTERS_IGNORE_ROS_OK
        while (ros::ok() && policy_ == BufferPolicy::Queue && queue_.empty())
        {
            std::chrono::system_clock::duration remaining = deadline - std::chrono::system_clock::now();
            if (remaining > std::chrono::milliseconds(100))
                cond_.wait_for(lock, std::chrono::milliseconds(100));
            else
                cond_.wait_until(lock, deadline);
        }
        return ros::ok() && !queue_.empty();
    #else
        while (policy_ == BufferPolicy::Queue && queue_.empty()) cond_.wait_until(lock, deadline);
        return !queue_.empty();
    #endif
    }
    Buffer* parent_;
    BufferPolicy policy_;
    boost::circular_buffer<QueueElement> queue_;
    std::mutex mutex_;
    std::condition_variable cond_;
    std::size_t epoch_;
};

template<class... Inputs>
class Buffer<Inputs...>::RosCB : public ros::CallbackInterface
{
public:
    RosCB(const std::weak_ptr<Impl>& parent, std::size_t epoch) noexcept
    : parent_(parent), epoch_(epoch) {}

    CallResult call() noexcept override
    {
        std::shared_ptr<Impl> impl = parent_.lock();
        if (impl)
        {
            std::unique_lock<std::mutex> lock(impl->mutex_);
            if (!impl->queue_.empty() && epoch_ == impl->epoch_)
            {
                QueueElement e{impl->queue_.front()};
                impl->queue_.pop_front();
                lock.unlock();
                impl->parent_->send_queue_element(e);
            }
        }
        return Success;
    }
private:
    std::weak_ptr<Impl> parent_;
    std::size_t epoch_;
};

template<class... Inputs>
Buffer<Inputs...>::Buffer(BufferPolicy policy, std::size_t max_queue_size, ros::CallbackQueueInterface* cbq) noexcept
: impl_(std::make_shared<Impl>(this, policy)), cbq_(cbq)
{
    impl_->queue_.set_capacity(max_queue_size);
}

template<class... Inputs>
Buffer<Inputs...>::Buffer(const ros::NodeHandle& nh, std::size_t max_queue_size) noexcept
: impl_(std::make_shared<Impl>(this, BufferPolicy::Queue)), cbq_(nh.getCallbackQueue())
{
    impl_->queue_.set_capacity(max_queue_size);
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
            ++impl_->epoch_;
            if (cbq_) cbq_->removeByID(reinterpret_cast<uint64_t>(this));
            lock.unlock();
            break;
        case BufferPolicy::Queue:
            if (max_queue_size > 0) impl_->queue_.set_capacity(max_queue_size);
            lock.unlock();
            break;
        case BufferPolicy::Passthru:
            ++impl_->epoch_;
            if (cbq_) cbq_->removeByID(reinterpret_cast<uint64_t>(this));
            process_some(lock);
            // lock is unlocked now
            break;
    }
    impl_->cond_.notify_all();
}

template<class... Inputs>
void Buffer<Inputs...>::set_callback_queue(ros::CallbackQueueInterface* cbq) noexcept
{
    std::lock_guard<std::mutex> lock{impl_->mutex_};
    cbq_ = cbq;
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
    ++impl_->epoch_;
    if (cbq_) cbq_->removeByID(reinterpret_cast<uint64_t>(this));
}

template<class... Inputs>
void Buffer<Inputs...>::process_some(std::unique_lock<std::mutex>& lock)
{
    std::vector<QueueElement> tmp;
    tmp.reserve(impl_->queue_.size());
    std::copy(impl_->queue_.begin(), impl_->queue_.end(), std::back_inserter(tmp));
    impl_->queue_.clear();
    lock.unlock();
    for (const QueueElement& e : tmp)
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
        QueueElement e{impl_->queue_.front()};
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
        QueueElement e{impl_->queue_.front()};
        impl_->queue_.pop_front();
        lock.unlock();
        send_queue_element(e);
        return true;
    }
    return false;
}

template<class... Inputs>
void Buffer<Inputs...>::receive(const Inputs&... in)
{
    std::unique_lock<std::mutex> lock{impl_->mutex_};
    switch (impl_->policy_)
    {
        case BufferPolicy::Discard:
            break;
        case BufferPolicy::Queue:
            impl_->queue_.push_back(QueueElement(in...));
            if (cbq_) cbq_->addCallback(ros::CallbackInterfacePtr(new RosCB(impl_, impl_->epoch_)), reinterpret_cast<uint64_t>(this));
            lock.unlock();
            impl_->cond_.notify_one();
            break;
        case BufferPolicy::Passthru:
            lock.unlock();
            this->send(in...);
            break;
    }
}

template<class... Inputs>
void Buffer<Inputs...>::spin()
{
    std::unique_lock<std::mutex> lock{impl_->mutex_};
    while (impl_->wait_for_queue_element(lock))
    {
        QueueElement e{impl_->queue_.front()};
        impl_->queue_.pop_front();
        lock.unlock();
        send_queue_element(e);
        lock.lock();
    }
}

template<class... Inputs>
void Buffer<Inputs...>::send_queue_element(const QueueElement& e)
{
    helpers::index_apply<sizeof...(Inputs)>(
        [&](auto... is)
        {
            this->send(std::get<is>(e)...);
        }
    );
}

} // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_BUFFER_IMPL_H_ */
