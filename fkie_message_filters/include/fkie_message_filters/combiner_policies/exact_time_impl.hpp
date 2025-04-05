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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_EXACT_TIME_IMPL_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_EXACT_TIME_IMPL_HPP_
#pragma once

// IWYU pragma: private; include "exact_time.hpp"

#include "../helpers/access_ros_header.hpp"
#include "../helpers/scoped_unlock.hpp"
#include "../helpers/tuple.hpp"
#include "../logging.hpp"
#include "exact_time.hpp"

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

namespace combiner_policies
{

template<typename... IOs>
ExactTime<IOs...>::ExactTime() : max_age_(rclcpp::Duration(1, 0)), max_queue_size_(0)
{
}

template<typename... IOs>
ExactTime<IOs...>::ExactTime(const ExactTime& other)
    : PolicyBase<IOs...>(other), max_age_(other.max_age_), max_queue_size_(other.max_queue_size_)
{
    /* The copy constructor deliberately avoids copying the incoming queue, because
     * a) the connection to any Combiner instance is broken by the copying anyway and
     * b) it avoids issues with move-only types (std::deque<T> is copyable iff T is copyable)
     */
}

template<typename... IOs>
ExactTime<IOs...>& ExactTime<IOs...>::set_max_age(const rclcpp::Duration& max_age) noexcept
{
    max_queue_size_ = 0;
    max_age_ = max_age;
    return *this;
}

template<typename... IOs>
ExactTime<IOs...>& ExactTime<IOs...>::set_max_queue_size(std::size_t queue_size,
                                                         const std::optional<rclcpp::Duration>& max_age) noexcept
{
    max_queue_size_ = queue_size;
    max_age_ = max_age;
    return *this;
}

template<typename... IOs>
template<std::size_t N>
void ExactTime<IOs...>::add(std::unique_lock<std::mutex>& lock, std::tuple_element_t<N, IncomingTuples>&& in)
{
    rclcpp::Time stamp = helpers::access_ros_header_stamp(std::get<0>(in));
    if (!std::get<N>(queues_).try_emplace(stamp, std::move(in)).second)
    {
        FKIE_MF_WARN("message with repeating time stamp " << std::fixed << std::setprecision(9) << stamp.seconds()
                                                          << " is being dropped");
        return;
    }
    bool complete;
    MaybeOutgoingTuples out = try_assemble_output(stamp, complete);
    if (max_age_ || complete)
    {
        rclcpp::Time cutoff = complete ? stamp : stamp - *max_age_;
        helpers::for_each_apply<sizeof...(IOs)>(
            [this, &cutoff](auto I)
            {
                auto& queue = std::get<I>(this->queues_);
                auto ub = queue.upper_bound(cutoff);
                queue.erase(queue.begin(), ub);
            });
    }
    if (max_queue_size_ > 0)
    {
        auto& queue = std::get<N>(queues_);
        if (queue.size() > max_queue_size_)
            queue.erase(queue.begin()); /* can be at most one element */
    }
    if (complete)
    {
        helpers::index_apply<sizeof...(IOs)>(
            [this, &out, &lock](auto... Is)
            {
                auto unlock = helpers::with_scoped_unlock(lock);
                OutgoingTuple e{std::tuple_cat(std::move(*std::get<Is>(out))...)};
                this->emit(e);
            });
    }
}

template<typename... IOs>
typename ExactTime<IOs...>::MaybeOutgoingTuples ExactTime<IOs...>::try_assemble_output(const rclcpp::Time& time,
                                                                                       bool& complete) noexcept
{
    MaybeOutgoingTuples result;
    complete = helpers::all_true<sizeof...(IOs)>(
        [this, &time](auto I)
        {
            auto& queue = std::get<I>(this->queues_);
            return queue.find(time) != queue.end();
        });
    if (!complete)
        return result;
    helpers::for_each_apply<sizeof...(IOs)>(
        [this, &time, &result](auto I)
        {
            auto& queue = std::get<I>(this->queues_);
            auto it = queue.find(time);
            std::get<I>(result) = std::move(it->second);
            queue.erase(it);
        });
    return result;
}

template<typename... IOs>
void ExactTime<IOs...>::reset() noexcept
{
    helpers::for_each_apply<sizeof...(IOs)>(
        [this](auto I)
        {
            auto& queue = std::get<I>(this->queues_);
            queue.clear();
        });
}

}  // namespace combiner_policies
FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_EXACT_TIME_IMPL_HPP_ */
