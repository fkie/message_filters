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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_APPROXIMATE_TIME_IMPL_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_APPROXIMATE_TIME_IMPL_HPP_
#pragma once

// IWYU pragma: private; include "approximate_time.hpp"

#include "../helpers/access_ros_header.hpp"
#include "../helpers/scoped_unlock.hpp"
#include "../helpers/tuple.hpp"
#include "../logging.hpp"
#include "approximate_time.hpp"

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

namespace combiner_policies
{

template<typename... IOs>
ApproximateTime<IOs...>::ApproximateTime()
    : max_age_(rclcpp::Duration(1, 0)), max_queue_size_(0), max_delta_(std::nullopt),
      min_dist_{(static_cast<void>(typeid(IOs)), rclcpp::Duration(0, 0))...}, pivot_(UNSET)
{
}

template<typename... IOs>
ApproximateTime<IOs...>::ApproximateTime(const ApproximateTime& other)
    : PolicyBase<IOs...>(other), max_age_(other.max_age_), max_queue_size_(other.max_queue_size_),
      max_delta_(other.max_delta_), min_dist_(other.min_dist_), pivot_(UNSET)
{
    /* The copy constructor deliberately avoids copying the incoming queue and related members, because
     * a) the connection to any Combiner instance is broken by the copying anyway and
     * b) it avoids issues with move-only types (std::deque<T> is copyable iff T is copyable)
     */
}

template<typename... IOs>
ApproximateTime<IOs...>& ApproximateTime<IOs...>::set_max_age(const rclcpp::Duration& max_age) noexcept
{
    max_queue_size_ = 0;
    max_age_ = max_age;
    return *this;
}

template<typename... IOs>
ApproximateTime<IOs...>&
ApproximateTime<IOs...>::set_max_queue_size(std::size_t queue_size,
                                            const std::optional<rclcpp::Duration>& max_age) noexcept
{
    max_queue_size_ = queue_size;
    max_age_ = max_age;
    return *this;
}

template<typename... IOs>
ApproximateTime<IOs...>& ApproximateTime<IOs...>::set_max_timespan(const rclcpp::Duration& max_delta) noexcept
{
    max_delta_ = max_delta;
    return *this;
}

template<typename... IOs>
ApproximateTime<IOs...>& ApproximateTime<IOs...>::set_min_distance(std::size_t i,
                                                                   const rclcpp::Duration& min_dist) noexcept
{
    min_dist_[i] = min_dist;
    return *this;
}

template<typename... IOs>
template<std::size_t N>
void ApproximateTime<IOs...>::add(std::unique_lock<std::mutex>& lock, std::tuple_element_t<N, IncomingTuples>&& in)
{
    rclcpp::Time stamp = helpers::access_ros_header_stamp(std::get<0>(in));
    if (max_age_)
    {
        rclcpp::Time cutoff = stamp - *max_age_;
        discard_expired(cutoff);
    }
    auto& head = std::get<N>(heads_);
    auto& queue = std::get<N>(queues_);
    /* First, make sure that all slots have in-order arrival of messages */
    if (latest_[N].nanoseconds())
    {
        if (stamp < latest_[N])
        {
            FKIE_MF_ERROR("message with earlier time stamp " << std::fixed << std::setprecision(9) << stamp.seconds()
                                                             << " received (latest is " << latest_[N].seconds()
                                                             << "), resetting filter");
            reset();
        }
        else if (latest_[N] + min_dist_[N] > stamp)
        {
            FKIE_MF_WARN("new message arrived sooner than anticipated: time stamp "
                         << std::fixed << std::setprecision(9) << stamp.seconds() << " is earlier than latest "
                         << latest_[N].seconds() << " + " << min_dist_[N].seconds() << " = "
                         << (latest_[N] + min_dist_[N]).seconds());
        }
    }
    latest_[N] = stamp;
    /* Add data to slot */
    if (!head)
        head = std::move(in);
    else
        queue.push_back(std::move(in));
    /* Enforce queue size limit */
    if (max_queue_size_ > 0)
    {
        prune_queue_at<N>(max_queue_size_);
    }
    while (true)
    {
        /* The pivot is the last head slot that fills after the previous set was emitted.
         * The pivot will always be a member of the next emitted set unless the timespan constraint is
         * violated. */
        if (pivot_ == UNSET)
        {
            if (!determine_pivot())
                return; /* Head is still incomplete, thus pivot undetermined */
        }
        if (pivot_ != N)
        {
            /* The pivot slot will never advance, because it must be part of the next set.
             * For all other slots, we try to improve and see if we reached an optimum.
             * If we could still improve with a later message, we stop here for now. */
            if (can_still_improve_at<N>())
                return;
        }
        /* The current slot cannot improve, but maybe some other slot can */
        if (can_still_improve())
            return;
        /* We have reached the best possible set with the current pivot */
        if (max_delta_)
        {
            /* Optionally, check if the total timespan of the current set is acceptable */
            if (heads_timespan() > *max_delta_)
            {
                drop_pivot(); /* This pivot is not good enough, maybe the next one? */
                continue;
            }
        }
        emit_heads(lock);
    }
}

template<typename... IOs>
void ApproximateTime<IOs...>::emit_heads(std::unique_lock<std::mutex>& lock)
{
    MaybeOutgoingTuples out = std::move(heads_);
    helpers::for_each_apply<NUM_SLOTS>(
        [this](auto I)
        {
            auto& head = std::get<I>(this->heads_);
            auto& queue = std::get<I>(this->queues_);
            if (!queue.empty())
            {
                head = std::move(queue.front());
                queue.pop_front();
            }
            else
            {
                head.reset();
            }
        });
    pivot_ = UNSET;
    auto unlock = helpers::with_scoped_unlock(lock);
    helpers::index_apply<NUM_SLOTS>(
        [&](auto... Is)
        {
            OutgoingTuple e{std::tuple_cat(std::move(*std::get<Is>(out))...)};
            this->emit(e);
        });
}

template<typename... IOs>
template<std::size_t N>
bool ApproximateTime<IOs...>::can_still_improve_at() noexcept
{
    // Check we can improve the current set by advancing in slot N
    auto& head = std::get<N>(heads_);
    auto& queue = std::get<N>(queues_);
    rclcpp::Time stamp = helpers::access_ros_header_stamp(std::get<0>(*head));
    while (!queue.empty())
    {
        rclcpp::Time next_stamp = helpers::access_ros_header_stamp(std::get<0>(queue.front()));
        if (pivot_timedelta(next_stamp) < pivot_timedelta(stamp))
        {
            /* The next message is closer to the pivot element */
            head = std::move(queue.front());
            queue.pop_front();
            stamp = next_stamp;
        }
        else
            return false; /* cannot improve further with this pivot */
    }
    /* We ran out of messages in this queue, so we do not know for sure whether or not the next
     * message would be a better fit. However, we can only improve if the current message is still
     * earlier than the pivot and if the next message could arrive early enough to be closer to the
     * pivot than the current one.
     */
    return stamp < pivot_ts_ && pivot_timedelta(stamp + min_dist_[N]) <= pivot_timedelta(stamp);
}

template<typename... IOs>
bool ApproximateTime<IOs...>::can_still_improve() noexcept
{
    bool result = false;
    helpers::for_each_apply<NUM_SLOTS>(
        [&](auto I)
        {
            if (I != pivot_)
            {
                if (this->can_still_improve_at<I>())
                    result = true;
            }
        });
    return result;
}

template<typename... IOs>
template<std::size_t N>
void ApproximateTime<IOs...>::discard_expired_at(const rclcpp::Time& cutoff) noexcept
{
    auto& head = std::get<N>(heads_);
    auto& queue = std::get<N>(queues_);
    if (head)
    {
        rclcpp::Time stamp = helpers::access_ros_header_stamp(std::get<0>(*head));
        if (stamp < cutoff)
        {
            head.reset();
            pivot_ = UNSET;
        }
    }
    while (!queue.empty())
    {
        rclcpp::Time stamp = helpers::access_ros_header_stamp(std::get<0>(queue.front()));
        if (stamp <= cutoff)
            queue.pop_front();
        else
            break;
    }
    if (!head && !queue.empty())
    {
        head = std::move(queue.front());
        queue.pop_front();
    }
}

template<typename... IOs>
void ApproximateTime<IOs...>::discard_expired(const rclcpp::Time& cutoff) noexcept
{
    helpers::for_each_apply<NUM_SLOTS>([this, cutoff](auto I) { this->discard_expired_at<I>(cutoff); });
}

template<typename... IOs>
template<std::size_t N>
void ApproximateTime<IOs...>::prune_queue_at(std::size_t queue_size) noexcept
{
    auto& queue = std::get<N>(queues_);
    if (queue.size() <= queue_size)
        return;
    while (queue.size() > queue_size + 1)
        queue.pop_front();
    auto& head = std::get<N>(heads_);
    head = std::move(queue.front());
    queue.pop_front();
    pivot_ = UNSET;
}

template<typename... IOs>
void ApproximateTime<IOs...>::reset() noexcept
{
    helpers::for_each_apply<NUM_SLOTS>(
        [this](auto I)
        {
            std::get<I>(heads_).reset();
            std::get<I>(queues_).clear();
        });
    std::fill(latest_.begin(), latest_.end(), rclcpp::Time());
    pivot_ = UNSET;
}

template<typename... IOs>
rclcpp::Duration ApproximateTime<IOs...>::heads_timespan() noexcept
{
    rclcpp::Time first_ts, last_ts;
    helpers::for_each_apply<NUM_SLOTS>(
        [&](auto I)
        {
            auto& head = std::get<I>(this->heads_);
            if (head)
            {
                rclcpp::Time stamp = helpers::access_ros_header_stamp(std::get<0>(*head));
                if (first_ts.nanoseconds() == 0 || stamp < first_ts)
                {
                    first_ts = stamp;
                }
                if (last_ts.nanoseconds() == 0 || stamp > last_ts)
                {
                    last_ts = stamp;
                }
            }
        });
    return last_ts - first_ts;
}

template<typename... IOs>
rclcpp::Duration ApproximateTime<IOs...>::pivot_timedelta(const rclcpp::Time& ts) noexcept
{
    /* The pivot timedelta is the absolute temporal distance from the pivot. */
    return ts < pivot_ts_ ? pivot_ts_ - ts : ts - pivot_ts_;
}

template<typename... IOs>
void ApproximateTime<IOs...>::drop_pivot() noexcept
{
    helpers::select_apply<NUM_SLOTS>(pivot_,
                                     [this](auto I)
                                     {
                                         auto& head = std::get<I>(this->heads_);
                                         auto& queue = std::get<I>(this->queues_);
                                         if (!queue.empty())
                                         {
                                             head = std::move(queue.front());
                                             queue.pop_front();
                                         }
                                         else
                                             head.reset();
                                     });
    pivot_ = UNSET;
}

template<typename... IOs>
bool ApproximateTime<IOs...>::determine_pivot() noexcept
{
    pivot_ = UNSET;
    pivot_ts_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    bool ok = true;
    helpers::for_each_apply<NUM_SLOTS>(
        [&](auto I)
        {
            auto& head = std::get<I>(this->heads_);
            if (head)
            {
                rclcpp::Time stamp = helpers::access_ros_header_stamp(std::get<0>(*head));
                if (stamp > pivot_ts_)
                {
                    pivot_ts_ = stamp;
                    if (ok)
                        pivot_ = I;
                }
            }
            else
            {
                pivot_ = UNSET;
                ok = false;
            }
        });
    return pivot_ != UNSET;
}

}  // namespace combiner_policies
FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_APPROXIMATE_TIME_IMPL_HPP_ */
