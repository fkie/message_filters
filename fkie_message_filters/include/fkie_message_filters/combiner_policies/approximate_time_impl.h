/****************************************************************************
 *
 * fkie_message_filters
 * Copyright © 2018 Fraunhofer FKIE
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

#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_APPROXIMATE_TIME_IMPL_H_
#define INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_APPROXIMATE_TIME_IMPL_H_

#include "approximate_time.h"
#include "../helpers/access_ros_header.h"
#include "../helpers/scoped_unlock.h"
#include "../helpers/tuple.h"
#include <ros/console.h>

namespace fkie_message_filters
{
namespace combiner_policies
{

template<typename... IOs>
ApproximateTime<IOs...>::ApproximateTime()
: max_age_(ros::Duration(1, 0)), max_queue_size_(0), max_delta_(boost::none), pivot_(UNSET)
{
}

template<typename... IOs>
ApproximateTime<IOs...>& ApproximateTime<IOs...>::set_max_age(const ros::Duration& max_age) noexcept
{
    max_queue_size_ = 0;
    max_age_ = max_age;
    return *this;
}

template<typename... IOs>
ApproximateTime<IOs...>& ApproximateTime<IOs...>::set_max_queue_size(std::size_t queue_size, const boost::optional<ros::Duration>& max_age) noexcept
{
    max_queue_size_ = queue_size;
    max_age_ = max_age;
    return *this;
}

template<typename... IOs>
ApproximateTime<IOs...>& ApproximateTime<IOs...>::set_max_timespan(const ros::Duration& max_delta) noexcept
{
    max_delta_ = max_delta;
    return *this;
}

template<typename... IOs>
ApproximateTime<IOs...>& ApproximateTime<IOs...>::set_min_distance(std::size_t i, const ros::Duration& min_dist) noexcept
{
    min_dist_[i] = min_dist;
    return *this;
}

template<typename... IOs>
template<std::size_t N>
void ApproximateTime<IOs...>::add(std::unique_lock<std::mutex>& lock, const std::tuple_element_t<N, IncomingTuples>& in)
{
    ros::Time stamp = helpers::access_ros_header_stamp(std::get<0>(in));
    if (max_age_)
    {
        ros::Time cutoff = stamp - *max_age_;
        discard_expired(cutoff);
    }
    auto& head = std::get<N>(heads_);
    auto& queue = std::get<N>(queues_);
    /* First, make sure that all slots have in-order arrival of messages */
    if (head)
    {
        ros::Time latest = helpers::access_ros_header_stamp(std::get<0>(queue.empty() ? *head : queue.back()));
        if (stamp < latest)
        {
            ROS_ERROR_STREAM_NAMED("Combiner<ApproximateTime>", "message with older time stamp " << stamp << "<" << latest << " received, resetting filter");
            reset();
        }
        else
        if (latest + min_dist_[N] > stamp)
        {
            ROS_WARN_STREAM_NAMED("Combiner<ApproximateTime>", "new message arrived sooner than anticipated: time stamp " << stamp << "<" << latest + min_dist_[N]);
        }
    }
    /* Add data to slot */
    if (!head) head = in; else queue.push_back(in);
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
            if (!determine_pivot()) return; /* Head is still incomplete, thus pivot undetermined */
        }
        if (pivot_ != N)
        {
            /* The pivot slot will never advance, because it must be part of the next set.
             * For all other slots, we try to improve and see if we reached an optimum.
             * If we could still improve with a later message, we stop here for now. */
            if (try_to_improve_at<N>()) return;
        }
        /* The current slot cannot improve, but maybe some other slot can */
        if (can_still_improve()) return;
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
    MaybeOutgoingTuples out = heads_;
    helpers::for_each_apply<NUM_SLOTS>(
        [this](auto I)
        {
            auto& head = std::get<I>(this->heads_);
            auto& queue = std::get<I>(this->queues_);
            if (!queue.empty())
            {
                head = queue.front();
                queue.pop_front();
            }
            else
            {
                head.reset();
            }
        }
    );
    pivot_ = UNSET;
    auto unlock = helpers::with_scoped_unlock(lock);
    helpers::index_apply<NUM_SLOTS>(
        [&](auto... Is)
        {
            this->emit(std::tuple_cat(*std::get<Is>(out)...));
        }
    );
}

template<typename... IOs>
template<std::size_t N>
bool ApproximateTime<IOs...>::try_to_improve_at() noexcept
{
    // Check we can improve the current set by advancing in slot N
    auto& head = std::get<N>(heads_);
    auto& queue = std::get<N>(queues_);
    ros::Time stamp = helpers::access_ros_header_stamp(std::get<0>(*head));
    while (!queue.empty())
    {
        ros::Time next_stamp = helpers::access_ros_header_stamp(std::get<0>(queue.front()));
        if (pivot_timedelta(next_stamp) < pivot_timedelta(stamp))
        {
            /* The next message is closer to the pivot element */
            head = queue.front();
            stamp = next_stamp;
            queue.pop_front();
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
                if (this->try_to_improve_at<I>()) result = true;
            }
        }
    );
    return result;
}

template<typename... IOs>
template<std::size_t N>
void ApproximateTime<IOs...>::discard_expired_at(const ros::Time& cutoff) noexcept
{
    auto& head = std::get<N>(heads_);
    auto& queue = std::get<N>(queues_);
    if (head)
    {
        ros::Time stamp = helpers::access_ros_header_stamp(std::get<0>(*head));
        if (stamp < cutoff)
        {
            head.reset();
            pivot_ = UNSET;
        }
    }
    while (!queue.empty())
    {
        ros::Time stamp = helpers::access_ros_header_stamp(std::get<0>(queue.front()));
        if (stamp <= cutoff) queue.pop_front(); else break;
    }
    if (!head && !queue.empty())
    {
        head = queue.front();
        queue.pop_front();
    }
}

template<typename... IOs>
void ApproximateTime<IOs...>::discard_expired(const ros::Time& cutoff) noexcept
{
    helpers::for_each_apply<NUM_SLOTS>([this, cutoff](auto I) { this->discard_expired_at<I>(cutoff); });
}

template<typename... IOs>
template<std::size_t N>
void ApproximateTime<IOs...>::prune_queue_at(std::size_t queue_size) noexcept
{
    auto& queue = std::get<N>(queues_);
    if (queue.size() <= queue_size) return;
    while (queue.size() > queue_size + 1) queue.pop_front();
    auto& head = std::get<N>(heads_);
    head = queue.front();
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
        }
    );
    pivot_ = UNSET;
}

template<typename... IOs>
ros::Duration ApproximateTime<IOs...>::heads_timespan() noexcept
{
    ros::Time first_ts, last_ts;
    helpers::for_each_apply<NUM_SLOTS>(
        [&](auto I)
        {
            auto& head = std::get<I>(this->heads_);
            if (head)
            {
                ros::Time stamp = helpers::access_ros_header_stamp(std::get<0>(*head));
                if (first_ts.isZero() || stamp < first_ts)
                {
                    first_ts = stamp;
                }
                if (last_ts.isZero() || stamp > last_ts)
                {
                    last_ts = stamp;
                }
            }
        }
    );
    return last_ts - first_ts;
}

template<typename... IOs>
ros::Duration ApproximateTime<IOs...>::pivot_timedelta(const ros::Time& ts) noexcept
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
                head = queue.front();
                queue.pop_front();
            }
            else
                head.reset();
        }
    );
    pivot_ = UNSET;
}

template<typename... IOs>
bool ApproximateTime<IOs...>::determine_pivot() noexcept
{
    assert(pivot_ == UNSET);
    pivot_ts_ = ros::Time();
    bool ok = true;
    helpers::for_each_apply<NUM_SLOTS>(
        [&](auto I)
        {
            auto& head = std::get<I>(this->heads_);
            if (head)
            {
                ros::Time stamp = helpers::access_ros_header_stamp(std::get<0>(*head));
                if (stamp > pivot_ts_)
                {
                    pivot_ts_ = stamp;
                    if (ok) pivot_ = I;
                }
            }
            else
            {
                pivot_ = UNSET;
                ok = false;
            }
        }
    );
    return pivot_ != UNSET;
}

} // namespace combiner_policies
} // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_APPROXIMATE_TIME_IMPL_H_ */
