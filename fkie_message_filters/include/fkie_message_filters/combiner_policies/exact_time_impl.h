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

#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_EXACT_TIME_IMPL_H_
#define INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_EXACT_TIME_IMPL_H_

#include "exact_time.h"
#include "../helpers/access_ros_header.h"
#include "../helpers/tuple.h"
#include "../helpers/scoped_unlock.h"
#include <ros/console.h>

namespace fkie_message_filters
{
namespace combiner_policies
{

template<typename... IOs>
ExactTime<IOs...>::ExactTime()
: max_age_(ros::Duration(1, 0)), max_queue_size_(0)
{
}

template<typename... IOs>
ExactTime<IOs...>& ExactTime<IOs...>::set_max_age (const ros::Duration& max_age) noexcept
{
    max_queue_size_ = 0;
    max_age_ = max_age;
    return *this;
}

template<typename... IOs>
ExactTime<IOs...>& ExactTime<IOs...>::set_max_queue_size (std::size_t queue_size, const boost::optional<ros::Duration>& max_age) noexcept
{
    max_queue_size_ = queue_size;
    max_age_ = max_age;
    return *this;
}

template<typename... IOs>
template<std::size_t N>
void ExactTime<IOs...>::add(std::unique_lock<std::mutex>& lock, const std::tuple_element_t<N, IncomingTuples>& in)
{
    ros::Time stamp = helpers::access_ros_header_stamp(std::get<0>(in));
    if (!std::get<N>(queues_).emplace(stamp, in).second)
    {
        ROS_WARN_STREAM_NAMED("Combiner<ExactTime>", "message with repeating time stamp " << stamp << " is dropped");
        return;
    }
    bool complete;
    MaybeOutgoingTuples out = try_assemble_output(stamp, complete);
    if (max_age_ || complete)
    {
        ros::Time cutoff = complete ? stamp : stamp - *max_age_;
        helpers::for_each_apply<sizeof...(IOs)>(
            [this, &cutoff](auto I)
            {
                auto& queue = std::get<I>(this->queues_);
                auto ub = queue.upper_bound(cutoff);
                queue.erase(queue.begin(), ub);
            }
        );
    }
    if (max_queue_size_ > 0)
    {
        auto& queue = std::get<N>(queues_);
        if (queue.size() > max_queue_size_) queue.erase(queue.begin()); /* can be at most one element */
    }
    if (complete)
    {
        helpers::index_apply<sizeof...(IOs)>(
            [this, &out, &lock](auto... Is)
            {
                auto unlock = helpers::with_scoped_unlock(lock);
                this->emit(std::tuple_cat(*std::get<Is>(out)...));
            }
        );
    }
}

template<typename... IOs>
typename ExactTime<IOs...>::MaybeOutgoingTuples ExactTime<IOs...>::try_assemble_output(const ros::Time& time, bool& complete) noexcept
{
    complete = true;
    MaybeOutgoingTuples result;
    helpers::for_each_apply<sizeof...(IOs)>(
        [&](auto I)
        {
            if (complete)
            {
                auto& queue = std::get<I>(this->queues_);
                auto it = queue.find(time);
                if (it != queue.end())
                {
                    std::get<I>(result) = it->second;
                }
                else
                {
                    complete = false;
                }
            }
        }
    );
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
        }
    );
}

} // namespace combiner_policies
} // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_EXACT_TIME_IMPL_H_ */
