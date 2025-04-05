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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_SEQUENCER_IMPL_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_SEQUENCER_IMPL_HPP_
#pragma once

// IWYU pragma: private; include "sequencer.hpp"

#include "helpers/access_ros_header.hpp"
#include "helpers/tuple.hpp"
#include "sequencer.hpp"

#include <algorithm>
#include <list>

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

template<class... Inputs>
Sequencer<Inputs...>::Sequencer(const rclcpp::Duration& max_delay) noexcept
    : max_delay_(max_delay), cutoff_(0, 0, RCL_ROS_TIME)
{
}

template<class... Inputs>
void Sequencer<Inputs...>::set_max_delay(const rclcpp::Duration& max_delay) noexcept
{
    std::lock_guard<std::mutex> lock{mutex_};
    max_delay_ = max_delay;
}

template<class... Inputs>
void Sequencer<Inputs...>::receive(helpers::argument_t<Inputs>... in)
{
    std::lock_guard<std::mutex> lock{mutex_};
    rclcpp::Time stamp = helpers::access_ros_header_stamp(std::get<0>(std::forward_as_tuple(in...)));
    cutoff_ = std::max(cutoff_, stamp - max_delay_);
    if (stamp < cutoff_)
        return;
    queue_.emplace(stamp, QueueElement(helpers::maybe_move(in)...));
    auto ub = queue_.upper_bound(cutoff_);
    std::list<QueueElement> out;
    std::for_each(queue_.begin(), ub, [&out](auto& q) { out.push_back(helpers::maybe_move(q.second)); });
    queue_.erase(queue_.begin(), ub);
    /* Unlike most other filters, we do not release the mutex for send(), because we want to ensure strict temporal
     * order. */
    for (QueueElement& e : out)
    {
        helpers::index_apply<sizeof...(Inputs)>([this, &e](auto... Is)
                                                { this->send(std::get<Is>(helpers::maybe_move(e))...); });
    }
}

template<class... Inputs>
void Sequencer<Inputs...>::flush()
{
    std::list<QueueElement> out;
    std::for_each(queue_.begin(), queue_.end(), [&out](auto& q) { out.push_back(helpers::maybe_move(q.second)); });
    queue_.clear();
    /* Unlike most other filters, we do not release the mutex for send(), because we want to ensure strict temporal
     * order. */
    for (QueueElement& e : out)
    {
        cutoff_ = helpers::access_ros_header_stamp(std::get<0>(e));
        helpers::index_apply<sizeof...(Inputs)>([this, &e](auto... Is)
                                                { this->send(std::get<Is>(helpers::maybe_move(e))...); });
    }
}

template<class... Inputs>
void Sequencer<Inputs...>::reset() noexcept
{
    std::lock_guard<std::mutex> lock{mutex_};
    queue_.clear();
    cutoff_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_SEQUENCER_IMPL_HPP_ */
