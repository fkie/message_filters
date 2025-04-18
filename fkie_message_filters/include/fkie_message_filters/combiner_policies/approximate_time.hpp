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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_APPROXIMATE_TIME_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_APPROXIMATE_TIME_HPP_
#pragma once

#include "../helpers/abi_namespace.hpp"
#include "policy_base.hpp"

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <deque>
#include <mutex>

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

namespace combiner_policies
{

/** \brief Approximate time policy.
 *
 * This is a policy for the Combiner class. It will associate data from the connected sources, but unlike ExactTime, it
 * can match messages even when their ROS header timestamps do not match perfectly. If an input source is not unary,
 * only the first argument will be examined to determine the timestamp.
 *
 * The policy employs a modified version of the ROS <a
 * href="http://wiki.ros.org/message_filters/ApproximateTime">ApproximateTime</a> algorithm. Like its predecessor, the
 * algorithm is not merely applying an epsilon to account for time differences, but tries to find the best possible
 * match. Each set of grouped messages which are output by the policy, satisfies the following criteria: \li <b>Each
 * message is used only once.</b> Two sets will never share the same message, but some messages can be dropped. \li
 * <b>Messages are used in order.</b> Unlike the original algorithm, we do not guarantee that sets as a whole do not
 * overlap, but we guarantee that for each topic, messages will be used in time stamp order. We found that revising this
 * criterion improves match quality for high-frequency topics, such as stereo vision systems with depth images, in the
 * presence of transient jitter. \li <b>Sets are contiguous.</b> This means that you cannot form a valid set from the
 * dropped messages. \li <b>Sets have minimal timespan.</b> This means that the time stamp difference among messages in
 * a set cannot be smaller without violating the previous property. \li <b>The output only depends on the time
 * stamps,</b> not on the arrival time of messages. The messages have to arrive in order on each topic, but not
 * necessarily across topics, as long as the queue size is large enough to accommodate for the differences.
 *
 * Optional parameters:
 * \li <b>Set timespan</b>: the time difference of two messages in the same set will never exceed this value. By
 * default, the policy does not constrain the timespan. If you set this value, the policy will avoid "wasting" messages
 * in sets which are ultimately rejected anyway. Oftentimes, this will give you more valid sets in total.
 * \li <b>Message distance</b>: if messages of a particular topic cannot be closer together than a known interval,
 * providing this lower bound will not change the output but will allow the algorithm to conclude earlier that a given
 * set is optimal, thereby reducing output lag. The default lower bound is zero. An incorrect lower bound will result in
 * suboptimal sets being selected.
 *
 * The algorithm works as follows:
 * \li Wait until each input queue has at least one message available.
 * \li Pick the latest message among the heads of all queues, which is going to be the pivot. The pivot belongs to every
 * set that is contiguous to the previous set, so it must belong to the newly formed set.
 * \li For all other input queues except the pivot, advance to the next message until the time difference to the pivot
 * message is minimal. Proving optimality depends on the minimum message distance and the fact that on every topic,
 * messages will arrive in time stamp order. If any queue is exhausted and, at least theoretically, the next message
 * could still be closer to the pivot, wait for the next message to arrive.
 * \li Once all input queues have reached their optimum, send the resulting set. If the set violates the user-specified
 * timespan constraint, drop the pivot element instead, and restart from scratch with the remaining messages.
 * \li If messages arrive out of order (i.e. a message on a topic has a time stamp that is earlier than the previously
 * received one), all queues are flushed and the policy restarts from scratch.
 * \li If messages need to be dropped because the maximum queue size or the message age limit is exceeded, the pivot
 * element is chosen again from the new queue heads. By default, the policy will buffer arbitrary many messages for at
 * most one second.
 */
template<typename... IOs>
class ApproximateTime : public PolicyBase<IOs...>
{
public:
    template<template<typename...> class, class...>
    friend class fkie_message_filters::Combiner;
    using typename PolicyBase<IOs...>::EmitterCB;
    using typename PolicyBase<IOs...>::IncomingTuples;
    using typename PolicyBase<IOs...>::OutgoingTuple;
    /** \brief Constructor.
     *
     * \nothrow
     */
    ApproximateTime();
    /** \brief Copy constructor. */
    ApproximateTime(const ApproximateTime& other);
    /** \brief Set maximum age of any data in the queue
     *
     * This is equivalent to
     * \code
     * set_max_queue_size(0, max_age);
     * \endcode
     *
     * \arg \c max_age maximum age
     *
     * \nothrow
     */
    ApproximateTime& set_max_age(const rclcpp::Duration& max_age) noexcept;
    /** \brief Set maximum queue size.
     *
     * \arg \c queue_size maximum queue size per slot (zero means unlimited)
     * \arg \c max_age the maximum age of any data in the queue (\c none means unlimited)
     *
     * \nothrow
     */
    ApproximateTime& set_max_queue_size(std::size_t queue_size,
                                        const std::optional<rclcpp::Duration>& max_age = std::nullopt) noexcept;
    /** \brief Set maximum permissible timestamp difference of matched messages.
     *
     * \arg \c max_delta the maximum permissible timestamp difference of matched messages
     *
     * \nothrow
     */
    ApproximateTime& set_max_timespan(const rclcpp::Duration& max_delta) noexcept;
    /** \brief Set the minimum distance between consecutive messages on a source.
     *
     * If it is known in advance that messages from a certain source cannot arrive closer together
     * than \a min_dist, the policy can conclude earlier that a set of matched messages is optimal,
     * thereby reducing the introduced lag. A typical example would be a camera with fixed frame rate F,
     * where the minimum distance between consecutive messages can be assumed to be at least \c 0.5/F.
     *
     * \arg \c i the input source slot
     * \arg \c min_dist the minimum temporal distance between consecutive messages
     *
     * \nothrow
     */
    ApproximateTime& set_min_distance(std::size_t i, const rclcpp::Duration& min_dist) noexcept;

protected:
    /** \brief Input function.
     *
     * This function will be called by the Combiner class for incoming data.
     */
    template<std::size_t N>
    void add(std::unique_lock<std::mutex>&, std::tuple_element_t<N, IncomingTuples>&&);
    void reset() noexcept override;

private:
    static constexpr std::size_t NUM_SLOTS = sizeof...(IOs);
    static constexpr std::size_t UNSET = NUM_SLOTS;
    using typename PolicyBase<IOs...>::MaybeOutgoingTuples;
    using IncomingQueues = std::tuple<std::deque<helpers::io_tuple_t<IOs>>...>;
    template<std::size_t N>
    void discard_expired_at(const rclcpp::Time& cutoff) noexcept;
    void discard_expired(const rclcpp::Time& cutoff) noexcept;
    template<std::size_t N>
    void prune_queue_at(std::size_t queue_size) noexcept;
    template<std::size_t N>
    bool can_still_improve_at() noexcept;
    bool can_still_improve() noexcept;
    rclcpp::Duration pivot_timedelta(const rclcpp::Time& ts) noexcept;
    rclcpp::Duration heads_timespan() noexcept;
    bool determine_pivot() noexcept;
    void drop_pivot() noexcept;
    void emit_heads(std::unique_lock<std::mutex>&);
    std::optional<rclcpp::Duration> max_age_;
    std::size_t max_queue_size_;
    std::optional<rclcpp::Duration> max_delta_;
    std::array<rclcpp::Duration, NUM_SLOTS> min_dist_;
    std::array<rclcpp::Time, NUM_SLOTS> latest_;
    std::size_t pivot_;
    rclcpp::Time pivot_ts_;
    IncomingQueues queues_;
    MaybeOutgoingTuples heads_;
};

}  // namespace combiner_policies
FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#include "approximate_time_impl.hpp"  // IWYU pragma: keep

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_APPROXIMATE_TIME_HPP_ */
