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

#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_APPROXIMATE_TIME_H_
#define INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_APPROXIMATE_TIME_H_

#include "policy_base.h"
#include <deque>
#include <mutex>
#include <ros/duration.h>

namespace fkie_message_filters
{
namespace combiner_policies
{

/** \brief Approximate time policy.
 *
 * This is a policy for the Combiner class. It will associate data from the connected sources, but unlike ExactTime,
 * it can match messages even when their ROS header timestamps are different. If an input has multiple data elements,
 * the first one is considered only. The examined data types must have an accessible ROS header, which is determined
 * with the ros::message_traits templates.
 *
 * The policy employs a simplified version of the
 * ROS <a href="http://wiki.ros.org/message_filters/ApproximateTime">ApproximateTime</a> algorithm.
 * A set of messages is formed by taking one item from each input. The output of this policy satisfies the following
 * properties:
 * \li <b>Each message is used only once.</b> Two sets will never share the same message, but some messages can be dropped.
 * \li <b>Messages are used in order.</b> Unlike the original ROS implementation, we do not guarantee that sets
 * as a whole do not overlap, but we guarantee that for each topic, messages will be used in time stamp order.
 * \li <b>Sets are contiguous.</b> This means that you cannot form a valid set from the dropped messages.
 * \li <b>Sets have minimal timespan.</b> This means that the time stamp difference among
 * messages in a set cannot be smaller without violating the previous property.
 * \li <b>The output only depends on the time stamps,</b> not on the arrival time of messages. The messages have to
 * arrive in order on each topic, but not necessarily across topics, as long as the queue size is large enough to
 * accommodate for the differences.
 *
 * Optional parameters:
 * \li <b>Set timespan</b>: the latest message in a set will have a time stamp that is at most this long after the
 * first message in a set. By default, the policy does not constrain the timespan.
 * \li <b>Message distance</b>: if messages of a particular topic cannot be closer together than a known interval,
 * providing this lower bound will not change the output but will allow the algorithm to conclude earlier that a given
 * set is optimal, thereby reducing output lag. The default lower bound is zero. An incorrect lower bound will result
 * in suboptimal sets being selected.
 *
 * The algorithm works as follows:
 * \li Wait until each input queue has at least one message available.
 * \li Pick the latest message among the heads of all queues, which is going to be the <i>pivot</i>. The pivot belongs
 * to every set that is contiguous to the previous set, so it must belong to the next set.
 * \li For all other input queues except the pivot, advance to the next message until the timedelta to the pivot message
 * is minimal. Proving optimality depends on the minimum message distance and the fact that on every topic, messages will arrive in
 * time stamp order. If the queue is exhausted and, at least theoretically, the next message could still be closer to
 * the pivot, wait for the next message to arrive.
 * \li Once all input queues have reached their optimum, send the resulting set. If the set violates the user-specified
 * timespan constraint, drop the pivot element instead, and restart from scratch with the remaining messages.
 *
 * If messages arrive out of order (i.e. a message on a topic has a time stamp that is earlier than the previously
 * received one), all queues are flushed and the policy restarts from scratch. If messages need to be dropped because the
 * maximum queue size or the message age limit is exceeded, the pivot element is chosen again from the new queue heads.
 */
template<typename... IOs>
class ApproximateTime : public PolicyBase<IOs...>
{
public:
    template<template<typename...> class, class...> friend class Combiner;
    using typename PolicyBase<IOs...>::EmitterCB;
    using typename PolicyBase<IOs...>::IncomingTuples;
    using typename PolicyBase<IOs...>::OutgoingTuple;
    /** \brief Constructor.
     *
     * \nothrow
     */
    ApproximateTime();
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
    ApproximateTime& set_max_age(const ros::Duration& max_age) noexcept;
    /** \brief Set maximum queue size.
     *
     * \arg \c queue_size maximum queue size per slot (zero means unlimited)
     * \arg \c max_age the maximum age of any data in the queue
     *
     * \nothrow
     */
    ApproximateTime& set_max_queue_size (std::size_t queue_size, const boost::optional<ros::Duration>& max_age = boost::none) noexcept;
    /** \brief Set maximum permissible timestamp difference of matched messages.
     *
     * \arg \c max_delta the maximum permissible timestamp difference of matched messages
     *
     * \nothrow
     */
    ApproximateTime& set_max_timespan (const ros::Duration& max_delta) noexcept;
    /** \brief Set the minimum distance between consecutive messages on a source.
     *
     * If it is known in advance that messages from a certain source cannot arrive closer together
     * than \a min_dist, the policy can conclude earlier that a set of matched messages is optimal,
     * thereby reducing the introduced lag. A typical example would be a camera with fixed frame rate F,
     * where the minimum distance betwen consecutive messages can be assumed to be at least \c 0.5/F.
     *
     * \arg \c i the input source slot
     * \arg \c min_dist the minimum temporal distance between consecutive messages
     *
     * \nothrow
     */
    ApproximateTime& set_min_distance(std::size_t i, const ros::Duration& min_dist) noexcept;
protected:
    /** \brief Input function.
     *
     * This function will be called by the Combiner class for incoming data.
     */
    template<std::size_t N>
    void add(std::unique_lock<std::mutex>&, const std::tuple_element_t<N, IncomingTuples>&);
    void reset() noexcept override;
private:
    static constexpr std::size_t NUM_SLOTS = sizeof...(IOs);
    static constexpr std::size_t UNSET = NUM_SLOTS;
    using typename PolicyBase<IOs...>::MaybeOutgoingTuples;
    using IncomingQueues = std::tuple<std::deque<helpers::io_tuple_t<IOs>>...>;
    template<std::size_t N>
    void discard_expired_at(const ros::Time& cutoff) noexcept;
    void discard_expired(const ros::Time& cutoff) noexcept;
    template<std::size_t N>
    void prune_queue_at(std::size_t queue_size) noexcept;
    template<std::size_t N>
    bool try_to_improve_at() noexcept;
    bool can_still_improve() noexcept;
    ros::Duration pivot_timedelta(const ros::Time& ts) noexcept;
    ros::Duration heads_timespan() noexcept;
    bool determine_pivot() noexcept;
    void drop_pivot() noexcept;
    void emit_heads(std::unique_lock<std::mutex>&);
    boost::optional<ros::Duration> max_age_;
    std::size_t max_queue_size_;
    boost::optional<ros::Duration> max_delta_;
    std::size_t pivot_;
    ros::Time pivot_ts_;
    std::array<ros::Duration, NUM_SLOTS> min_dist_;
    IncomingQueues queues_;
    MaybeOutgoingTuples heads_;
};

} // namespace combiner_policies
} // namespace fkie_message_filters

#include "approximate_time_impl.h"

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_APPROXIMATE_TIME_H_ */
