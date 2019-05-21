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

#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_EXACT_TIME_H_
#define INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_EXACT_TIME_H_

#include "policy_base.h"
#include <map>
#include <mutex>
#include <ros/duration.h>

namespace fkie_message_filters
{
namespace combiner_policies
{

/** \brief Exact time policy.
 *
 * This is a policy for the Combiner class. It will associate data from the connected sources when their ROS header
 * timestamp matches exactly. Each input source can itself be a tuple, but only the first element of each tuple will
 * be examined to determine the timestamp. It must have an accessible ROS header, which is determined using the
 * ros::message_traits template.
 *
 * The policy will discard unmatched data which exceeds configurable age limit or overflows the maximum queue size.
 * The resulting timestamps will be strictly increasing if at least one of the inputs receives messages in correct temporal order.
 * Whenever matched data is emitted, all queued inputs with older timestamps will be discarded.
 *
 * The filter will not output any data at all if the time lag between two inputs is larger than the maximum permissible age,
 * or if the time lag requires more messages to be buffered than the maximum queue size permits. By default, the filter
 * will buffer arbitrary many messages for at most one second.
 */
template<typename... IOs>
class ExactTime : public PolicyBase<IOs...>
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
    ExactTime();
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
    ExactTime& set_max_age(const ros::Duration& max_age) noexcept;
    /** \brief Set maximum queue size.
     *
     * \arg \c queue_size maximum queue size per slot (zero means unlimited)
     * \arg \c max_age the maximum age of any data in the queue
     *
     * \nothrow
     */
    ExactTime& set_max_queue_size(std::size_t queue_size, const boost::optional<ros::Duration>& max_age = boost::none) noexcept;
protected:
    /** \brief Input function.
     *
     * This function will be called by the Combiner class for incoming data.
     */
    template<std::size_t N>
    void add(std::unique_lock<std::mutex>&, const std::tuple_element_t<N, IncomingTuples>&);
    void reset() noexcept override;
private:
    using typename PolicyBase<IOs...>::MaybeOutgoingTuples;
    using IncomingQueues = std::tuple<std::map<ros::Time, helpers::io_tuple_t<IOs>>...>;
    MaybeOutgoingTuples try_assemble_output(const ros::Time& time, bool& complete) noexcept;
    IncomingQueues queues_;
    boost::optional<ros::Duration> max_age_;
    std::size_t max_queue_size_;
};

} // namespace combiner_policies
} // namespace fkie_message_filters

#include "exact_time_impl.h"

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_FIFO_H_ */
