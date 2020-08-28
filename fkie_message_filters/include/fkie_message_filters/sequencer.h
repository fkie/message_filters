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

#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_SEQUENCER_H_
#define INCLUDE_FKIE_MESSAGE_FILTERS_SEQUENCER_H_

#include "filter.h"
#include <ros/time.h>
#include <ros/duration.h>
#include <map>

namespace fkie_message_filters
{

/** \brief Enforce correct temporal order.
 *
 * This filter sorts incoming messages according to their header timestamp, and forwards them in order. It needs
 * the expected maximum delay by which messages might arrive out of order, and forwarded messages will be out of date
 * by at least this delay. However, all forwarded messages will be guaranteed to be in temporal order, and no message
 * will be dropped unless it arrives with a larger delay, compared to the most recently received message.
 *
 * If the filter input is not unary, only the first argument is examined, which must have
 * an accessible ROS header as determined by the \c ros::message_traits template.
 */
template<class... Inputs>
class Sequencer : public Filter<IO<Inputs...>, IO<Inputs...>>
{
public:
    /** \brief Constructor.
     *
     * \arg \c max_delay the maximum delay of any message
     *
     * \nothrow
     */
    explicit Sequencer (const ros::Duration& max_delay = ros::Duration(1, 0)) noexcept;
    /** \brief Modify maximum delay.
     *
     * New messages older than the most recently forwarded one will continue to be dropped, even if their delay is smaller
     * than \a max_delay. In other words, a call to this method will never violate the temporal order constraint.
     *
     * \arg \c max_delay the maximum delay of any message
     *
     * \nothrow
     */
    void set_max_delay (const ros::Duration& max_delay) noexcept;
    /** \brief Flush the message queue.
     *
     * This will forward all messages in the queue, regardless of their age. Afterwards, new messages which are older
     * than the most recently forwarded one will be dropped, even if their delay is smaller than the configured threshold.
     * In other words, a call to this method will never violate the temporal order constraint.
     *
     * \filterthrow
     */
    void flush();
    void reset() noexcept override;
protected:
    void receive (const Inputs&... in) override;
private:
    using QueueElement = std::tuple<Inputs...>;
    using Queue = std::multimap<ros::Time, QueueElement>;
    std::mutex mutex_;
    ros::Duration max_delay_;
    ros::Time cutoff_;
    Queue queue_;
};

template<class... Inputs>
class Sequencer<IO<Inputs...>> : public Sequencer<Inputs...>
{
public:
    explicit Sequencer(const ros::Duration& max_delay = ros::Duration(1, 0)) noexcept
    : Sequencer<Inputs...>(max_delay) {}
};

} // namespace fkie_message_filters

#include "sequencer_impl.h"

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_SEQUENCER_H_ */
