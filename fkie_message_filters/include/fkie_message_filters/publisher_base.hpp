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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_PUBLISHER_BASE_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_PUBLISHER_BASE_HPP_
#pragma once

#include "helpers/abi_namespace.hpp"
#include "helpers/signaling.hpp"
#include "types.hpp"

#include <rclcpp/node.hpp>

#include <tuple>

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

class SubscriberBase;

/** \brief Base class for ROS publishers in a filter pipeline.
 *
 * ROS subscribers and publishers can act as sources and sinks in the message filter library. This class provides some
 * basic functionality for on-demand subscriptions.
 *
 * \sa Publisher
 */
class PublisherBase
{
    friend class SubscriberBase;

public:
    virtual ~PublisherBase();
    /** \brief Check if the publisher is active.
     *
     * Returns \c true if the number of subscribers is greater than zero. The result of the function is used to
     * subscribe or unsubscribe linked subscribers on demand.
     *
     * \nothrow
     */
    virtual bool is_active() const = 0;
    /** \brief Return advertised topic name.
     *
     * \abstractthrow
     */
    virtual std::string topic() const = 0;

protected:
    /** \brief Cause all linked subscribers to subscribe or unsubscribe to their ROS topics.
     *
     * This will check the return value of is_active() to determine if the publisher is active, and then call
     * SubscriberBase::subscribe_impl() or SubscriberBase::unsubscribe_impl() accordingly.
     *
     * \implthrow
     */
    void update_subscriber_state();
    /** \brief Add a new subscriber that will be controlled by this publisher.
     *
     * \arg \c sub the subscriber
     * \return two connection objects for the signal to enable and disable the linked subscriber
     *
     * \implthrow
     */
    std::tuple<Connection, Connection> link_with_subscriber(SubscriberBase& sub);
    /** \brief Start monitoring thread for the number of subscribers.
     *
     * This is needed in ROS 2 because there is no longer a dedicated callback for created publishers.
     *
     * \nothrow
     */
    void start_monitor(const rclcpp::Node::SharedPtr& node) noexcept;
    /** \brief Shutdown monitoring thread for the number of subscribers.
     *
     * This function is called automatically when the publisher object is destroyed.
     */
    void shutdown_monitor() noexcept;

private:
    helpers::Signal<> enable_signal_, disable_signal_;
    class Monitor;
    std::shared_ptr<Monitor> monitor_;
};

FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_PUBLISHER_BASE_HPP_ */
