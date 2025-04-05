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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_SUBSCRIBER_BASE_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_SUBSCRIBER_BASE_HPP_
#pragma once

#include "helpers/abi_namespace.hpp"
#include "types.hpp"

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

class PublisherBase;

/** \brief Base class for ROS subscribers in a filter pipeline.
 *
 * ROS subscribers and publishers can act as sources and sinks in the message filter library. This class provides
 * some basic functionality for on-demand subscriptions.
 *
 * \sa Subscriber
 */
class SubscriberBase
{
    friend class PublisherBase;

public:
    virtual ~SubscriberBase();
    /** \brief Subscribe to the configured ROS topic.
     *
     * This method does nothing if no ROS topic was configured or if the subscriber is subscribed already.
     * Cancels the effect of subscribe_on_demand(), i.e. the subscriber will remain subscribed permanently.
     *
     * \implthrow
     */
    virtual void subscribe();
    /** \brief Unsubscribe from the configured ROS topic.
     *
     * You can call subscribe() afterwards to re-subscribe to the ROS topic. This method does nothing if the subscriber
     * is not subscribed to any ROS topic. Cancels the effect of subscribe_on_demand(), i.e. the subscriber will
     * remain unsubscribed permanently.
     *
     * \implthrow
     */
    virtual void unsubscribe();
    /** \brief Subscribe to the configured ROS topic whenever the given publisher is active.
     *
     * This method does nothing if no ROS topic was configured. Otherwise, it will immediately subscribe or
     * unsubscribe depending on the publisher's current state. If the publisher becomes active or inactive, the
     * subscriber's state will update accordingly.
     *
     * \arg \c pub publisher
     *
     * \implthrow
     */
    virtual void subscribe_on_demand(PublisherBase& pub);
    /** \brief Return the subscribed topic name.
     *
     * \abstractthrow
     */
    virtual std::string topic() const = 0;

protected:
    /** \brief Check if the subscriber is properly configured.
     *
     * This virtual method must be overridden in derived classes.
     * \retval true if subscribe_impl() or unsubscribe_impl() may be called and have all the information
     *              they need to subscribe or unsubscribe, respectively.
     * \retval false otherwise
     *
     * \abstractthrow
     */
    virtual bool is_configured() const = 0;
    /** \brief Implement how to subscribe to the configured ROS topic.
     *
     * This virtual method must be overridden in derived classes to actually subscribe to a topic. It should do
     * nothing if it is subscribed to a topic already.
     *
     * \abstractthrow
     */
    virtual void subscribe_impl() = 0;
    /** \brief Implement how to unsubscribe from the configured ROS topic.
     *
     * This virtual method must be overridden in derived classes to actually unsubscribe from a topic. It should
     * do nothing if it is not subscribed in the first place.
     *
     * \abstractthrow
     */
    virtual void unsubscribe_impl() = 0;
    /** \brief Add self to the list of subscribers which are controlled by a publisher.
     *
     * A subscriber can be linked with one publisher only. Any previously linked publisher is unlinked first.
     *
     * \arg \c pub publisher
     *
     * \implthrow
     * \sa PublisherBase::link_with_subscriber()
     */
    void link_with_publisher(PublisherBase& pub);
    /** \brief Remove self from the list of subscribers which are controlled by a publisher.
     *
     * This will not affect the current subscription state. It will only prevent further updates from
     * the previously linked publisher.
     *
     * \nothrow
     */
    void unlink_from_publisher();

private:
    Connection conn1_, conn2_;
};

FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_SUBSCRIBER_BASE_HPP_ */
