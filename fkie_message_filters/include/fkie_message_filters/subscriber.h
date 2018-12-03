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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_SUBSCRIBER_H_
#define INCLUDE_FKIE_MESSAGE_FILTERS_SUBSCRIBER_H_

#include "source.h"
#include "message_translate.h"
#include "subscriber_base.h"
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/subscribe_options.h>
#include <ros/transport_hints.h>
#include <memory>

namespace fkie_message_filters
{

/** \brief Subscribe to a ROS topic as data provider.
 *
 * This class together with the Publisher class is the generic interface between ROS and this library. All messages
 * which are received on the subscribed topic will be passed to the connected sinks for further processing.
 * For maximum flexibility, you can choose one of four ways how to pass the received message:
 *
 * \li \c Subscriber<M, RosMessageEvent> will act as a source of \c ros::MessageEvent<const M> objects (default)
 * \li \c Subscriber<M, RosMessageConstPtr> will act as a source of \c M::ConstPtr objects (as of this writing, this is a \c boost::shared_ptr<const M>)
 * \li \c Subscriber<M, RosMessageStdSharedPtr> will act as a source of \c std::shared_ptr<const M> objects
 * \li \c Subscriber<M, RosMessage> will act as a source of plain \c M objects
 *
 * Unlike regular ROS subscribers, this class can be associated with a publisher instance. In that case, the subscriber
 * will delay subscription until the publisher is actively used and will unsubscribe (and stop passing data) as soon
 * as the publisher becomes idle. This is a convenient method to save processing power if the filter pipeline is used
 * only intermittently.
 *
 * \sa CameraSubscriber, ImageSubscriber
 */
template<class M, template<typename> class Translate = RosMessageEvent>
class Subscriber : public SubscriberBase, public Source<typename Translate<M>::FilterType>
{
public:
    /** \brief Constructs an empty subscriber.
     *
     * You need to call set_subscribe_options() and either subscribe() or subscribe_on_demand() to actually subscribe to a ROS topic.
     *
     * \nothrow
     */
    Subscriber() noexcept;
    /** \brief Constructor that subscribes to the given ROS topic.
     *
     * This constructor calls set_subscribe_options() and subscribe() for you.
     *
     * \arg \c nh ROS node handle to create the ROS subscription
     * \arg \c topic name of the ROS topic, subject to remapping
     * \arg \c queue_size size of the ROS subscription queue
     * \arg \c transport_hints low-level transport hints for the ROS client library
     * \arg \c callback_queue custom ROS callback queue
     *
     * \nothrow
	 */
    Subscriber(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size, const ros::TransportHints& transport_hints = ros::TransportHints(), ros::CallbackQueueInterface* callback_queue = 0) noexcept;
    /** \brief Configure ROS topic that is to be subscribed.
     *
     * All arguments are passed to the ROS client library; see the ROS documentation for further information. Calling
     * this method will automatically unsubscribe any previously subscribed ROS topic.
     *
     * \arg \c nh ROS node handle to create the ROS subscription
     * \arg \c topic name of the ROS topic, subject to remapping
     * \arg \c queue_size size of the ROS subscription queue
     * \arg \c transport_hints low-level transport hints for the ROS client library
     * \arg \c callback_queue custom ROS callback queue
     *
     * \nothrow
     */
    void set_subscribe_options (ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size, const ros::TransportHints& transport_hints = ros::TransportHints(), ros::CallbackQueueInterface* callback_queue = 0) noexcept;
    /** \brief Convenience function to subscribe to a ROS topic.
     *
     * This function is equivalent to calling set_subscribe_options() and then subscribe().
     *
     * \arg \c nh ROS node handle to create the ROS subscription
     * \arg \c topic name of the ROS topic, subject to remapping
     * \arg \c queue_size size of the ROS subscription queue
     * \arg \c transport_hints low-level transport hints for the ROS client library
     * \arg \c callback_queue custom ROS callback queue
     *
     * \nothrow
     */
    void subscribe (ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size, const ros::TransportHints& transport_hints = ros::TransportHints(), ros::CallbackQueueInterface* callback_queue = 0) noexcept;
    using SubscriberBase::subscribe;
    using SubscriberBase::unsubscribe;
    using SubscriberBase::subscribe_on_demand;
    virtual std::string topic() const noexcept override;
protected:
    /** \brief Check if the ROS subscriber is properly configured.
     *
     * \nothrow
     */
    virtual bool is_configured() const noexcept override;
    /** \brief Create a ROS subscriber.
     *
     * \nothrow
     */
    virtual void subscribe_impl() noexcept override;
    /** \brief Shut the ROS subscriber down.
     *
     * \nothrow
     */
    virtual void unsubscribe_impl() noexcept override;
private:
    using EventType = typename Translate<M>::EventType;
    using FilterType = typename Translate<M>::FilterType;
    void cb(const EventType& event);
    ros::Subscriber sub_;
    ros::SubscribeOptions opts_;
    std::shared_ptr<ros::NodeHandle> nh_;
};

} // namespace fkie_message_filters

#include "subscriber_impl.h"

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_SUBSCRIBER_H_ */
