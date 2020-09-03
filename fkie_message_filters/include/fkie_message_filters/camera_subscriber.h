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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_CAMERA_SUBSCRIBER_H_
#define INCLUDE_FKIE_MESSAGE_FILTERS_CAMERA_SUBSCRIBER_H_

#include "subscriber_base.h"
#include "source.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

namespace fkie_message_filters
{

/** \brief Subscribe to ROS camera topics as data provider.
 *
 * This is a specialized subscriber that uses image_transport to subscribe to ROS camera topics.
 * All messages which are received on the subscribed topics will be passed to the connected sinks for further processing.
 *
 * Unlike regular ROS subscribers, this class can be associated with a publisher instance. In that case, the subscriber
 * will delay subscription until the publisher is actively used and will unsubscribe (and stop passing data) as soon
 * as the publisher becomes idle. This is a convenient method to save processing power if the filter pipeline is used
 * only intermittently.
 *
 * \sa CameraPublisher, ImageSubscriber, Subscriber
 */
class CameraSubscriber : public SubscriberBase, public Source<sensor_msgs::ImageConstPtr, sensor_msgs::CameraInfoConstPtr>
{
public:
    /** \brief Constructs an empty subscriber.
     *
     * You need to call set_subscribe_options() and either subscribe() or subscribe_on_demand() to actually subscribe to a ROS topic.
     */
    CameraSubscriber() noexcept {}
    /** \brief Constructor that subscribes to the given ROS camera base topic.
     *
     * This constructor calls set_subscribe_options() and subscribe() for you.
     *
     * \arg \c it ROS image_transport instance to handle the subscription
     * \arg \c base_topic name of the ROS camera base topic, subject to remapping
     * \arg \c queue_size size of the ROS subscription queue
     * \arg \c transport_hints transport hints for the ROS image_transport framework
     *
     * \nothrow
     */
    CameraSubscriber(const image_transport::ImageTransport& it, const std::string& base_topic, uint32_t queue_size, const image_transport::TransportHints& transport_hints = image_transport::TransportHints()) noexcept;
    /** \brief Configure ROS topic that is to be subscribed.
     *
     * All arguments are passed to the ROS client library; see the ROS documentation for further information. Calling
     * this method will automatically unsubscribe any previously subscribed ROS topic.
     *
     * \arg \c it ROS image_transport instance to handle the subscription
     * \arg \c base_topic name of the ROS camera base topic, subject to remapping
     * \arg \c queue_size size of the ROS subscription queue
     * \arg \c transport_hints transport hints for the ROS image_transport framework
     *
     * \nothrow
     */
    void set_subscribe_options(const image_transport::ImageTransport& it, const std::string& base_topic, uint32_t queue_size, const image_transport::TransportHints& transport_hints = image_transport::TransportHints()) noexcept;
    /** \brief Convenience function to subscribe to a ROS topic.
     *
     * This function is equivalent to calling set_subscribe_options() and then subscribe().
     *
     *
     * \arg \c it ROS image_transport instance to handle the subscription
     * \arg \c base_topic name of the ROS camera base topic, subject to remapping
     * \arg \c queue_size size of the ROS subscription queue
     * \arg \c transport_hints transport hints for the ROS image_transport framework
     *
     * \nothrow
     */
    void subscribe(const image_transport::ImageTransport& it, const std::string& base_topic, uint32_t queue_size, const image_transport::TransportHints& transport_hints = image_transport::TransportHints()) noexcept;
    using SubscriberBase::subscribe;
    using SubscriberBase::unsubscribe;
    using SubscriberBase::subscribe_on_demand;
    /** \brief Return the configured ROS topic.
     *
     * \nothrow
     */
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
    void cb(const sensor_msgs::ImageConstPtr&, const sensor_msgs::CameraInfoConstPtr&);
    std::shared_ptr<image_transport::ImageTransport> it_;
    std::string base_topic_;
    uint32_t queue_size_;
    image_transport::TransportHints hints_;
    image_transport::CameraSubscriber sub_;
};

} // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_CAMERA_SUBSCRIBER_H_ */
