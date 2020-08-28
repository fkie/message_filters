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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_CAMERA_PUBLISHER_H_
#define INCLUDE_FKIE_MESSAGE_FILTERS_CAMERA_PUBLISHER_H_

#include "publisher_base.h"
#include "sink.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

namespace fkie_message_filters
{

/** \brief Publish consumed data to ROS camera topics
 *
 * This is a specialized publisher that uses image_transport to publish ROS camera topics.
 * All messages which are received from the connected sources will be published on the corresponding advertised
 * ROS topics.
 *
 * Unlike regular ROS publishers, this class can be associated with one or more subscriber instances. In that case,
 * the subscribers will subscribe to their ROS topics only if the publisher is actively used. This is a convenient
 * method to save processing power if the filter pipeline is used only intermittently.
 *
 * \sa CameraSubscriber, ImagePublisher, Publisher
 */
class CameraPublisher : public PublisherBase, public Sink<sensor_msgs::ImageConstPtr, sensor_msgs::CameraInfoConstPtr>
{
public:
    /** \brief Constructs an empty publisher.
     *
     * You need to call advertise() to actually publish to a ROS topic.
     *
     * \nothrow
     */
    CameraPublisher() noexcept {}
    /** \brief Constructor that advertises the given ROS camera topic.
     *
     * The constructor calls advertise() for you.
     *
     * \nothrow
     */
    CameraPublisher (const image_transport::ImageTransport& it, const std::string& base_topic, uint32_t queue_size, bool latch = false) noexcept;
    /** \brief Check if the ROS publisher has at least one subscriber.
     *
     * \nothrow
     */
    virtual bool is_active() const noexcept override;
    /** \brief Return the configured ROS topic.
     *
     * \nothrow
     */
    virtual std::string topic() const noexcept override;
    /** \brief Advertise ROS camera topic.
     *
     * All arguments are passed to the ROS client library; see the ROS documentation for further information. Calling
     * this method will automatically unadvertise any previously advertised ROS topic.
     *
     * \arg \c it ROS image_transport instance to handle the publishing
     * \arg \c base_topic name of the ROS image topic, subject to remapping
     * \arg \c queue_size size of the ROS publishing queue
     * \arg \c latch if true, the last published message remains available for later subscribers
     *
     * \nothrow
     */
    void advertise (const image_transport::ImageTransport& it, const std::string& base_topic, uint32_t queue_size, bool latch = false) noexcept;
    /** \brief Advertise ROS camera topic with subscriber status callbacks.
     *
     * All arguments are passed to the ROS client library; see the ROS documentation for further information. Calling
     * this method will automatically unadvertise any previously advertised ROS topic.
     *
     * \arg \c it ROS image_transport instance to handle the publishing
     * \arg \c base_topic name of the ROS image topic, subject to remapping
     * \arg \c queue_size size of the ROS publishing queue
     * \arg \c image_connect_cb callback that is invoked each time a new subscriber connects to the advertised image topic
     * \arg \c image_disconnect_cb callback that is invoked each time an existing subscriber disconnects from the advertised image topic
     * \arg \c info_connect_cb callback that is invoked each time a new subscriber connects to the advertised camera info topic
     * \arg \c info_disconnect_cb callback that is invoked each time an existing subscriber disconnects from the advertised camera info topic
     * \arg \c tracked_object an associated object whose lifetime will limit the lifetimes of the advertised topics
     * \arg \c latch if true, the last published message remains available for later subscribers
     *
     * \nothrow
     */
    void advertise (const image_transport::ImageTransport& it, const std::string& base_topic, uint32_t queue_size, const image_transport::SubscriberStatusCallback& image_connect_cb, const image_transport::SubscriberStatusCallback& image_disconnect_cb = image_transport::SubscriberStatusCallback(), const ros::SubscriberStatusCallback& info_connect_cb = ros::SubscriberStatusCallback(), const ros::SubscriberStatusCallback& info_disconnect_cb = ros::SubscriberStatusCallback(), const ros::VoidPtr& tracked_object = ros::VoidPtr(), bool latch = false) noexcept;
protected:
    /** \private */
    void receive (const sensor_msgs::ImageConstPtr&, const sensor_msgs::CameraInfoConstPtr&) noexcept override;
private:
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::CameraPublisher pub_;
};

} // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_CAMERA_PUBLISHER_H_ */
