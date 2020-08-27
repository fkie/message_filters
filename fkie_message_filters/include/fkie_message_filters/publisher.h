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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_PUBLISHER_H_
#define INCLUDE_FKIE_MESSAGE_FILTERS_PUBLISHER_H_


#include "source.h"
#include "publisher_base.h"
#include "message_translate.h"
#include <ros/node_handle.h>
#include <ros/publisher.h>

namespace fkie_message_filters
{

/** \brief Publish consumed data on a ROS topic.
 *
 * This class together with the Subscriber class is the generic interface between ROS and this library.
 * All messages which are received from the connected sources will be published on the advertised ROS topic.
 * For maximum flexibility, you can choose one of four ways how to receive messages from your sources:
 *
 * \li \c Publisher<M, RosMessageEvent> will act as a sink of \c ros::MessageEvent<const M> objects (default)
 * \li \c Publisher<M, RosMessageConstPtr> will act as a sink of \c M::ConstPtr objects (as of this writing, this is a \c boost::shared_ptr<const M>)
 * \li \c Publisher<M, RosMessageStdSharedPtr> will act as a sink of \c std::shared_ptr<const M> objects
 * \li \c Publisher<M, RosMessage> will act as a sink of plain \c M objects
 *
 * Unlike regular ROS publishers, this class can be associated with one or more subscriber instances. In that case,
 * the subscribers will subscribe to their ROS topics only if the publisher is actively used. This is a convenient
 * method to save processing power if the filter pipeline is used only intermittently.
 *
 * \sa CameraPublisher, ImagePublisher
 */
template<class M, template<typename> class Translate = RosMessageEvent>
class Publisher : public PublisherBase, public Sink<typename Translate<M>::FilterType>
{
public:
    /** \brief Constructs an empty publisher.
     *
     * You need to call advertise() to actually publish to a ROS topic.
     *
     * \nothrow
     */
    Publisher() noexcept;
    /** \brief Constructor that advertises the given ROS topic.
     *
     * The constructor calls advertise() for you.
     *
     * \nothrow
     */
    Publisher(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size, bool latch = false, ros::CallbackQueueInterface* callback_queue = nullptr) noexcept;
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
    /** \brief Advertise ROS topic.
     *
     * All arguments are passed to the ROS client library; see the ROS documentation for further information. Calling
     * this method will automatically unadvertise any previously advertised ROS topic.
     *
     * \arg \c nh ROS node handle to create the ROS advertisement
     * \arg \c topic name of the ROS topic, subject to remapping
     * \arg \c queue_size size of the ROS publishing queue
     * \arg \c latch if true, the last published message remains available for later subscribers
     * \arg \c callback_queue custom ROS callback queue
     *
     * \nothrow
     */
    void advertise(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size, bool latch = false, ros::CallbackQueueInterface* callback_queue = nullptr) noexcept;
    /** \brief Advertise ROS topic with subscriber status callbacks.
     *
     * All arguments are passed to the ROS client library; see the ROS documentation for further information. Calling
     * this method will automatically unadvertise any previously advertised ROS topic.
     *
     * \arg \c nh ROS node handle to create the ROS advertisement
     * \arg \c topic name of the ROS topic, subject to remapping
     * \arg \c queue_size size of the ROS publishing queue
     * \arg \c connect_cb callback that is invoked each time a new subscriber connects to the advertised topic
     * \arg \c disconnect_cb callback that is invoked each time an existing subscriber disconnects from the advertised topic
     * \arg \c tracked_object an associated object whose lifetime will limit the lifetime of the advertised topic
     * \arg \c latch if true, the last published message remains available for later subscribers
     * \arg \c callback_queue custom ROS callback queue
     *
     * \nothrow
     */
    void advertise(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size, const ros::SubscriberStatusCallback& connect_cb, const ros::SubscriberStatusCallback& disconnect_cb = ros::SubscriberStatusCallback(), const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr(), bool latch = false, ros::CallbackQueueInterface* callback_queue = nullptr) noexcept;

protected:
    /** \private */
    virtual void receive (const typename Translate<M>::FilterType& t) noexcept override;
private:
    ros::Publisher pub_;
};

} // namespace fkie_message_filters

#include "publisher_impl.h"

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_PUBLISHER_H_ */
