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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_PUBLISHER_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_PUBLISHER_HPP_
#pragma once

#include "helpers/abi_namespace.hpp"
#include "message_translate.hpp"
#include "publisher_base.hpp"
#include "source.hpp"

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

/** \brief Publish consumed data on a ROS topic.
 *
 * This class together with the Subscriber class is the generic interface between ROS and this library. All messages
 * which are received from the connected sources will be published on the advertised ROS topic. For maximum flexibility,
 * you can choose how to receive messages from your sources:
 *
 * \li \c Publisher<M, RosMessageSharedPtr> will act as a sink of \c M::ConstSharedPtr objects (default)
 * \li \c Publisher<M, RosMessageUniquePtr> will act as a sink of \c M::UniquePtr objects
 * \li \c Publisher<M, RosMessage> will act as a sink of plain \c M objects
 *
 * Unlike regular ROS publishers, this class can be associated with one or more subscriber instances. In that case, the
 * subscribers will subscribe to their ROS topics only if the publisher is actively used. This is a convenient method to
 * save processing power if the filter pipeline is used only intermittently.
 *
 * \sa CameraPublisher, ImagePublisher
 */
template<class M, template<typename, typename> class Translate = RosMessageSharedPtr, class A = std::allocator<void>>
class Publisher : public PublisherBase, public Sink<typename Translate<M, A>::FilterType>
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
     * \rmwthrow
     */
    template<class NodeT>
    Publisher(NodeT&& node, const std::string& topic,
              const rclcpp::QoS& qos = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default),
              const rclcpp::PublisherOptionsWithAllocator<A>& options = rclcpp::PublisherOptionsWithAllocator<A>());
    /** \brief Destructor. */
    virtual ~Publisher();
    /** \brief Check if the ROS publisher has at least one subscriber.
     *
     * \arg \c node ROS node instance to create the ROS publisher
     * \arg \c topic name of the ROS topic, subject to remapping
     * \arg \c qos the ROS quality of service specification
     * \arg \c options ROS publisher options
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
     * \arg \c node ROS node instance to create the ROS publisher
     * \arg \c topic name of the ROS topic, subject to remapping
     * \arg \c qos the ROS quality of service specification
     * \arg \c options ROS publisher options
     *
     * \rmwthrow
     */
    template<class NodeT>
    void
    advertise(NodeT&& node, const std::string& topic,
              const rclcpp::QoS& qos = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default),
              const rclcpp::PublisherOptionsWithAllocator<A>& options = rclcpp::PublisherOptionsWithAllocator<A>());

protected:
    /** \private */
    virtual void receive(helpers::argument_t<typename Translate<M, A>::FilterType> m) override;

private:
    using MessageType = typename Translate<M, A>::MessageType;
    using PublisherROS = typename Translate<M, A>::Publisher;
    typename PublisherROS::SharedPtr pub_;
};

FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#include "publisher_impl.hpp"  // IWYU pragma: keep

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_PUBLISHER_HPP_ */
