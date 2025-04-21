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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_CAMERA_SUBSCRIBER_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_CAMERA_SUBSCRIBER_HPP_
#pragma once

#include "helpers/abi_namespace.hpp"
#include "message_translate.hpp"
#include "source.hpp"
#include "subscriber_base.hpp"

#include <image_transport/image_transport.hpp>

#include <optional>

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

/** \brief Subscribe to ROS camera topics as data provider.
 *
 * This is a specialized subscriber that uses image_transport to subscribe to ROS camera topics. All messages which are
 * received on the subscribed topics will be passed to the connected sinks for further processing.
 * For maximum flexibility, you can choose how to pass the received messages:
 *
 * \li \c ImagePublisher<RosMessageSharedPtr> will act as a source of \c sensor_msgs::msg::Image::ConstSharedPtr and
 * \c sensor_msgs::msg::CameraInfo::ConstSharedPtr objects (default)
 * \li \c ImagePublisher<RosMessage> will act as a source of plain \c sensor_msgs::msg::Image and
 * \c sensor_msgs::msg::CameraInfo objects
 *
 * Unlike regular ROS subscribers, this class can be associated with a publisher instance. In that case, the subscriber
 * will delay subscription until the publisher is actively used and will unsubscribe (and stop passing data) as soon as
 * the publisher becomes idle. This is a convenient method to save processing power if the filter pipeline is used only
 * intermittently.
 *
 * \sa CameraPublisher, ImageSubscriber, Subscriber
 */
template<template<typename> class Translate = RosMessageSharedPtr>
class CameraSubscriber : public SubscriberBase,
                         public Source<typename Translate<sensor_msgs::msg::Image>::FilterType,
                                       typename Translate<sensor_msgs::msg::CameraInfo>::FilterType>
{
public:
    /** \brief Constructs an empty subscriber.
     *
     * You need to call set_subscribe_options() and either subscribe() or subscribe_on_demand() to actually subscribe to
     * a ROS topic.
     */
    CameraSubscriber() noexcept {}
    /** \brief Constructor that subscribes to the given ROS camera base topic.
     *
     * This constructor calls set_subscribe_options() and subscribe() for you.
     *
     * \arg \c node a node instance to handle the subscription
     * \arg \c base_topic name of the ROS camera topic, subject to remapping
     * \arg \c qos the ROS quality of service specification
     * \arg \c transport_hints transport hints for the ROS image_transport framework
     * \arg \c options ROS subscription options (currently ignored by image_transport)
     *
     * \rmwthrow
     */
    CameraSubscriber(const rclcpp::Node::SharedPtr& node, const std::string& base_topic,
                     const rclcpp::QoS& qos = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default),
                     const std::optional<image_transport::TransportHints>& transport_hints = std::nullopt,
                     const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions()) noexcept;
    /** \brief Configure ROS topic that is to be subscribed.
     *
     * All arguments are passed to the ROS client library; see the ROS documentation for further information. Calling
     * this method will automatically unsubscribe any previously subscribed ROS topic.
     *
     * \arg \c node a node instance to handle the subscription
     * \arg \c base_topic name of the ROS camera topic, subject to remapping
     * \arg \c qos the ROS quality of service specification
     * \arg \c transport_hints transport hints for the ROS image_transport framework
     * \arg \c options ROS subscription options (currently ignored by image_transport)
     *
     * \nothrow
     */
    void set_subscribe_options(const rclcpp::Node::SharedPtr& node, const std::string& base_topic,
                               const rclcpp::QoS& qos = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default),
                               const std::optional<image_transport::TransportHints>& transport_hints = std::nullopt,
                               const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions()) noexcept;
    /** \brief Convenience function to subscribe to a ROS topic.
     *
     * This function is equivalent to calling set_subscribe_options() and then subscribe().
     *
     * \arg \c node a node instance to handle the subscription
     * \arg \c base_topic name of the ROS camera topic, subject to remapping
     * \arg \c qos the ROS quality of service specification
     * \arg \c transport_hints transport hints for the ROS image_transport framework
     * \arg \c options ROS subscription options (currently ignored by image_transport)
     *
     * \rmwthrow
     */
    void subscribe(const rclcpp::Node::SharedPtr& node, const std::string& base_topic,
                   const rclcpp::QoS& qos = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default),
                   const std::optional<image_transport::TransportHints>& transport_hints = std::nullopt,
                   const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions());
    using SubscriberBase::subscribe;
    using SubscriberBase::subscribe_on_demand;
    using SubscriberBase::unsubscribe;
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
    virtual void subscribe_impl() override;
    /** \brief Shut the ROS subscriber down.
     *
     * \nothrow
     */
    virtual void unsubscribe_impl() override;

private:
    static_assert(!std::is_same_v<Translate<sensor_msgs::msg::Image>, RosMessageUniquePtr<sensor_msgs::msg::Image>>,
                  "CameraSubscriber cannot be used with RosMessageUniquePtr");
    rclcpp::Node::SharedPtr node_;
    std::string base_topic_;
    std::string transport_;
    rclcpp::QoS qos_{rclcpp::KeepLast(10), rmw_qos_profile_default};
    rclcpp::SubscriptionOptions options_;
    image_transport::CameraSubscriber sub_;
};

FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#include "camera_subscriber_impl.hpp"  // IWYU pragma: keep

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_CAMERA_SUBSCRIBER_HPP_ */
