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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_CAMERA_PUBLISHER_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_CAMERA_PUBLISHER_HPP_
#pragma once

#include "helpers/abi_namespace.hpp"
#include "message_translate.hpp"
#include "publisher_base.hpp"
#include "sink.hpp"

#include <image_transport/image_transport.hpp>

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

/** \brief Publish consumed data to ROS camera topics
 *
 * This is a specialized publisher that uses image_transport to publish ROS camera topics. All messages which are
 * received from the connected sources will be published on the corresponding advertised ROS topics. For maximum
 * flexibility, you can choose how to receive messages from your sources:
 *
 * \li \c CameraPublisher<RosMessageSharedPtr> will act as a sink of \c sensor_msgs::msg::Image::ConstSharedPtr and
 * \c sensor_msgs::msg::CameraInfo::ConstSharedPtr objects (default)
 * \li \c CameraPublisher<RosMessageUniquePtr> will act as a sink of \c sensor_msgs::msg::Image::UniquePtr and
 * \c sensor_msgs::msg::CameraInfo::UniquePtr objects
 * \li \c CameraPublisher<RosMessage> will act as a sink of plain \c sensor_msgs::msg::Image and
 * \c sensor_msgs::msg::CameraInfo objects
 *
 * Unlike regular ROS publishers, this class can be associated with one or more subscriber instances. In that case, the
 * subscribers will subscribe to their ROS topics only if the publisher is actively used. This is a convenient method to
 * save processing power if the filter pipeline is used only intermittently.
 *
 * \sa CameraSubscriber, ImagePublisher, Publisher
 */
template<template<typename> class Translate = RosMessageSharedPtr>
class CameraPublisher : public PublisherBase,
                        public Sink<typename Translate<sensor_msgs::msg::Image>::FilterType,
                                    typename Translate<sensor_msgs::msg::CameraInfo>::FilterType>
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
    CameraPublisher(rclcpp::Node::SharedPtr& node, const std::string& topic,
                    const rclcpp::QoS& qos = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default),
                    const rclcpp::PublisherOptions& options = rclcpp::PublisherOptions()) noexcept;
    /** \brief Destructor. */
    virtual ~CameraPublisher();
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
     * All arguments are passed to the image_transport library; see the ROS documentation for further information.
     * Calling this method will automatically unadvertise any previously advertised ROS topic.
     *
     * \arg \c node a node instance to handle the publishing
     * \arg \c base_topic name of the ROS camera topic, subject to remapping
     * \arg \c qos the ROS quality of service specification
     * \arg \c options ROS publisher options
     *
     * \nothrow
     */
    void advertise(rclcpp::Node::SharedPtr& node, const std::string& base_topic,
                   const rclcpp::QoS& qos = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default),
                   const rclcpp::PublisherOptions& options = rclcpp::PublisherOptions()) noexcept;

protected:
    /** \private */
    void receive(helpers::argument_t<typename Translate<sensor_msgs::msg::Image>::FilterType>,
                 helpers::argument_t<typename Translate<sensor_msgs::msg::CameraInfo>::FilterType>) noexcept override;

private:
    image_transport::CameraPublisher pub_;
};

FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#include "camera_publisher_impl.hpp"  // IWYU pragma: keep

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_CAMERA_PUBLISHER_HPP_ */
