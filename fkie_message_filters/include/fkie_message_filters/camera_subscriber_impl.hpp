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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_CAMERA_SUBSCRIBER_IMPL_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_CAMERA_SUBSCRIBER_IMPL_HPP_
#pragma once

// IWYU pragma: private; include "camera_subscriber.hpp"

#include "camera_subscriber.hpp"
#include "helpers/argument.hpp"
#include "helpers/image_transport.hpp"

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

template<template<typename> class Translate>
template<class NodeT>
CameraSubscriber<Translate>::CameraSubscriber(NodeT&& node, const std::string& base_topic, const rclcpp::QoS& qos,
                                              const std::optional<image_transport::TransportHints>& transport_hints,
                                              const rclcpp::SubscriptionOptions& options) noexcept
{
    set_subscribe_options<NodeT>(node, base_topic, qos, transport_hints, options);
    subscribe();
}

template<template<typename> class Translate>
template<class NodeT>
void CameraSubscriber<Translate>::set_subscribe_options(
    NodeT&& node, const std::string& base_topic, const rclcpp::QoS& qos,
    const std::optional<image_transport::TransportHints>& transport_hints,
    const rclcpp::SubscriptionOptions& options) noexcept
{
    node_ = FKIE_MF_IMAGE_TRANSPORT_NODE_COPY(node);
    base_topic_ = base_topic;
    transport_ = transport_hints ? transport_hints->getTransport()
                                 : image_transport::TransportHints(FKIE_MF_IMAGE_TRANSPORT_NODE(node)).getTransport();
    qos_ = qos;
    options_ = options;
}

template<template<typename> class Translate>
template<class NodeT>
void CameraSubscriber<Translate>::subscribe(NodeT&& node, const std::string& base_topic, const rclcpp::QoS& qos,
                                            const std::optional<image_transport::TransportHints>& transport_hints,
                                            const rclcpp::SubscriptionOptions& options)
{
    set_subscribe_options<NodeT>(node, base_topic, qos, transport_hints, options);
    subscribe();
}

template<template<typename> class Translate>
std::string CameraSubscriber<Translate>::topic() const noexcept
{
    return sub_.getTopic();
}

template<template<typename> class Translate>
bool CameraSubscriber<Translate>::is_configured() const noexcept
{
    return !transport_.empty() && !base_topic_.empty();
}

template<template<typename> class Translate>
void CameraSubscriber<Translate>::subscribe_impl()
{
    if (!sub_)
    {
        sub_ = image_transport::create_camera_subscription(
            FKIE_MF_IMAGE_TRANSPORT_NODE(node_), base_topic_,
            [this](const sensor_msgs::msg::Image::ConstSharedPtr& img,
                   const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info)
            {
                this->send(Translate<sensor_msgs::msg::Image>::subscriberToFilter(img),
                           Translate<sensor_msgs::msg::CameraInfo>::subscriberToFilter(info));
            },
            transport_, FKIE_MF_IMAGE_TRANSPORT_QOS(qos_));
    }
}

template<template<typename> class Translate>
void CameraSubscriber<Translate>::unsubscribe_impl()
{
    if (sub_)
        sub_.shutdown();
}

FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#endif
