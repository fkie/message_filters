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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_HELPERS_ROS_NODE_INTERFACES_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_HELPERS_ROS_NODE_INTERFACES_HPP_
#pragma once

#include "helpers/abi_namespace.hpp"
#include "version.hpp"

#include <image_transport/image_transport.hpp>

#include <memory>

#if FKIE_MF_IMAGE_TRANSPORT_VERSION >= FKIE_MF_VERSION_TUPLE(6, 4, 0)
#    define FKIE_MF_IMAGE_TRANSPORT_NODE(node)      fkie_message_filters::helpers::get_image_transport_node_like(node)
#    define FKIE_MF_IMAGE_TRANSPORT_NODE_COPY(node) fkie_message_filters::helpers::get_image_transport_node_like(node)
#    define FKIE_MF_IMAGE_TRANSPORT_QOS(qos)        (qos)
#else
#    define FKIE_MF_IMAGE_TRANSPORT_NODE(node) fkie_message_filters::helpers::get_image_transport_node_ptr(node)
#    define FKIE_MF_IMAGE_TRANSPORT_NODE_COPY(node) \
        fkie_message_filters::helpers::get_image_transport_node_shared_ptr(node)
#    define FKIE_MF_IMAGE_TRANSPORT_QOS(qos) ((qos).get_rmw_qos_profile())
#endif

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

namespace helpers
{

#if FKIE_MF_IMAGE_TRANSPORT_VERSION >= FKIE_MF_VERSION_TUPLE(6, 4, 0)
inline image_transport::RequiredInterfaces get_image_transport_node_like(image_transport::RequiredInterfaces interfaces)
{
    return interfaces;
}

template<class NodeT>
image_transport::RequiredInterfaces get_image_transport_node_like(const std::shared_ptr<NodeT>& node_like)
{
    return *node_like;
}

template<class NodeT>
image_transport::RequiredInterfaces get_image_transport_node_like(const std::unique_ptr<NodeT>& node_like)
{
    return *node_like;
}

template<class NodeT>
image_transport::RequiredInterfaces get_image_transport_node_like(NodeT* node_like)
{
    return *node_like;
}

#else

inline rclcpp::Node* get_image_transport_node_ptr(rclcpp::Node* node)
{
    return node;
}

inline rclcpp::Node* get_image_transport_node_ptr(const std::shared_ptr<rclcpp::Node>& node)
{
    return node.get();
}

inline rclcpp::Node* get_image_transport_node_ptr(const std::unique_ptr<rclcpp::Node>& node)
{
    return node.get();
}

inline rclcpp::Node* get_image_transport_node_ptr(rclcpp::Node& node)
{
    return &node;
}

inline std::shared_ptr<rclcpp::Node> get_image_transport_node_shared_ptr(rclcpp::Node* node)
{
    return node->shared_from_this();
}

inline std::shared_ptr<rclcpp::Node> get_image_transport_node_shared_ptr(const std::shared_ptr<rclcpp::Node>& node)
{
    return node;
}

inline std::shared_ptr<rclcpp::Node> get_image_transport_node_shared_ptr(std::unique_ptr<rclcpp::Node>&& node)
{
    return std::shared_ptr<rclcpp::Node>(std::move(node));
}

#endif

}  // namespace helpers

FKIE_MF_END_ABI_NAMESPACE

}  // namespace fkie_message_filters

#endif
