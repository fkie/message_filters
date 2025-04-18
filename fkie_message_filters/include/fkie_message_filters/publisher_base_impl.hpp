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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_PUBLISHER_BASE_IMPL_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_PUBLISHER_BASE_IMPL_HPP_
#pragma once

// IWYU pragma: private; include "publisher_base.hpp"

#include "publisher_base.hpp"

#include <rclcpp/node_interfaces/get_node_graph_interface.hpp>

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

template<class NodeT, typename std::enable_if_t<
                          !std::is_convertible_v<NodeT, rclcpp::node_interfaces::NodeGraphInterface::SharedPtr>, bool>>
void PublisherBase::start_monitor(NodeT&& node) noexcept
{
    if (node)
    {
        rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface =
            rclcpp::node_interfaces::get_node_graph_interface(node);
        start_monitor(node_graph_interface);
    }
}

FKIE_MF_END_ABI_NAMESPACE

}  // namespace fkie_message_filters

#endif
