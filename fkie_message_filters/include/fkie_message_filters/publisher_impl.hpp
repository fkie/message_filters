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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_PUBLISHER_IMPL_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_PUBLISHER_IMPL_HPP_
#pragma once

// IWYU pragma: private; include "publisher.hpp"

#include "publisher.hpp"

#include <rclcpp/create_publisher.hpp>

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

template<class M, template<typename, typename> class Translate, class A>
Publisher<M, Translate, A>::Publisher() noexcept
{
}

template<class M, template<typename, typename> class Translate, class A>
Publisher<M, Translate, A>::~Publisher()
{
    shutdown_monitor();
}

template<class M, template<typename, typename> class Translate, class A>
template<class NodeT>
Publisher<M, Translate, A>::Publisher(NodeT&& node, const std::string& topic, const rclcpp::QoS& qos,
                                      const rclcpp::PublisherOptionsWithAllocator<A>& options)
{
    advertise<NodeT>(std::forward<NodeT&&>(node), topic, qos, options);
}

template<class M, template<typename, typename> class Translate, class A>
bool Publisher<M, Translate, A>::is_active() const noexcept
{
    return pub_ ? pub_->get_subscription_count() > 0 : false;
}

template<class M, template<typename, typename> class Translate, class A>
std::string Publisher<M, Translate, A>::topic() const noexcept
{
    return pub_ ? pub_->get_topic_name() : std::string();
}

template<class M, template<typename, typename> class Translate, class A>
template<class NodeT>
void Publisher<M, Translate, A>::advertise(NodeT&& node, const std::string& topic, const rclcpp::QoS& qos,
                                           const rclcpp::PublisherOptionsWithAllocator<A>& options)
{
    pub_ =
        rclcpp::create_publisher<MessageType, A, PublisherROS, NodeT>(std::forward<NodeT&&>(node), topic, qos, options);
    start_monitor<NodeT>(std::forward<NodeT&&>(node));
    update_subscriber_state();
}

template<class M, template<typename, typename> class Translate, class A>
void Publisher<M, Translate, A>::receive(helpers::argument_t<typename Translate<M, A>::FilterType> m)
{
    Translate<M, A>::publish(*pub_, m);
}

FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_PUBLISHER_IMPL_HPP_ */
