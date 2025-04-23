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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_SUBSCRIBER_IMPL_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_SUBSCRIBER_IMPL_HPP_
#pragma once

// IWYU pragma: private; include "subscriber.hpp"

#include "subscriber.hpp"

#include <rclcpp/create_subscription.hpp>
#include <rclcpp/node_interfaces/get_node_parameters_interface.hpp>
#include <rclcpp/node_interfaces/get_node_topics_interface.hpp>

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

template<class M, template<typename, typename> class Translate, class A>
Subscriber<M, Translate, A>::Subscriber() noexcept
{
}

template<class M, template<typename, typename> class Translate, class A>
template<class NodeT>
Subscriber<M, Translate, A>::Subscriber(NodeT&& node, const std::string& topic, const rclcpp::QoS& qos,
                                        const rclcpp::SubscriptionOptionsWithAllocator<A>& options)
{
    subscribe<NodeT>(std::forward<NodeT&&>(node), topic, qos, options);
}

template<class M, template<typename, typename> class Translate, class A>
std::string Subscriber<M, Translate, A>::topic() const noexcept
{
    return sub_ ? sub_->get_topic_name() : std::string();
}

template<class M, template<typename, typename> class Translate, class A>
template<class NodeT>
void Subscriber<M, Translate, A>::set_subscribe_options(NodeT&& node, const std::string& topic, const rclcpp::QoS& qos,
                                                        const rclcpp::SubscriptionOptionsWithAllocator<A>& options)
{
    unsubscribe();
    node_parameters_ = rclcpp::node_interfaces::get_node_parameters_interface(node);
    node_topics_ = rclcpp::node_interfaces::get_node_topics_interface(node);
    topic_ = topic;
    qos_ = qos;
    options_ = options;
}

template<class M, template<typename, typename> class Translate, class A>
template<class NodeT>
void Subscriber<M, Translate, A>::subscribe(NodeT&& node, const std::string& topic, const rclcpp::QoS& qos,
                                            const rclcpp::SubscriptionOptionsWithAllocator<A>& options)
{
    set_subscribe_options<NodeT>(std::forward<NodeT&&>(node), topic, qos, options);
    subscribe();
}

template<class M, template<typename, typename> class Translate, class A>
bool Subscriber<M, Translate, A>::is_configured() const noexcept
{
    return node_parameters_ && node_topics_ && !topic_.empty();
}

template<class M, template<typename, typename> class Translate, class A>
void Subscriber<M, Translate, A>::subscribe_impl() noexcept
{
    if (!sub_)
    {
        sub_ = rclcpp::create_subscription<MessageType, SubscriptionCB, A>(
            node_parameters_, node_topics_, topic_, qos_,
            [this](SubscriptionType message) { this->send(Translate<M, A>::subscriberToFilter(message)); }, options_);
    }
}

template<class M, template<typename, typename> class Translate, class A>
void Subscriber<M, Translate, A>::unsubscribe_impl() noexcept
{
    sub_.reset();
}

FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_SUBSCRIBER_IMPL_HPP_ */
