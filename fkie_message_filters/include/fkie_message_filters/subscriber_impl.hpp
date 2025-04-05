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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_SUBSCRIBER_IMPL_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_SUBSCRIBER_IMPL_HPP_
#pragma once

// IWYU pragma: private; include "subscriber.hpp"

#include "subscriber.hpp"

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

template<class M, template<typename> class Translate>
Subscriber<M, Translate>::Subscriber() noexcept
{
}

template<class M, template<typename> class Translate>
Subscriber<M, Translate>::Subscriber(const rclcpp::Node::SharedPtr& node, const std::string& topic,
                                     const rclcpp::QoS& qos, const rclcpp::SubscriptionOptions& options) noexcept
{
    subscribe(node, topic, qos, options);
}

template<class M, template<typename> class Translate>
std::string Subscriber<M, Translate>::topic() const noexcept
{
    return sub_ ? sub_->get_topic_name() : std::string();
}

template<class M, template<typename> class Translate>
void Subscriber<M, Translate>::set_subscribe_options(const rclcpp::Node::SharedPtr& node, const std::string& topic,
                                                     const rclcpp::QoS& qos,
                                                     const rclcpp::SubscriptionOptions& options) noexcept
{
    unsubscribe();
    node_ = node;
    topic_ = topic;
    qos_ = qos;
    options_ = options;
}

template<class M, template<typename> class Translate>
void Subscriber<M, Translate>::subscribe(const rclcpp::Node::SharedPtr& node, const std::string& topic,
                                         const rclcpp::QoS& qos, const rclcpp::SubscriptionOptions& options) noexcept
{
    set_subscribe_options(node, topic, qos, options);
    subscribe();
}

template<class M, template<typename> class Translate>
bool Subscriber<M, Translate>::is_configured() const noexcept
{
    return node_ && !topic_.empty();
}

template<class M, template<typename> class Translate>
void Subscriber<M, Translate>::subscribe_impl() noexcept
{
    if (!sub_)
    {
        sub_ = node_->create_subscription<MessageType, SubscriptionCB>(
            topic_, qos_, [this](SubscriptionType message) { this->send(Translate<M>::subscriberToFilter(message)); },
            options_);
    }
}

template<class M, template<typename> class Translate>
void Subscriber<M, Translate>::unsubscribe_impl() noexcept
{
    sub_.reset();
}

FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_SUBSCRIBER_IMPL_HPP_ */
