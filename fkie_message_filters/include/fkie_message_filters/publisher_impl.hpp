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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_PUBLISHER_IMPL_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_PUBLISHER_IMPL_HPP_
#pragma once

// IWYU pragma: private; include "publisher.hpp"

#include "publisher.hpp"

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

template<class M, template<typename> class Translate>
Publisher<M, Translate>::Publisher() noexcept
{
}

template<class M, template<typename> class Translate>
Publisher<M, Translate>::~Publisher()
{
    shutdown_monitor();
}

template<class M, template<typename> class Translate>
Publisher<M, Translate>::Publisher(rclcpp::Node::SharedPtr& node, const std::string& topic, const rclcpp::QoS& qos,
                                   const rclcpp::PublisherOptions& options) noexcept
{
    advertise(node, topic, qos, options);
}

template<class M, template<typename> class Translate>
bool Publisher<M, Translate>::is_active() const noexcept
{
    return pub_ ? pub_->get_subscription_count() > 0 : false;
}

template<class M, template<typename> class Translate>
std::string Publisher<M, Translate>::topic() const noexcept
{
    return pub_ ? pub_->get_topic_name() : std::string();
}

template<class M, template<typename> class Translate>
void Publisher<M, Translate>::advertise(rclcpp::Node::SharedPtr& node, const std::string& topic, const rclcpp::QoS& qos,
                                        const rclcpp::PublisherOptions& options) noexcept
{
    pub_ = node->create_publisher<MessageType>(topic, qos, options);
    start_monitor(node);
    update_subscriber_state();
}

template<class M, template<typename> class Translate>
void Publisher<M, Translate>::receive(helpers::argument_t<typename Translate<M>::FilterType> m) noexcept
{
    Translate<M>::publish(*pub_, m);
}

FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_PUBLISHER_IMPL_HPP_ */
