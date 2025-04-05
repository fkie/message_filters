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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_MESSAGE_TRANSLATE_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_MESSAGE_TRANSLATE_HPP_
#pragma once

#include "helpers/abi_namespace.hpp"

#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <memory>

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

template<class M>
struct RosMessage
{
    using MessageType = M;
    using FilterType = M;
    using Publisher = rclcpp::Publisher<MessageType>;

    static MessageType create()
    {
        return MessageType();
    }

    template<class PublisherT, class... Ms>
    static void publish(PublisherT& pub, Ms&&... in)
    {
        pub.publish(std::forward<Ms&&>(in)...);
    }

    static FilterType subscriberToFilter(typename M::UniquePtr& m)
    {
        return *m;
    }

    static FilterType subscriberToFilter(const typename M::ConstSharedPtr& m)
    {
        return *m;
    }
};

template<class M>
struct RosMessageUniquePtr
{
    using MessageType = M;
    using FilterType = typename M::UniquePtr;
    using Publisher = rclcpp::Publisher<MessageType>;

    static typename MessageType::UniquePtr create()
    {
        return std::make_unique<MessageType>();
    }

    template<class PublisherT, class... Ms>
    static void publish(PublisherT& pub, Ms&... ms)
    {
        pub.publish(std::move(ms)...);
    }

    static FilterType&& subscriberToFilter(typename M::UniquePtr& m)
    {
        return std::move(m);
    }
};

template<class M>
struct RosMessageSharedPtr
{
    using MessageType = M;
    using FilterType = typename M::ConstSharedPtr;
    using Publisher = rclcpp::Publisher<MessageType>;

    static typename MessageType::SharedPtr create()
    {
        return std::make_shared<MessageType>();
    }

    template<class PublisherT, class... Ms>
    static void publish(PublisherT& pub, const Ms&... ms)
    {
        pub.publish(*ms...);
    }

    static FilterType subscriberToFilter(typename M::UniquePtr& m)
    {
        return FilterType(std::move(m));
    }

    static FilterType subscriberToFilter(const typename M::ConstSharedPtr& m)
    {
        return m;
    }
};

FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_MESSAGE_TRANSLATE_HPP_ */
