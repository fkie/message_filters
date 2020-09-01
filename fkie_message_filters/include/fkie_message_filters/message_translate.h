/****************************************************************************
 *
 * fkie_message_filters
 * Copyright © 2018-2020 Fraunhofer FKIE
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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_MESSAGE_TRANSLATE_H_
#define INCLUDE_FKIE_MESSAGE_FILTERS_MESSAGE_TRANSLATE_H_

#include <ros/message_event.h>
#include "helpers/shared_ptr_compat.h"

namespace fkie_message_filters
{

template<class M>
struct RosMessage
{
    using MessageType = M;
    using PublishType = M;
    using EventType = ros::MessageEvent<const M>;
    using FilterType = M;
    static FilterType eventToFilter(const EventType& t)
    {
        return *t.getConstMessage().get();
    }
    static PublishType filterToPublish(const FilterType& t)
    {
        return t;
    }
    static const MessageType& filterToMessage(const FilterType& t)
    {
        return t;
    }
};

template<class M>
struct RosMessageConstPtr
{
    using MessageType = M;
    using PublishType = typename M::ConstPtr;
    using EventType = ros::MessageEvent<const M>;
    using FilterType = typename M::ConstPtr;
    static FilterType eventToFilter(const EventType& t) noexcept
    {
        return t.getConstMessage();
    }
    static PublishType filterToPublish(const FilterType& t) noexcept
    {
        return t;
    }
    static const MessageType& filterToMessage(const FilterType& t) noexcept
    {
        return *t;
    }
};

template<class M>
struct RosMessageStdSharedPtr
{
    using MessageType = M;
    using PublishType = typename M::ConstPtr;
    using EventType = ros::MessageEvent<const M>;
    using FilterType = std::shared_ptr<const M>;
    static FilterType eventToFilter(const EventType& t) noexcept
    {
        return helpers::convert_shared_ptr<FilterType>(t.getConstMessage());
    }
    static PublishType filterToPublish(const FilterType& t) noexcept
    {
        return helpers::convert_shared_ptr<PublishType>(t);
    }
    static const MessageType& filterToMessage(const FilterType& t) noexcept
    {
        return *t;
    }
};

template<class M>
struct RosMessageEvent
{
    using MessageType = M;
    using PublishType = typename M::ConstPtr;
    using EventType = ros::MessageEvent<const M>;
    using FilterType = ros::MessageEvent<const M>;
    static FilterType eventToFilter(const EventType& t) noexcept
    {
        return t;
    }
    static PublishType filterToPublish(const FilterType& t) noexcept
    {
        return t.getConstMessage();
    }
    static const MessageType& filterToMessage(const FilterType& t) noexcept
    {
        return *t.getConstMessage();
    }
};

} // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_MESSAGE_TRANSLATE_H_ */
