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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_SUBSCRIBER_IMPL_H_
#define INCLUDE_FKIE_MESSAGE_FILTERS_SUBSCRIBER_IMPL_H_

#include "subscriber.h"

namespace fkie_message_filters
{

template<class M, template<typename > class Translate>
Subscriber<M, Translate>::Subscriber() noexcept
{
}

template<class M, template<typename > class Translate>
Subscriber<M, Translate>::Subscriber(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size, const ros::TransportHints& transport_hints, ros::CallbackQueueInterface* callback_queue) noexcept
{
    subscribe(nh, topic, queue_size, transport_hints, callback_queue);
}

template<class M, template<typename > class Translate>
std::string Subscriber<M, Translate>::topic() const noexcept
{
    return sub_.getTopic();
}

template<class M, template<typename > class Translate>
void Subscriber<M, Translate>::set_subscribe_options(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size, const ros::TransportHints& transport_hints, ros::CallbackQueueInterface* callback_queue) noexcept
{
    unsubscribe();
    if (!topic.empty())
    {
        opts_.initByFullCallbackType<const EventType&>(topic, queue_size, std::bind(&Subscriber<M, Translate>::cb, this, std::placeholders::_1));
        opts_.callback_queue = callback_queue;
        opts_.transport_hints = transport_hints;
        nh_ = std::make_shared<ros::NodeHandle>(nh);
    }
}

template<class M, template<typename > class Translate>
void Subscriber<M, Translate>::subscribe(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size,
        const ros::TransportHints& transport_hints, ros::CallbackQueueInterface* callback_queue) noexcept
{
    set_subscribe_options(nh, topic, queue_size, transport_hints, callback_queue);
    subscribe();
}

template<class M, template<typename > class Translate>
bool Subscriber<M, Translate>::is_configured() const noexcept
{
    return nh_ && !opts_.topic.empty();
}

template<class M, template<typename > class Translate>
void Subscriber<M, Translate>::subscribe_impl() noexcept
{
    if (!sub_)
    {
        sub_ = nh_->subscribe(opts_);
    }
}

template<class M, template<typename > class Translate>
void Subscriber<M, Translate>::unsubscribe_impl() noexcept
{
    sub_.shutdown();
}

template<class M, template<typename > class Translate>
void Subscriber<M, Translate>::cb(const EventType& event)
{
    this->send(Translate<M>::eventToFilter(event));
}

} // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_SUBSCRIBER_IMPL_H_ */
