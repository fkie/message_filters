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

#include <fkie_message_filters/subscriber_base.h>
#include <fkie_message_filters/publisher_base.h>

namespace fkie_message_filters
{

std::tuple<boost::signals2::connection, boost::signals2::connection> PublisherBase::link_with_subscriber(SubscriberBase& sub)
{
    boost::signals2::connection c1, c2;
    c1 = enable_signal_.connect([&sub]() { sub.subscribe_impl(); });
    c2 = disable_signal_.connect([&sub]() { sub.unsubscribe_impl(); });
    if (is_active()) sub.subscribe_impl(); else sub.unsubscribe_impl();
    return std::tie(c1, c2);
}

void PublisherBase::update_subscriber_state()
{
    if (is_active()) enable_signal_(); else disable_signal_();
}

SubscriberBase::~SubscriberBase()
{
    unlink_from_publisher();
}

void SubscriberBase::subscribe()
{
    unlink_from_publisher();
    if (is_configured()) subscribe_impl();
}

void SubscriberBase::unsubscribe()
{
    unlink_from_publisher();
    if (is_configured()) unsubscribe_impl();
}

void SubscriberBase::subscribe_on_demand(PublisherBase& pub)
{
    unlink_from_publisher();
    if (is_configured()) link_with_publisher(pub);
}

void SubscriberBase::unlink_from_publisher()
{
    conn1_.disconnect();
    conn2_.disconnect();
}

void SubscriberBase::link_with_publisher(PublisherBase& pub)
{
    unlink_from_publisher();
    std::tie(conn1_, conn2_) = pub.link_with_subscriber(*this);
}

} // namespace fkie_message_filters
