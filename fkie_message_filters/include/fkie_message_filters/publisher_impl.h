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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_PUBLISHER_IMPL_H_
#define INCLUDE_FKIE_MESSAGE_FILTERS_PUBLISHER_IMPL_H_

#include "publisher.h"
#include <ros/advertise_options.h>

namespace fkie_message_filters
{

template<class M, template<typename> class Translate>
Publisher<M, Translate>::Publisher() noexcept
{
}

template<class M, template<typename> class Translate>
Publisher<M, Translate>::Publisher(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size, bool latch, ros::CallbackQueueInterface* callback_queue) noexcept
{
    advertise(nh, topic, queue_size, latch, callback_queue);
}

template<class M, template<typename> class Translate>
bool Publisher<M, Translate>::is_active() const noexcept
{
    return pub_.getNumSubscribers() > 0;
}

template<class M, template<typename> class Translate>
std::string Publisher<M, Translate>::topic() const noexcept
{
    return pub_.getTopic();
}

template<class M, template<typename> class Translate>
void Publisher<M, Translate>::advertise(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size, bool latch, ros::CallbackQueueInterface* callback_queue) noexcept
{
    ros::AdvertiseOptions opts;
    opts.init<M>(topic, queue_size,
            [this](const ros::SingleSubscriberPublisher&)
            {
                this->update_subscriber_state();
            },
            [this](const ros::SingleSubscriberPublisher&)
            {
                this->update_subscriber_state();
            }
    );
    opts.latch = latch;
    opts.callback_queue = callback_queue;
    pub_ = nh.advertise(opts);
    update_subscriber_state();
}

template<class M, template<typename> class Translate>
void Publisher<M, Translate>::advertise(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size, const ros::SubscriberStatusCallback& connect_cb, const ros::SubscriberStatusCallback& disconnect_cb, const ros::VoidConstPtr& tracked_object, bool latch, ros::CallbackQueueInterface* callback_queue) noexcept
{
    ros::AdvertiseOptions opts;
    opts.init<M>(topic, queue_size,
            [this, connect_cb](const ros::SingleSubscriberPublisher& ssp)
            {
                this->update_subscriber_state();
                if (connect_cb) connect_cb(ssp);
            },
            [this, disconnect_cb](const ros::SingleSubscriberPublisher& ssp)
            {
                this->update_subscriber_state();
                if (disconnect_cb) disconnect_cb(ssp);
            }
    );
    opts.latch = latch;
    opts.callback_queue = callback_queue;
    opts.tracked_object = tracked_object;
    pub_ = nh.advertise(opts);
    update_subscriber_state();
}    

template<class M, template<typename> class Translate>
void Publisher<M, Translate>::receive (const typename Translate<M>::FilterType& m) noexcept
{
    pub_.publish(Translate<M>::filterToPublish(m));
}

} // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_PUBLISHER_IMPL_H_ */
