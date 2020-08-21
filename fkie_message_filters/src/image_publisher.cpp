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

#include <fkie_message_filters/image_publisher.h>

namespace fkie_message_filters
{

ImagePublisher::ImagePublisher(const image_transport::ImageTransport& it, const std::string& base_topic, uint32_t queue_size, bool latch) noexcept
{
    advertise(it, base_topic, queue_size, latch);
}

bool ImagePublisher::is_active() const noexcept
{
    return pub_.getNumSubscribers() > 0;
}

std::string ImagePublisher::topic() const noexcept
{
    return pub_.getTopic();
}

void ImagePublisher::advertise(const image_transport::ImageTransport& it, const std::string& base_topic, uint32_t queue_size, bool latch) noexcept
{
    it_ = std::make_shared<image_transport::ImageTransport>(it);
    pub_ = it_->advertise(
        base_topic, queue_size,
        [this](const image_transport::SingleSubscriberPublisher&)
        {
            this->update_subscriber_state();
        },
        [this](const image_transport::SingleSubscriberPublisher&)
        {
            this->update_subscriber_state();
        },
        ros::VoidPtr(),
        latch
    );
    update_subscriber_state();
}

void ImagePublisher::advertise (const image_transport::ImageTransport& it, const std::string& base_topic, uint32_t queue_size, const image_transport::SubscriberStatusCallback& connect_cb, const image_transport::SubscriberStatusCallback& disconnect_cb, const ros::VoidPtr& tracked_object, bool latch) noexcept
{
    it_ = std::make_shared<image_transport::ImageTransport>(it);
    pub_ = it_->advertise(
        base_topic, queue_size,
        [this, connect_cb](const image_transport::SingleSubscriberPublisher& ssp)
        {
            this->update_subscriber_state();
            if (connect_cb) connect_cb(ssp);
        },
        [this, disconnect_cb](const image_transport::SingleSubscriberPublisher& ssp)
        {
            this->update_subscriber_state();
            if (disconnect_cb) disconnect_cb(ssp);
        },
        tracked_object,
        latch
    );
    update_subscriber_state();
}

void ImagePublisher::receive (const sensor_msgs::ImageConstPtr& img) noexcept
{
    pub_.publish(img);
}

} // namespace fkie_message_filters
