/****************************************************************************
 *
 * fkie_message_filters
 * Copyright © 2018 Fraunhofer FKIE
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

#include <fkie_message_filters/image_subscriber.h>

namespace fkie_message_filters
{

ImageSubscriber::ImageSubscriber(const image_transport::ImageTransport& it, const std::string& base_topic, uint32_t queue_size, const image_transport::TransportHints& transport_hints) noexcept
{
    set_subscribe_options(it, base_topic, queue_size, transport_hints);
    subscribe();
}

void ImageSubscriber::set_subscribe_options(const image_transport::ImageTransport& it, const std::string& base_topic, uint32_t queue_size, const image_transport::TransportHints& transport_hints) noexcept
{
    it_ = std::make_shared<image_transport::ImageTransport>(it);
    base_topic_ = base_topic;
    queue_size_ = queue_size;
    hints_ = transport_hints;
}

void ImageSubscriber::subscribe(const image_transport::ImageTransport& it, const std::string& base_topic, uint32_t queue_size, const image_transport::TransportHints& transport_hints) noexcept
{
    set_subscribe_options(it, base_topic, queue_size, transport_hints);
    subscribe();
}

std::string ImageSubscriber::topic() const noexcept
{
    return sub_.getTopic();
}

bool ImageSubscriber::is_configured() const noexcept
{
    return it_ && !base_topic_.empty();
}

void ImageSubscriber::subscribe_impl() noexcept
{
    if (!sub_)
    {
        sub_ = it_->subscribe(
            base_topic_, queue_size_,
            [this](const sensor_msgs::ImageConstPtr& m)
            {
                this->cb(m);
            },
            ros::VoidPtr(), hints_
        );
    }
}

void ImageSubscriber::unsubscribe_impl() noexcept
{
    sub_.shutdown();
}

void ImageSubscriber::cb (const sensor_msgs::ImageConstPtr& m)
{
    send(m);
}

} // namespace fkie_message_filters
