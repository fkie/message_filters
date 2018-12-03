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

#include <fkie_message_filters/camera_publisher.h>

namespace fkie_message_filters
{

CameraPublisher::CameraPublisher(const image_transport::ImageTransport& it, const std::string& base_topic, uint32_t queue_size, bool latch) noexcept
{
    advertise(it, base_topic, queue_size, latch);
}

bool CameraPublisher::is_active() const noexcept
{
    return pub_.getNumSubscribers() > 0;
}

std::string CameraPublisher::topic() const noexcept
{
    return pub_.getTopic();
}

void CameraPublisher::advertise(const image_transport::ImageTransport& it, const std::string& base_topic, uint32_t queue_size, bool latch) noexcept
{
    it_ = std::make_shared<image_transport::ImageTransport>(it);
    pub_ = it_->advertiseCamera(
        base_topic, queue_size,
        [this](const image_transport::SingleSubscriberPublisher&)
        {
            this->update_subscriber_state();
        },
        [this](const image_transport::SingleSubscriberPublisher&)
        {
            this->update_subscriber_state();
        },
        [this](const ros::SingleSubscriberPublisher&)
        {
            this->update_subscriber_state();
        },
        [this](const ros::SingleSubscriberPublisher&)
        {
            this->update_subscriber_state();
        },
        ros::VoidPtr(),
        latch
    );
    update_subscriber_state();
}

void CameraPublisher::receive (const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& info) noexcept
{
    pub_.publish(img, info);
}

} // namespace fkie_message_filters
