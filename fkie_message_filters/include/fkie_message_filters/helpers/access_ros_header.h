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

#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_HELPERS_ACCESS_ROS_HEADER_H_
#define INCLUDE_FKIE_MESSAGE_FILTERS_HELPERS_ACCESS_ROS_HEADER_H_

#include <memory>
#include <boost/shared_ptr.hpp>
#include <ros/message_event.h>
#include <ros/message_traits.h>
#include <ros/time.h>
#include <string>
#include <type_traits>

namespace fkie_message_filters
{
namespace helpers
{

template<class M>
struct AccessRosHeader
{
    static std::string frame_id(const M& m) noexcept { return ros::message_traits::FrameId<std::remove_cv_t<M>>::value(m); }
    static ros::Time stamp(const M& m) noexcept { return ros::message_traits::TimeStamp<std::remove_cv_t<M>>::value(m); }
};

template<class M>
struct AccessRosHeader<std::shared_ptr<M>>
{
    static std::string frame_id(const std::shared_ptr<M>& m) noexcept { return ros::message_traits::FrameId<std::remove_cv_t<M>>::value(*m); }
    static ros::Time stamp(const std::shared_ptr<M>& m) noexcept { return ros::message_traits::TimeStamp<std::remove_cv_t<M>>::value(*m); }
};

template<class M>
struct AccessRosHeader<boost::shared_ptr<M>>
{
    static std::string frame_id(const boost::shared_ptr<M>& m) noexcept { return ros::message_traits::FrameId<std::remove_cv_t<M>>::value(*m); }
    static ros::Time stamp(const boost::shared_ptr<M>& m) noexcept { return ros::message_traits::TimeStamp<std::remove_cv_t<M>>::value(*m); }
};

template<class M>
struct AccessRosHeader<ros::MessageEvent<M>>
{
    static std::string frame_id(const ros::MessageEvent<M>& m) noexcept { return ros::message_traits::FrameId<std::remove_cv_t<M>>::value(*m.getConstMessage()); }
    static ros::Time stamp(const ros::MessageEvent<M>& m) noexcept { return ros::message_traits::TimeStamp<std::remove_cv_t<M>>::value(*m.getConstMessage()); }
};

template<class M>
std::string access_ros_header_frame_id(const M& m) noexcept
{
    return AccessRosHeader<M>::frame_id(m);
}

template<class M>
ros::Time access_ros_header_stamp(const M& m) noexcept
{
    return AccessRosHeader<M>::stamp(m);
}

} // namespace helpers
} // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_HELPERS_ACCESS_ROS_HEADER_H_ */
