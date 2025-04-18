/****************************************************************************
 *
 * fkie_message_filters
 * Copyright © 2018-2025 Fraunhofer FKIE
 * Author: Timo Röhling
 * SPDX-License-Identifier: Apache-2.0
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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_HELPERS_ACCESS_ROS_HEADER_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_HELPERS_ACCESS_ROS_HEADER_HPP_
#pragma once

#include "abi_namespace.hpp"

#if __has_include(<boost/shared_ptr.hpp>)
#    include <boost/shared_ptr.hpp>
#    define FKIE_MF_HAS_BOOST 1
#else
#    define FKIE_MF_HAS_BOOST 0
#endif

#include <rclcpp/time.hpp>

#include <memory>
#include <string>
namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

namespace helpers
{

template<class M>
struct AccessRosHeader
{
    static std::string frame_id(const M& m) noexcept
    {
        return m.header.frame_id;
    }
    static rclcpp::Time stamp(const M& m) noexcept
    {
        return rclcpp::Time(m.header.stamp, RCL_ROS_TIME);
    }
};

template<class M>
struct AccessRosHeader<std::shared_ptr<M>>
{
    static std::string frame_id(const std::shared_ptr<M>& m) noexcept
    {
        return m->header.frame_id;
    }
    static rclcpp::Time stamp(const std::shared_ptr<M>& m) noexcept
    {
        return rclcpp::Time(m->header.stamp, RCL_ROS_TIME);
    }
};

template<class M>
struct AccessRosHeader<std::unique_ptr<M>>
{
    static std::string frame_id(const std::unique_ptr<M>& m) noexcept
    {
        return m->header.frame_id;
    }
    static rclcpp::Time stamp(const std::unique_ptr<M>& m) noexcept
    {
        return rclcpp::Time(m->header.stamp, RCL_ROS_TIME);
    }
};

#if FKIE_MF_HAS_BOOST
template<class M>
struct AccessRosHeader<boost::shared_ptr<M>>
{
    static std::string frame_id(const boost::shared_ptr<M>& m) noexcept
    {
        return m->header.frame_id;
    }
    static rclcpp::Time stamp(const boost::shared_ptr<M>& m) noexcept
    {
        return rclcpp::Time(m->header.stamp, RCL_ROS_TIME);
    }
};
#endif

template<class M>
std::string access_ros_header_frame_id(const M& m) noexcept
{
    return AccessRosHeader<M>::frame_id(m);
}

template<class M>
rclcpp::Time access_ros_header_stamp(const M& m) noexcept
{
    return AccessRosHeader<M>::stamp(m);
}

}  // namespace helpers
FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_HELPERS_ACCESS_ROS_HEADER_HPP_ */
