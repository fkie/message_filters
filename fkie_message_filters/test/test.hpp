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
#ifndef TEST_TEST_HPP_
#define TEST_TEST_HPP_
#pragma once

#include <gtest/gtest.h>  // IWYU pragma: export
#include <image_transport/image_transport.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/header.hpp>

#include <type_traits>

namespace fkie_message_filters
{
}

template<class T>
struct CopyConstructable
{
    CopyConstructable() = delete;
    explicit CopyConstructable(const T& t) : data_(t) {}
    CopyConstructable(const CopyConstructable&) = default;
    operator const T&() const
    {
        return data_;
    }
    bool operator==(const CopyConstructable& other) const
    {
        return data_ == other.data_;
    }
    bool operator!=(const CopyConstructable& other) const
    {
        return data_ != other.data_;
    }

private:
    T data_;
};

using int_C = CopyConstructable<int>;
using double_C = CopyConstructable<double>;
using string_C = CopyConstructable<std::string>;

static_assert(std::is_copy_constructible_v<int_C>);

template<class T>
struct MoveConstructable
{
    MoveConstructable() = delete;
    MoveConstructable(const MoveConstructable&) = delete;
    explicit MoveConstructable(const T& t) : data_(t), valid_(true) {}
    MoveConstructable(MoveConstructable&& other) : data_(std::move(other.data_)), valid_(other.valid_)
    {
        other.valid_ = false;
    }
    MoveConstructable& operator=(const MoveConstructable&) = delete;
    MoveConstructable& operator=(MoveConstructable&& other)
    {
        data_ = std::move(other.data_);
        valid_ = other.valid_;
        other.valid_ = false;
        return *this;
    }
    bool is_valid() const
    {
        return valid_;
    }

    operator const T&() const
    {
        return data_;
    }
    bool operator==(const MoveConstructable& other) const
    {
        return valid_ && other.valid_ && data_ == other.data_;
    }
    bool operator!=(const MoveConstructable& other) const
    {
        return !valid_ || !other.valid_ || data_ != other.data_;
    }

private:
    T data_;
    bool valid_;
};

using int_M = MoveConstructable<int>;
using double_M = MoveConstructable<double>;
using string_M = MoveConstructable<std::string>;

static_assert(!std::is_copy_constructible_v<int_M>);
static_assert(std::is_move_constructible_v<int_M>);

template<class T>
struct Stamped : public T
{
    template<class U, std::enable_if_t<std::is_constructible_v<T, U>, bool> = true>
    explicit Stamped(const U& u, const std::string& frame_id = std::string(),
                     const rclcpp::Time& stamp = rclcpp::Time(0, 0, RCL_ROS_TIME))
        : T(u)
    {
        header.frame_id = frame_id;
        header.stamp = stamp;
    }
    std_msgs::msg::Header header;
};

inline rclcpp::Time make_stamp(int32_t seconds, uint32_t nanoseconds = 0)
{
    return rclcpp::Time(seconds, nanoseconds, RCL_ROS_TIME);
}

class ExpectedException : public std::runtime_error
{
public:
    ExpectedException(const char* what) : std::runtime_error(what) {}
};

template<class PublisherT>
std::size_t get_subscription_count(const std::shared_ptr<PublisherT>& pub)
{
    return pub->get_subscription_count();
}

inline std::size_t get_subscription_count(image_transport::Publisher& pub)
{
    return pub.getNumSubscribers();
}

inline std::size_t get_subscription_count(image_transport::CameraPublisher& pub)
{
    return pub.getNumSubscribers();
}

template<class SubscriberT>
std::size_t get_publisher_count(std::shared_ptr<SubscriberT>& sub)
{
    return sub->get_publisher_count();
}

inline std::size_t get_publisher_count(image_transport::Subscriber& sub)
{
    return sub.getNumPublishers();
}

inline std::size_t get_publisher_count(image_transport::CameraSubscriber& sub)
{
    return sub.getNumPublishers();
}

template<class PublisherT, class... MessagesT>
void publish(std::shared_ptr<PublisherT>& pub, MessagesT&&... msgs)
{
    pub->publish(std::forward<MessagesT&&>(msgs)...);
}

template<class... MessagesT>
void publish(image_transport::Publisher& pub, MessagesT&&... msgs)
{
    pub.publish(std::forward<MessagesT&&>(msgs)...);
}

template<class... MessagesT>
void publish(image_transport::CameraPublisher& pub, MessagesT&&... msgs)
{
    pub.publish(std::forward<MessagesT&&>(msgs)...);
}

namespace mf = fkie_message_filters;

#endif /* TEST_TEST_HPP_ */
