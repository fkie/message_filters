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

#ifndef TEST_TEST_H_
#define TEST_TEST_H_

#define FKIE_MESSAGE_FILTERS_IGNORE_ROS_OK

#include <gtest/gtest.h>
#include <std_msgs/Header.h>
#include <ros/message_traits.h>

namespace fkie_message_filters
{
}

template<class T>
struct NotDefaultConstructable
{
    NotDefaultConstructable() = delete;
    explicit NotDefaultConstructable(const T& t) : data(t) {}
    NotDefaultConstructable(const NotDefaultConstructable&) = default;
    operator const T() const { return data; }
    bool operator == (const NotDefaultConstructable& other) const { return data == other.data; }
    bool operator != (const NotDefaultConstructable& other) const { return data != other.data; }
private:
    T data;
};

using int_M = NotDefaultConstructable<int>;
using double_M = NotDefaultConstructable<double>;
using string_M = NotDefaultConstructable<std::string>;

template<class T>
struct Stamped : public NotDefaultConstructable<T>
{
    explicit Stamped (const T& t, const std::string& frame_id = std::string(), const ros::Time& stamp = ros::Time())
    : NotDefaultConstructable<T>(t)
    {
        header.frame_id = frame_id;
        header.stamp = stamp;
    }
    std_msgs::Header header;
};

namespace ros
{
namespace message_traits
{

template<class T>
struct HasHeader<Stamped<T>> : public TrueType {};

} // namespace message_traits
} // namespace ros

class ExpectedException : public std::runtime_error
{
public:
    ExpectedException(const char* what) : std::runtime_error(what) {}
};


namespace mf = fkie_message_filters;

#endif /* TEST_TEST_H_ */
