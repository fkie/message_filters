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
#include "test.h"
#include <fkie_message_filters/helpers/access_ros_header.h>
#include <fkie_message_filters/helpers/io.h>
#include <fkie_message_filters/helpers/tuple.h>
#include <boost/shared_ptr.hpp>
#include <memory>
#include <ros/message_event.h>

static std::size_t seq_helper(std::size_t a, std::size_t b, std::size_t c)
{
    return a + b + c;
}

TEST(fkie_message_filters, SequenceHelpers)
{
    std::size_t result = 0;
    mf::helpers::for_each_apply<4>(
        [&](auto Is)
        {
            result += Is;
        }
    );
    ASSERT_EQ(6u, result);
    mf::helpers::for_each_apply<4>(
        [&](auto Is)
        {
            result += Is;
        }
    );
    result = mf::helpers::index_apply<3>(
        [](auto... Is) -> std::size_t
        {
            return seq_helper(Is...);
        }
    );
    ASSERT_EQ(3u, result);
    result = 0;
    mf::helpers::select_apply<4>(2,
        [&](auto Is)
        {
            result += Is;
        }
    );
    ASSERT_EQ(2u, result);
}

TEST(fkie_message_filters, RosHeaderExtraction)
{
    // Verify that the access_ros_header() helper function works with all supported data types
    using IntegerStamped = Stamped<int>;

    IntegerStamped i1{0, std::string(), ros::Time(1, 0)};
    std::shared_ptr<IntegerStamped const> i2 = std::make_shared<IntegerStamped>(0, std::string(), ros::Time(2, 0));
    boost::shared_ptr<IntegerStamped const> i3 = boost::make_shared<IntegerStamped>(0, std::string(), ros::Time(3, 0));
    ros::MessageEvent<IntegerStamped const> i4{boost::make_shared<IntegerStamped>(0, std::string(), ros::Time(4, 0)), boost::shared_ptr<ros::M_string>(), ros::Time(4, 0), false, []() -> boost::shared_ptr<IntegerStamped> { return boost::shared_ptr<IntegerStamped>();}};

    ASSERT_EQ(ros::Time(1,0), mf::helpers::access_ros_header_stamp(i1));
    ASSERT_EQ(ros::Time(2,0), mf::helpers::access_ros_header_stamp(i2));
    ASSERT_EQ(ros::Time(3,0), mf::helpers::access_ros_header_stamp(i3));
    ASSERT_EQ(ros::Time(4,0), mf::helpers::access_ros_header_stamp(i4));
}
