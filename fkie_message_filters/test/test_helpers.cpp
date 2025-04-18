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
#include "test.hpp"

#include <fkie_message_filters/helpers/access_ros_header.hpp>
#include <fkie_message_filters/helpers/io.hpp>
#include <fkie_message_filters/helpers/signaling.hpp>
#include <fkie_message_filters/helpers/tuple.hpp>
#include <fkie_message_filters/types.hpp>

#include <memory>
#if FKIE_MF_HAS_BOOST
#    include <boost/make_shared.hpp>
#    include <boost/shared_ptr.hpp>
#endif

static std::size_t seq_helper(std::size_t a, std::size_t b, std::size_t c)
{
    return a + b + c;
}

TEST(fkie_message_filters, SequenceHelpers)
{
    std::size_t result = 0;
    mf::helpers::for_each_apply<4>([&](auto Is) { result += Is; });
    ASSERT_EQ(6u, result);
    mf::helpers::for_each_apply<4>([&](auto Is) { result += Is; });
    result = mf::helpers::index_apply<3>([](auto... Is) -> std::size_t { return seq_helper(Is...); });
    ASSERT_EQ(3u, result);
    result = 0;
    mf::helpers::select_apply<4>(2, [&](auto Is) { result += Is; });
    ASSERT_EQ(2u, result);
}

TEST(fkie_message_filters, RosHeaderExtraction)
{
    // Verify that the access_ros_header() helper function works with all
    // supported data types
    using IntegerStamped = Stamped<int_C>;

    IntegerStamped i1{int_C(0), "frame1", rclcpp::Time(1, 0)};
    std::shared_ptr<IntegerStamped const> i2 = std::make_shared<IntegerStamped>(int_C(0), "frame2", rclcpp::Time(2, 0));
    std::unique_ptr<IntegerStamped const> i3 =
        std::make_unique<IntegerStamped const>(int_C(0), "frame3", rclcpp::Time(3, 0));
#if FKIE_MF_HAS_BOOST
    boost::shared_ptr<IntegerStamped const> i4 =
        boost::make_shared<IntegerStamped>(int_C(0), "frame4", rclcpp::Time(4, 0));
#endif
    ASSERT_EQ(rclcpp::Time(1, 0, RCL_ROS_TIME), mf::helpers::access_ros_header_stamp(i1));
    ASSERT_EQ("frame1", mf::helpers::access_ros_header_frame_id(i1));
    ASSERT_EQ(rclcpp::Time(2, 0, RCL_ROS_TIME), mf::helpers::access_ros_header_stamp(i2));
    ASSERT_EQ("frame2", mf::helpers::access_ros_header_frame_id(i2));
    ASSERT_EQ(rclcpp::Time(3, 0, RCL_ROS_TIME), mf::helpers::access_ros_header_stamp(i3));
    ASSERT_EQ("frame3", mf::helpers::access_ros_header_frame_id(i3));
#if FKIE_MF_HAS_BOOST
    ASSERT_EQ(rclcpp::Time(4, 0, RCL_ROS_TIME), mf::helpers::access_ros_header_stamp(i4));
    ASSERT_EQ("frame4", mf::helpers::access_ros_header_frame_id(i4));
#endif
}

TEST(fkie_message_filters, Signals)
{
    std::size_t triggered = 0;
    mf::Connection c;
    mf::helpers::Signal<int> sig1, sig2;
    ASSERT_FALSE(c.connected());
    c = sig1.connect(
        [&triggered](int i)
        {
            ASSERT_EQ(42, i);
            ++triggered;
        });
    ASSERT_TRUE(c.connected());
    ASSERT_EQ(0, triggered);
    sig1(42);
    ASSERT_EQ(1, triggered);
    sig2(0);
    ASSERT_EQ(1, triggered);
    sig2 = std::move(sig1);
    sig1(0);
    ASSERT_EQ(1, triggered);
    sig2(42);
    ASSERT_EQ(2, triggered);
    mf::helpers::Signal<int> sig3{std::move(sig2)};
    sig1(0);
    sig2(0);
    ASSERT_EQ(2, triggered);
    sig3(42);
    ASSERT_EQ(3, triggered);
    c.disconnect();
    sig1(0);
    sig2(0);
    sig3(0);
    ASSERT_EQ(3, triggered);
}

TEST(fkie_message_filters, SignalsWithReference)
{
    std::size_t triggered = 0;
    int i = 42;
    mf::helpers::Signal<int&> sig2;
    sig2.connect(
        [&triggered](int& i)
        {
            ASSERT_EQ(42, i);
            ++triggered;
        });
    ASSERT_EQ(0, triggered);
    sig2(i);
    ASSERT_EQ(1, triggered);
}

TEST(fkie_message_filters, SignalsWithUniquePtr)
{
    mf::helpers::Signal<std::unique_ptr<int>> sig3;
    sig3.connect([](std::unique_ptr<int> p) { ASSERT_EQ(42, *p); });
    ASSERT_THROW(sig3.connect([](std::unique_ptr<int> p) { ASSERT_EQ(0, *p); }), std::logic_error);
    std::unique_ptr<int> q = std::make_unique<int>(42);
    ASSERT_FALSE(!q);
    ASSERT_EQ(42, *q);
    sig3(std::move(q));
    ASSERT_TRUE(!q);
}
