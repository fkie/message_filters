/****************************************************************************
 *
 * fkie_message_filters
 * Copyright © 2018-2025 Fraunhofer FKIE
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
#include "test.hpp"
#include "version.hpp"

#include <fkie_message_filters/buffer.hpp>
#include <fkie_message_filters/simple_user_filter.hpp>
#include <fkie_message_filters/user_source.hpp>
#include <rclcpp/executors.hpp>

#include <chrono>

template<typename int_T>
void buffer_test_code()
{
    using Source = mf::UserSource<int_T>;
    using Buffer = mf::Buffer<int_T>;
    using Sink = mf::SimpleUserFilter<int_T>;

    std::size_t callback_counts = 0;
    Source src;
    Buffer buf(mf::BufferPolicy::Queue, 3);
    Sink snk;
    snk.set_processing_function(
        [&](const int_T& i) -> bool
        {
            ++callback_counts;
            if (i != 0)
                throw std::domain_error("invalid value");
            return true;
        });
    mf::chain(src, buf, snk);
    ASSERT_FALSE(buf.has_some());
    // Purposely overflow the buffer queue to verify that the first item gets
    // discarded
    src(int_T(10000));
    src(int_T(0));
    src(int_T(0));
    src(int_T(0));
    ASSERT_TRUE(buf.has_some());
    ASSERT_EQ(0u, callback_counts);
    // Check manual processing functions
    buf.process_one();
    ASSERT_EQ(1u, callback_counts);
    buf.spin_once();
    ASSERT_FALSE(buf.has_some());
    ASSERT_EQ(3u, callback_counts);
    buf.spin_once();
    ASSERT_EQ(3u, callback_counts);
    src(int_T(0));
    buf.spin_once();
    ASSERT_EQ(4u, callback_counts);
}

TEST(fkie_message_filters, BufferCopyConstructible)
{
    buffer_test_code<int_C>();
}

TEST(fkie_message_filters, BufferMoveConstructible)
{
    buffer_test_code<int_M>();
}

template<class Buffer, class Rep, class Period>
bool wait_for_buffer_processing(rclcpp::Node::SharedPtr& node, Buffer& buffer,
                                const std::chrono::duration<Rep, Period>& timeout)
{
#if FKIE_MF_RCLCPP_VERSION >= FKIE_MF_VERSION_TUPLE(22, 1, 0)
    if (buffer.has_some())
    {
        rclcpp::spin_all(node, std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));
    }
#else
    std::chrono::system_clock::time_point deadline = std::chrono::system_clock::now() + timeout;
    while (buffer.has_some() && std::chrono::system_clock::now() < deadline)
    {
        rclcpp::spin_some(node);
    }
#endif
    return !buffer.has_some();
}

TEST(fkie_message_filters, BufferCallbackGroup)
{
    using namespace std::chrono_literals;
    using Source = mf::UserSource<int_C>;
    using Buffer = mf::Buffer<int_C>;
    using Sink = mf::SimpleUserFilter<int_C>;

    std::size_t callback_counts = 0;
    int last_value = 0;
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("buffer_callback_group");
    Source src;
    Buffer buf(node, 3);
    Sink snk;
    snk.set_processing_function(
        [&callback_counts, &last_value](const int_C& i) -> bool
        {
            ++callback_counts;
            if (i < last_value)
                throw std::domain_error("invalid value");
            last_value = i;
            return true;
        });
    mf::chain(src, buf, snk);
    src(int_C(0));  // Note: this will be discarded before processing
    src(int_C(1));
    src(int_C(2));
    src(int_C(3));
    ASSERT_TRUE(wait_for_buffer_processing(node, buf, 1s));
    ASSERT_EQ(3u, callback_counts);
    ASSERT_EQ(3, last_value);
    src(int_C(4));
    src(int_C(5));
    src(int_C(6));
    ASSERT_TRUE(wait_for_buffer_processing(node, buf, 1s));
    ASSERT_EQ(6u, callback_counts);
    ASSERT_EQ(6, last_value);
}
