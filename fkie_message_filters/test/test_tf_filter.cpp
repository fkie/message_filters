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

#include <fkie_message_filters/simple_user_filter.hpp>
#include <fkie_message_filters/tf_filter.hpp>
#include <fkie_message_filters/user_source.hpp>

static tf2::TimePoint tp_from_stamp(const builtin_interfaces::msg::Time& time)
{
    return tf2::TimePoint(std::chrono::nanoseconds(1000000000ull * time.sec + time.nanosec));
}

static void set_tf_transform(tf2::BufferCore& bc, const std::string& target_frame, const std::string& source_frame,
                             const rclcpp::Time& time)
{
    geometry_msgs::msg::TransformStamped tr;
    tr.header.frame_id = target_frame;  // parent
    tr.header.stamp = time;
    tr.child_frame_id = source_frame;  // child
    tr.transform.rotation.w = 1;
    bc.setTransform(tr, "gtest", false);
}

template<typename int_T>
void tf_filter_test_code()
{
    using IntegerStamped = Stamped<int_T>;
    using Source = mf::UserSource<IntegerStamped>;
    using TfFilter = mf::TfFilter<typename Source::Output>;
    using Sink = mf::SimpleUserFilter<typename TfFilter::Output>;
    using namespace std::chrono_literals;

    std::size_t callback_counts = 0, empty_counts = 0, expired_counts = 0, overflow_counts = 0, failure_counts = 0;
    tf2::BufferCore bc{10s};
    Source src;
    TfFilter flt;
    Sink snk;
    flt.init(bc, 2);
    flt.set_target_frame("target");
    flt.set_filter_failure_function(
        [&](const IntegerStamped& i, mf::TfFilterResult reason)
        {
            switch (reason)
            {
                case mf::TfFilterResult::TransformExpired:
                    ++expired_counts;
                    break;
                case mf::TfFilterResult::EmptyFrameID:
                    ++empty_counts;
                    break;
                case mf::TfFilterResult::QueueOverflow:
                    ++overflow_counts;
                    break;
                case mf::TfFilterResult::TransformAvailable:
                    throw std::logic_error("Transform available reported as failure");
                default:
                    ++failure_counts;
                    break;
            }
        });
    mf::chain(src, flt, snk);
    snk.set_processing_function(
        [&](const IntegerStamped& i) -> bool
        {
            ++callback_counts;
            std::string err;
            if (!bc.canTransform("target", i.header.frame_id, tp_from_stamp(i.header.stamp), &err))
                throw std::logic_error(err.c_str());
            return true;
        });
    set_tf_transform(bc, "target", "alpha", make_stamp(99));
    set_tf_transform(bc, "target", "beta", make_stamp(99));
    // Check that filter will wait when the transform is older than needed
    src(IntegerStamped(0, "alpha", make_stamp(100)));
    ASSERT_EQ(0u, callback_counts);
    ASSERT_EQ(0u, empty_counts);
    ASSERT_EQ(0u, expired_counts);
    ASSERT_EQ(0u, overflow_counts);
    ASSERT_EQ(0u, failure_counts);
    // Check that filter pass the message once the transform updates
    set_tf_transform(bc, "target", "alpha", make_stamp(101));
    ASSERT_EQ(1u, callback_counts);
    ASSERT_EQ(0u, empty_counts);
    ASSERT_EQ(0u, expired_counts);
    ASSERT_EQ(0u, overflow_counts);
    ASSERT_EQ(0u, failure_counts);
    // If the message is older than the cache length, instant fail
    src(IntegerStamped(0, "alpha", make_stamp(50)));
    ASSERT_EQ(1u, callback_counts);
    ASSERT_EQ(0u, empty_counts);
    ASSERT_EQ(1u, expired_counts);
    ASSERT_EQ(0u, overflow_counts);
    ASSERT_EQ(0u, failure_counts);
    // If the transform is available, instant pass
    src(IntegerStamped(0, "alpha", make_stamp(100)));
    ASSERT_EQ(2u, callback_counts);
    ASSERT_EQ(0u, empty_counts);
    ASSERT_EQ(1u, expired_counts);
    ASSERT_EQ(0u, overflow_counts);
    ASSERT_EQ(0u, failure_counts);
    // The filter will wait for unknown transforms
    src(IntegerStamped(0, "gamma", make_stamp(100)));
    ASSERT_EQ(2u, callback_counts);
    ASSERT_EQ(0u, empty_counts);
    ASSERT_EQ(1u, expired_counts);
    ASSERT_EQ(0u, overflow_counts);
    ASSERT_EQ(0u, failure_counts);
    // If the queue overflows, the oldest message will be discarded
    src(IntegerStamped(0, "beta", make_stamp(102)));
    src(IntegerStamped(0, "alpha", make_stamp(102)));
    ASSERT_EQ(2u, callback_counts);
    ASSERT_EQ(0u, empty_counts);
    ASSERT_EQ(1u, expired_counts);
    ASSERT_EQ(1u, overflow_counts);
    ASSERT_EQ(0u, failure_counts);
    // Check that the remaining messages pass
    set_tf_transform(bc, "target", "alpha", make_stamp(103));
    set_tf_transform(bc, "target", "beta", make_stamp(103));
    ASSERT_EQ(4u, callback_counts);
    ASSERT_EQ(0u, empty_counts);
    ASSERT_EQ(1u, expired_counts);
    ASSERT_EQ(1u, overflow_counts);
    ASSERT_EQ(0u, failure_counts);
    // Fill the queue with messages, nothing should happen
    src(IntegerStamped(0, "gamma", make_stamp(100)));
    src(IntegerStamped(0, "gamma", make_stamp(100)));
    ASSERT_EQ(4u, callback_counts);
    ASSERT_EQ(0u, empty_counts);
    ASSERT_EQ(1u, expired_counts);
    ASSERT_EQ(1u, overflow_counts);
    ASSERT_EQ(0u, failure_counts);
    // Clear the queue
    flt.reset();
    // Check that filter still works and discarded the previous queue without
    // failure notifications
    src(IntegerStamped(0, "alpha", make_stamp(100)));
    src(IntegerStamped(0, "beta", make_stamp(100)));
    ASSERT_EQ(6u, callback_counts);
    ASSERT_EQ(0u, empty_counts);
    ASSERT_EQ(1u, expired_counts);
    ASSERT_EQ(1u, overflow_counts);
    ASSERT_EQ(0u, failure_counts);
    src(IntegerStamped(0, "", make_stamp(100)));
    ASSERT_EQ(6u, callback_counts);
    ASSERT_EQ(1u, empty_counts);
    ASSERT_EQ(1u, expired_counts);
    ASSERT_EQ(1u, overflow_counts);
    ASSERT_EQ(0u, failure_counts);
}

TEST(fkie_message_filters, TfFilterCopyConstructible)
{
    tf_filter_test_code<int_C>();
}

TEST(fkie_message_filters, TfFilterMoveConstructible)
{
    tf_filter_test_code<int_M>();
}
