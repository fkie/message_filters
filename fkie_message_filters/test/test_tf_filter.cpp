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
#include <fkie_message_filters/tf_filter.h>
#include <fkie_message_filters/user_source.h>
#include <fkie_message_filters/simple_user_filter.h>

static void set_tf_transform(tf2::BufferCore& bc, const std::string& target_frame, const std::string& source_frame, const ros::Time& time)
{
    geometry_msgs::TransformStamped tr;
    tr.header.frame_id = target_frame; // parent
    tr.header.stamp = time;
    tr.child_frame_id = source_frame; // child
    tr.transform.rotation.w = 1;
    bc.setTransform(tr, "gtest", false);
}

TEST(fkie_message_filters, TfFilter)
{
    using IntegerStamped = Stamped<int>;
    using Source = mf::UserSource<IntegerStamped>;
    using TfFilter = mf::TfFilter<Source::Output>;
    using Sink = mf::SimpleUserFilter<TfFilter::Output>;

    std::size_t callback_counts = 0, failure_counts = 0;
    tf2::BufferCore bc{ros::Duration(10, 0)};
    Source src;
    TfFilter flt;
    Sink snk;
    flt.init(bc, 2, nullptr);
    flt.set_target_frame("target");
    flt.set_filter_failure_function(
        [&](const IntegerStamped& i, mf::TfFilterResult reason)
        {
            if (reason != mf::TfFilterResult::TransformAvailable) ++failure_counts;
        }
    );
    mf::chain(src, flt, snk);
    snk.set_processing_function(
        [&](const IntegerStamped& i) -> bool
        {
            ++callback_counts;
            std::string err;
            if (!bc.canTransform("target", i.header.frame_id, i.header.stamp, &err)) throw std::logic_error(err.c_str());
            return true;
        }
    );
    set_tf_transform(bc, "target", "alpha", ros::Time(99, 0));
    set_tf_transform(bc, "target", "beta", ros::Time(99, 0));
    // Check that filter will wait when the transform is older than needed
    src(IntegerStamped(0, "alpha", ros::Time(100, 0)));
    ASSERT_EQ(0u, callback_counts);
    ASSERT_EQ(0u, failure_counts);
    // Check that filter pass the message once the transform updates
    set_tf_transform(bc, "target", "alpha", ros::Time(101, 0));
    ASSERT_EQ(1u, callback_counts);
    ASSERT_EQ(0u, failure_counts);
    // If the message is older than the cache length, instant fail
    src(IntegerStamped(0, "alpha", ros::Time(50, 0)));
    ASSERT_EQ(1u, callback_counts);
    ASSERT_EQ(1u, failure_counts);
    // If the transform is available, instant pass
    src(IntegerStamped(0, "alpha", ros::Time(100, 0)));
    ASSERT_EQ(2u, callback_counts);
    ASSERT_EQ(1u, failure_counts);
    // The filter will wait for unknown transforms
    src(IntegerStamped(0, "gamma", ros::Time(102, 0)));
    ASSERT_EQ(1u, failure_counts);
    // If the queue overflows, the oldest message will be discarded
    src(IntegerStamped(0, "beta", ros::Time(102, 0)));
    src(IntegerStamped(0, "alpha", ros::Time(102, 0)));
    ASSERT_EQ(2u, callback_counts);
    ASSERT_EQ(2u, failure_counts);
    // Check that the remaining messages pass
    set_tf_transform(bc, "target", "alpha", ros::Time(103, 0));
    set_tf_transform(bc, "target", "beta", ros::Time(103, 0));
    ASSERT_EQ(4u, callback_counts);
    ASSERT_EQ(2u, failure_counts);
    // Fill the queue with messages, nothing should happen
    src(IntegerStamped(0, "gamma", ros::Time(100, 0)));
    src(IntegerStamped(0, "gamma", ros::Time(100, 0)));
    ASSERT_EQ(4u, callback_counts);
    ASSERT_EQ(2u, failure_counts);
    // Clear the queue
    flt.reset();
    // Check that filter still works and discarded the previous queue without failure notifications
    src(IntegerStamped(0, "alpha", ros::Time(100, 0)));
    src(IntegerStamped(0, "beta", ros::Time(100, 0)));
    ASSERT_EQ(6u, callback_counts);
    ASSERT_EQ(2u, failure_counts);
}
