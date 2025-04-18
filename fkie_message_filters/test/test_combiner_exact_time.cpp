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

#include <fkie_message_filters/combiner.hpp>
#include <fkie_message_filters/combiner_policies/exact_time.hpp>
#include <fkie_message_filters/simple_user_filter.hpp>
#include <fkie_message_filters/user_source.hpp>

template<typename int_T>
void exact_time_test_code()
{
    using IntegerStamped = Stamped<int_T>;
    using Source = mf::UserSource<IntegerStamped>;
    using Combiner = mf::Combiner<mf::combiner_policies::ExactTime, typename Source::Output, typename Source::Output>;
    using Sink = mf::SimpleUserFilter<typename Combiner::Output>;

    std::size_t callback_counts = 0;
    Source src1, src2;
    Combiner combiner(typename Combiner::Policy().set_max_age(rclcpp::Duration(10, 0)));
    Sink snk;
    combiner.connect_to_sources(src1, src2);
    combiner.connect_to_sink(snk);
    snk.set_processing_function(
        [&](const IntegerStamped& m1, const IntegerStamped& m2) -> bool
        {
            ++callback_counts;
            if (m1 != 0 || m2 != 0)
                throw std::invalid_argument("expected only messages with zero value");
            if (m1.header.stamp != m2.header.stamp)
                throw std::invalid_argument("timestamps do not match");
            return true;
        });
    // Check that matching messages will be passed together
    src1(IntegerStamped(0, "", make_stamp(11)));
    src1(IntegerStamped(0, "", make_stamp(15)));
    src1(IntegerStamped(0, "", make_stamp(19)));
    src1(IntegerStamped(0, "", make_stamp(22)));
    ASSERT_EQ(0u, callback_counts);
    src2(IntegerStamped(0, "", make_stamp(11)));
    ASSERT_EQ(0u, callback_counts);
    src1(IntegerStamped(1, "", make_stamp(15)));  // Duplicate, gets ignored
    src2(IntegerStamped(0, "", make_stamp(15)));
    ASSERT_EQ(1u, callback_counts);
    src2(IntegerStamped(0, "", make_stamp(18)));
    ASSERT_EQ(1u, callback_counts);
    src2(IntegerStamped(0, "", make_stamp(19)));
    ASSERT_EQ(2u, callback_counts);
    src2(IntegerStamped(0, "", make_stamp(23)));
    ASSERT_EQ(2u, callback_counts);
    src1(IntegerStamped(0, "", make_stamp(23)));
    ASSERT_EQ(3u, callback_counts);
    // Check that old messages will be discarded after max age
    src1(IntegerStamped(0, "", make_stamp(30)));
    src1(IntegerStamped(0, "", make_stamp(50)));
    src2(IntegerStamped(0, "", make_stamp(30)));
    ASSERT_EQ(3u, callback_counts);
    // Check that old messages will be discarded after successful match
    src1(IntegerStamped(0, "", make_stamp(60)));
    src1(IntegerStamped(0, "", make_stamp(61)));
    src2(IntegerStamped(0, "", make_stamp(61)));
    src2(IntegerStamped(0, "", make_stamp(60)));
    ASSERT_EQ(4u, callback_counts);
}

TEST(fkie_message_filters, ExactTimeCombinerCopyConstructible)
{
    exact_time_test_code<int_C>();
}

TEST(fkie_message_filters, ExactTimeCombinerMoveConstructible)
{
    exact_time_test_code<int_M>();
}
