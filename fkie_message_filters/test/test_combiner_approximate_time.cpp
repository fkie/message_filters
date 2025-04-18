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
#include <fkie_message_filters/combiner_policies/approximate_time.hpp>
#include <fkie_message_filters/simple_user_filter.hpp>
#include <fkie_message_filters/user_source.hpp>

template<typename int_T>
void approximate_time_test_code()
{
    using IntegerStamped = Stamped<int_T>;
    using Source = mf::UserSource<IntegerStamped>;
    using Combiner = mf::Combiner<mf::combiner_policies::ApproximateTime, typename Source::Output,
                                  typename Source::Output, typename Source::Output>;
    using Sink = mf::SimpleUserFilter<typename Combiner::Output>;

    std::size_t callback_counts = 0;
    Source src1, src2, src3;
    Combiner combiner(typename Combiner::Policy().set_max_age(rclcpp::Duration(10, 0)));
    Sink snk;
    combiner.connect_to_sources(src1, src2, src3);
    combiner.connect_to_sink(snk);
    snk.set_processing_function(
        [&](const IntegerStamped& m1, const IntegerStamped& m2, const IntegerStamped& m3) -> bool
        {
            ++callback_counts;
            if (m1 != m2)
                throw std::domain_error("messages do not match");
            if (m1 != m3)
                throw std::domain_error("messages do not match");
            return true;
        });
    src1(IntegerStamped(0, "", make_stamp(99, 2)));
    src2(IntegerStamped(1, "", make_stamp(100, 1)));
    // Extra message
    src2(IntegerStamped(0, "", make_stamp(100, 20)));
    src3(IntegerStamped(1, "", make_stamp(100, 2)));
    src1(IntegerStamped(1, "", make_stamp(100, 0)));
    src1(IntegerStamped(2, "", make_stamp(101, 0)));
    src2(IntegerStamped(2, "", make_stamp(101, 2)));
    src3(IntegerStamped(2, "", make_stamp(101, 1)));
    ASSERT_EQ(1u, callback_counts);
    src1(IntegerStamped(3, "", make_stamp(102, 0)));
    src2(IntegerStamped(3, "", make_stamp(102, 0)));
    src3(IntegerStamped(3, "", make_stamp(102, 0)));
    ASSERT_EQ(3u, callback_counts);
    src1(IntegerStamped(4, "", make_stamp(103, 1)));
    src2(IntegerStamped(4, "", make_stamp(103, 0)));
    // missing message
    ASSERT_EQ(3u, callback_counts);
    src1(IntegerStamped(5, "", make_stamp(104, 2)));
    src2(IntegerStamped(5, "", make_stamp(104, 1)));
    src3(IntegerStamped(5, "", make_stamp(104, 0)));
    ASSERT_EQ(4u, callback_counts);
    /* Check that old messages get dropped */
    src1(IntegerStamped(0, "", make_stamp(105, 0)));
    src2(IntegerStamped(6, "", make_stamp(116, 0)));
    src3(IntegerStamped(6, "", make_stamp(116, 0)));
    src1(IntegerStamped(7, "", make_stamp(130, 0)));
    src2(IntegerStamped(7, "", make_stamp(130, 0)));
    src3(IntegerStamped(7, "", make_stamp(130, 0)));
    ASSERT_EQ(5u, callback_counts);
    /* Check that filter resets if earlier time stamps come in */
    src1(IntegerStamped(8, "", make_stamp(140, 0)));
    src2(IntegerStamped(8, "", make_stamp(140, 0)));
    src2(IntegerStamped(0, "", make_stamp(99, 0)));
    src3(IntegerStamped(8, "", make_stamp(140, 0)));
    ASSERT_EQ(5u, callback_counts);
}

TEST(fkie_message_filters, ApproximateTimeCombinerCopyConstructible)
{
    approximate_time_test_code<int_C>();
}

TEST(fkie_message_filters, ApproximateTimeCombinerMoveConstructible)
{
    approximate_time_test_code<int_M>();
}
