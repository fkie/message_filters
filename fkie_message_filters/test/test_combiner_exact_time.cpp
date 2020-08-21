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
#include <fkie_message_filters/combiner_policies/exact_time.h>
#include <fkie_message_filters/combiner.h>
#include <fkie_message_filters/user_source.h>
#include <fkie_message_filters/simple_user_filter.h>

TEST(fkie_message_filters, ExactTimeCombiner)
{
    using IntegerStamped = Stamped<int>;
    using Source = mf::UserSource<IntegerStamped>;
    using Combiner = mf::Combiner<mf::combiner_policies::ExactTime, Source::Output, Source::Output>;
    using Sink = mf::SimpleUserFilter<Combiner::Output>;

    std::size_t callback_counts = 0;
    Source src1, src2;
    Combiner combiner(Combiner::Policy().set_max_age(ros::Duration(10, 0)));
    Sink snk;
    combiner.connect_to_sources(src1, src2);
    combiner.connect_to_sink(snk);
    snk.set_processing_function(
        [&](const IntegerStamped& m1, const IntegerStamped& m2) -> bool
        {
            ++callback_counts;
            if (m1.header.stamp != m2.header.stamp) throw std::domain_error("timestamps do not match");
            return true;
        }
    );
    // Check that matching messages will be passed together
    src1(IntegerStamped(0, "", ros::Time(11, 0)));
    src1(IntegerStamped(0, "", ros::Time(15, 0)));
    src1(IntegerStamped(0, "", ros::Time(19, 0)));
    src1(IntegerStamped(0, "", ros::Time(22, 0)));
    ASSERT_EQ(0u, callback_counts);
    src2(IntegerStamped(0, "", ros::Time(11, 0)));
    ASSERT_EQ(0u, callback_counts);
    src2(IntegerStamped(0, "", ros::Time(15, 0)));
    ASSERT_EQ(1u, callback_counts);
    src2(IntegerStamped(0, "", ros::Time(18, 0)));
    ASSERT_EQ(1u, callback_counts);
    src2(IntegerStamped(0, "", ros::Time(19, 0)));
    ASSERT_EQ(2u, callback_counts);
    src2(IntegerStamped(0, "", ros::Time(23, 0)));
    ASSERT_EQ(2u, callback_counts);
    src1(IntegerStamped(0, "", ros::Time(23, 0)));
    ASSERT_EQ(3u, callback_counts);
    // Check that old messages will be discarded after max age
    src1(IntegerStamped(0, "", ros::Time(30, 0)));
    src1(IntegerStamped(0, "", ros::Time(50, 0)));
    src2(IntegerStamped(0, "", ros::Time(30, 0)));
    ASSERT_EQ(3u, callback_counts);
    // Check that old messages will be discarded after successful match
    src1(IntegerStamped(0, "", ros::Time(60, 0)));
    src1(IntegerStamped(0, "", ros::Time(61, 0)));
    src2(IntegerStamped(0, "", ros::Time(61, 0)));
    src2(IntegerStamped(0, "", ros::Time(60, 0)));
    ASSERT_EQ(4u, callback_counts);
}

