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
#include <fkie_message_filters/sequencer.h>
#include <fkie_message_filters/user_source.h>
#include <fkie_message_filters/simple_user_filter.h>

TEST(fkie_message_filters, Sequencer)
{
    using IntegerStamped = Stamped<int>;
    using Source = mf::UserSource<IntegerStamped>;
    using Sequencer = mf::Sequencer<Source::Output>;
    using Sink = mf::SimpleUserFilter<Source::Output>;

    std::size_t callback_counts = 0;
    ros::Time last_ts;
    Source src;
    Sequencer seq(ros::Duration(10, 0));
    Sink snk;
    snk.set_processing_function(
        [&](const IntegerStamped& i) -> bool
        {
            ++callback_counts;
            if (i.header.stamp < last_ts) throw std::logic_error("Time stamp order violated");
            last_ts = i.header.stamp;
            return true;
        }
    );
    mf::chain(src, seq, snk);

    src(IntegerStamped(0, "", ros::Time(100, 0)));
    src(IntegerStamped(0, "", ros::Time(95, 0)));
    src(IntegerStamped(0, "", ros::Time(98, 0)));
    src(IntegerStamped(0, "", ros::Time(92, 0)));
    src(IntegerStamped(0, "", ros::Time(91, 0)));
    src(IntegerStamped(0, "", ros::Time(97, 0)));
    src(IntegerStamped(0, "", ros::Time(50, 0)));
    ASSERT_EQ(0u, callback_counts);
    src(IntegerStamped(0, "", ros::Time(105, 0)));
    ASSERT_EQ(3u, callback_counts);
    src(IntegerStamped(0, "", ros::Time(120, 0)));
    ASSERT_EQ(7u, callback_counts);
    ASSERT_EQ(ros::Time(105, 0), last_ts);
    seq.flush();
    ASSERT_EQ(8u, callback_counts);
    src(IntegerStamped(0, "", ros::Time(119, 0)));
    seq.flush();
    ASSERT_EQ(8u, callback_counts);
}
