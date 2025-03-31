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

#include <fkie_message_filters/sequencer.hpp>
#include <fkie_message_filters/simple_user_filter.hpp>
#include <fkie_message_filters/user_source.hpp>

template<typename int_T>
void sequencer_test_code()
{
    using IntegerStamped = Stamped<int_T>;
    using Source = mf::UserSource<IntegerStamped>;
    using Sequencer = mf::Sequencer<typename Source::Output>;
    using Sink = mf::SimpleUserFilter<typename Source::Output>;

    std::size_t callback_counts = 0;
    rclcpp::Time last_ts{make_stamp(0)};
    Source src;
    Sequencer seq(rclcpp::Duration(10, 0));
    Sink snk;
    snk.set_processing_function(
        [&](const IntegerStamped& i) -> bool
        {
            ++callback_counts;
            if (rclcpp::Time(i.header.stamp) < last_ts)
                throw std::logic_error("Time stamp order violated");
            last_ts = i.header.stamp;
            return true;
        });
    mf::chain(src, seq, snk);

    src(IntegerStamped(0, "", make_stamp(100)));
    src(IntegerStamped(0, "", make_stamp(95)));
    src(IntegerStamped(0, "", make_stamp(98)));
    src(IntegerStamped(0, "", make_stamp(92)));
    src(IntegerStamped(0, "", make_stamp(91)));
    src(IntegerStamped(0, "", make_stamp(97)));
    src(IntegerStamped(0, "", make_stamp(50)));
    ASSERT_EQ(0u, callback_counts);
    src(IntegerStamped(0, "", make_stamp(105)));
    ASSERT_EQ(3u, callback_counts);
    src(IntegerStamped(0, "", make_stamp(120)));
    ASSERT_EQ(7u, callback_counts);
    ASSERT_EQ(make_stamp(105), last_ts);
    seq.flush();
    ASSERT_EQ(8u, callback_counts);
    src(IntegerStamped(0, "", make_stamp(119)));
    seq.flush();
    ASSERT_EQ(8u, callback_counts);
}

TEST(fkie_message_filters, SequencerCopyConstructible)
{
    sequencer_test_code<int_C>();
}

TEST(fkie_message_filters, SequencerMoveConstructible)
{
    sequencer_test_code<int_M>();
}
