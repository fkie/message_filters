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
#include <fkie_message_filters/user_filter.h>
#include <fkie_message_filters/user_source.h>
#include <fkie_message_filters/simple_user_filter.h>

TEST(fkie_message_filters, UserFilter)
{
    using Source = mf::UserSource<int_M>;
    using Sink = mf::SimpleUserFilter<double_M>;
    using Filter = mf::UserFilter<Source::Output, Sink::Input>;

    std::size_t callback_counts = 0;
    Source src;
    Filter flt;
    Sink snk;
    mf::chain(src, flt, snk);
    flt.set_processing_function(
        [](const int_M& i, const Filter::CallbackFunction& f)
        {
            if (i == 42) f(double_M(3.14));
        }
    );
    snk.set_processing_function(
        [&](const double_M& d) -> bool
        {
            ++callback_counts;
            if (d != 3.14) throw std::domain_error("d != 3.14");
            return true;
        }
    );
    // Check that 42 is passed through and -1 is discarded
    src(int_M(42));
    src(int_M(-1));
    ASSERT_EQ(1u, callback_counts);
}
