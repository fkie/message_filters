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
#include <fkie_message_filters/divider.h>
#include <fkie_message_filters/user_source.h>
#include <fkie_message_filters/simple_user_filter.h>

TEST(fkie_message_filters, Divider)
{
    using Source = mf::UserSource<int_M, double_M>;
    using Divider = mf::Divider<Source::Output>;
    using Sink1 = mf::SimpleUserFilter<int_M>;
    using Sink2 = mf::SimpleUserFilter<double_M>;

    std::size_t callback_counts = 0;
    Source src;
    Divider div;
    Sink1 snk1;
    Sink2 snk2;

    snk1.set_processing_function(
        [&](const int_M& i) -> bool
        {
            ++callback_counts;
            if (i != 42) throw std::domain_error("i != 42");
            return true;
        }
    );
    snk2.set_processing_function(
        [&](const double_M& d) -> bool
        {
            ++callback_counts;
            if (d != 3.14) throw std::domain_error("d != 3.14");
            return true;
        }
    );
    mf::chain(src, div);
    div.connect_to_sinks(snk1, snk2);
    src(int_M(42), double_M(3.14));
    ASSERT_EQ(2u, callback_counts);
}
