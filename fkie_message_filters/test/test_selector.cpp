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
#include <fkie_message_filters/selector.h>
#include <fkie_message_filters/user_source.h>
#include <fkie_message_filters/simple_user_filter.h>

TEST(fkie_message_filters, Selector)
{
    using Source = mf::UserSource<int_M, double_M, string_M>;
    using Select = mf::Selector<Source::Output, 1>;
    using Sink = mf::SimpleUserFilter<double_M>;

    std::size_t callback_counts = 0;
    Source src;
    Select select;
    Sink snk;
    snk.set_processing_function(
        [&](const double_M& d) -> bool
        {
            ++callback_counts;
            if (d == 2.5) throw ExpectedException("d == 2.5");
            if (d != 5.0) throw std::domain_error("d != 5.0");
            return true;
        }
    );
    mf::chain(src, select, snk);
    src(int_M(1), double_M(5.0), string_M("abc"));
    ASSERT_THROW(src(int_M(0), double_M(2.5), string_M("")), ExpectedException);
    ASSERT_EQ(2u, callback_counts);
}
