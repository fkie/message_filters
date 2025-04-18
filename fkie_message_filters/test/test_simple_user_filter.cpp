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

#include <fkie_message_filters/simple_user_filter.hpp>
#include <fkie_message_filters/user_source.hpp>

template<typename int_T>
void simple_user_filter_test_code()
{
    using Source = mf::UserSource<int_T>;
    using Filter = mf::SimpleUserFilter<int_T>;
    using Sink = mf::SimpleUserFilter<int_T>;

    std::size_t callback_counts = 0;
    Source src;
    Filter flt;
    Sink snk;
    mf::chain(src, flt, snk);
    flt.set_processing_function([](const int_T& i) -> bool { return i == 42; });
    snk.set_processing_function(
        [&](const int_T& i) -> bool
        {
            ++callback_counts;
            if (i != 42)
                throw std::domain_error("i != 42");
            return true;
        });
    // Check that 42 is passed through and -1 is discarded
    src(int_T(42));
    src(int_T(-1));
    ASSERT_EQ(1u, callback_counts);
}

TEST(fkie_message_filters, SimpleUserFilterCopyConstructible)
{
    simple_user_filter_test_code<int_C>();
}

TEST(fkie_message_filters, SimpleUserFilterMoveConstructible)
{
    simple_user_filter_test_code<int_M>();
}
