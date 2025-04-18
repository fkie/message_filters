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

TEST(fkie_message_filters, FilterLoop)
{
    using Source = mf::UserSource<int_C>;
    using Filter = mf::SimpleUserFilter<int_C>;

    std::size_t callback_counts = 0;
    Source src;
    Filter flt;
    flt.set_processing_function(
        [&](const int_C& i) -> bool
        {
            ++callback_counts;
            return true;
        });
    mf::chain(src, flt);
    // Connect the filter output to its own input
    mf::chain(flt, flt);
    ASSERT_THROW(src(int_C(0)), std::logic_error);
    ASSERT_EQ(1u, callback_counts);
}
