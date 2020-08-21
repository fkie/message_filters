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
#include <fkie_message_filters/user_source.h>
#include <fkie_message_filters/simple_user_filter.h>

TEST(fkie_message_filters, FilterLoop)
{
    using Source = mf::UserSource<int_M>;
    using Filter = mf::SimpleUserFilter<int_M>;

    std::size_t callback_counts = 0;
    Source src;
    Filter flt;
    flt.set_processing_function(
        [&](const int_M& i) -> bool
        {
            ++callback_counts;
            return true;
        }
    );
    mf::chain(src, flt);
    // Connect the filter output to its own input
    mf::chain(flt, flt);
    ASSERT_THROW(src(int_M(0)), std::logic_error);
}
