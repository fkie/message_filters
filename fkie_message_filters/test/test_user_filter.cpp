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

#include <fkie_message_filters/simple_user_filter.hpp>
#include <fkie_message_filters/user_filter.hpp>
#include <fkie_message_filters/user_source.hpp>

#include <stdexcept>

template<typename int_T, typename double_T>
void user_filter_test_code()
{
    using Source = mf::UserSource<int_T>;
    using Sink = mf::SimpleUserFilter<double_T>;
    using Filter = mf::UserFilter<typename Source::Output, typename Sink::Input>;

    std::size_t callback_counts = 0;
    Source src;
    Filter flt;
    Sink snk;
    mf::chain(src, flt, snk);
    flt.set_processing_function(
        [](const int_T& i, const typename Filter::CallbackFunction& f)
        {
            if (i == 42)
                f(double_T(3.14));
            if (i == 99)
                throw std::invalid_argument("99 is evil!");
        });
    snk.set_processing_function(
        [&](const double_T& d) -> bool
        {
            ++callback_counts;
            if (d != 3.14)
                throw std::domain_error("d != 3.14");
            return true;
        });
    // Check that 42 is passed through and -1 is discarded
    src(int_T(42));
    src(int_T(-1));
    ASSERT_EQ(1u, callback_counts);
    flt.disconnect();
    src(int_T(99));
}

TEST(fkie_message_filters, UserFilterCopyConstructible)
{
    user_filter_test_code<int_C, double_C>();
}

TEST(fkie_message_filters, UserFilterMoveConstructible)
{
    user_filter_test_code<int_M, double_M>();
}
