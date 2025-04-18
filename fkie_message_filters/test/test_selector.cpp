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

#include <fkie_message_filters/selector.hpp>
#include <fkie_message_filters/simple_user_filter.hpp>
#include <fkie_message_filters/user_source.hpp>

template<typename int_T, typename double_T, typename string_T>
void selector_test_code()
{
    using Source = mf::UserSource<int_T, double_T, string_T>;
    using Select = mf::Selector<typename Source::Output, 1>;
    using Sink = mf::SimpleUserFilter<double_T>;

    std::size_t callback_counts = 0;
    Source src;
    Select select;
    Sink snk;
    snk.set_processing_function(
        [&](const double_T& d) -> bool
        {
            ++callback_counts;
            if (d == 2.5)
                throw ExpectedException("d == 2.5");
            if (d != 5.0)
                throw std::domain_error("d != 5.0");
            return true;
        });
    mf::chain(src, select, snk);
    src(int_T(1), double_T(5.0), string_T("abc"));
    ASSERT_THROW(src(int_T(0), double_T(2.5), string_T("")), ExpectedException);
    ASSERT_EQ(2u, callback_counts);
}

TEST(fkie_message_filters, SelectorCopyConstructible)
{
    selector_test_code<int_C, double_C, string_C>();
}

TEST(fkie_message_filters, SelectorMoveConstructible)
{
    selector_test_code<int_M, double_M, string_M>();
}
