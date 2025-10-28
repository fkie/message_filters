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

#include <fkie_message_filters/combiner.hpp>
#include <fkie_message_filters/combiner_policies/latch.hpp>
#include <fkie_message_filters/simple_user_filter.hpp>
#include <fkie_message_filters/user_source.hpp>

template<typename int_T>
void latch_test_code()
{
    using Source = mf::UserSource<int_T, int_T>;
    using Combiner = mf::Combiner<mf::combiner_policies::Latch, typename Source::Output, typename Source::Output>;
    using Sink = mf::SimpleUserFilter<int_T, int_T, int_T, int_T>;

    std::size_t callback_counts = 0;
    Source src1, src2;
    Sink snk;
    Combiner combiner1((typename Combiner::Policy()));
    combiner1.connect_to_sources(src1, src2);
    combiner1.connect_to_sink(snk);
    snk.set_processing_function(
        [&](const int_T& i1, const int_T& i2, const int_T& i3, const int_T& i4) -> bool
        {
            ++callback_counts;
            if (i1 != i2)
                throw std::domain_error("i1 != i2");
            if (i3 != i4)
                throw std::domain_error("i3 != i4");
            if (i1 == 3 && i3 == 2)
                throw ExpectedException("reused second source");
            if (i2 != i3)
                throw std::domain_error("i2 != i3");
            return true;
        });
    src1(int_T(1), int_T(1));
    src2(int_T(1), int_T(1));
    src2(int_T(1000), int_T(1000));
    src2(int_T(2), int_T(2));
    src1(int_T(2), int_T(2));
    ASSERT_THROW(src1(int_T(3), int_T(3)), ExpectedException);
    ASSERT_EQ(2u, callback_counts);
}

TEST(fkie_message_filters, LatchCombinerCopyConstructible)
{
    latch_test_code<int_C>();
}
