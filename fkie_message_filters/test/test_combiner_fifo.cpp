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

#include <fkie_message_filters/combiner.hpp>
#include <fkie_message_filters/combiner_policies/fifo.hpp>
#include <fkie_message_filters/simple_user_filter.hpp>
#include <fkie_message_filters/user_source.hpp>

template<typename int_T, typename double_T>
void fifo_test_code()
{
    using Source1 = mf::UserSource<int_T, int_T>;
    using Source2 = mf::UserSource<double_T, double_T>;
    using Combiner = mf::Combiner<mf::combiner_policies::Fifo, typename Source1::Output, typename Source2::Output>;
    using Sink = mf::SimpleUserFilter<int_T, int_T, double_T, double_T>;

    std::size_t callback_counts = 0;
    Source1 src1;
    Source2 src2;
    Sink snk;
    Combiner combiner(typename Combiner::Policy(1));
    combiner.connect_to_sources(src1, src2);
    combiner.connect_to_sink(snk);
    snk.set_processing_function(
        [&](const int_T& i1, const int_T& i2, const double_T& d1, const double_T& d2) -> bool
        {
            ++callback_counts;
            if (i1 != 1)
                throw std::domain_error("i1 != 1");
            if (i2 != 2)
                throw std::domain_error("i2 != 2");
            if (d1 != 3.14)
                throw std::domain_error("d1 != 3.14");
            if (d2 != 6.28)
                throw ExpectedException("d2 != 6.28");
            return true;
        });
    src1(int_T(1), int_T(2));
    src2(double_T(3.14), double_T(6.28));
    src2(double_T(3.14), double_T(0));
    ASSERT_THROW(src1(int_T(1), int_T(2)), ExpectedException);
    ASSERT_EQ(2u, callback_counts);
}

TEST(fkie_message_filters, FifoCombinerCopyConstructible)
{
    fifo_test_code<int_C, double_C>();
}

TEST(fkie_message_filters, FifoCombinerMoveConstructible)
{
    fifo_test_code<int_M, double_M>();
}
