/****************************************************************************
 *
 * fkie_message_filters
 * Copyright © 2018 Fraunhofer FKIE
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
#include <fkie_message_filters/combiner_policies/fifo.h>
#include <fkie_message_filters/combiner.h>
#include <fkie_message_filters/simple_user_filter.h>
#include <fkie_message_filters/user_source.h>

TEST(fkie_message_filters, FifoCombiner)
{
    using Source1 = mf::UserSource<int_M, int_M>;
    using Source2 = mf::UserSource<double_M, double_M>;
    using Combiner = mf::Combiner<mf::combiner_policies::Fifo, Source1::Output, Source2::Output>;
    using Sink = mf::SimpleUserFilter<int_M, int_M, double_M, double_M>;

    std::size_t callback_counts = 0;
    Source1 src1;
    Source2 src2;
    Sink snk;
    Combiner combiner(Combiner::Policy(1));
    combiner.connect_to_sources(src1, src2);
    combiner.connect_to_sink(snk);
    snk.set_processing_function(
        [&](const int_M& i1, const int_M& i2, const double_M& d1, const double_M& d2) -> bool
        {
            ++callback_counts;
            if (i1 != 1) throw std::domain_error("i1 != 1");
            if (i2 != 2) throw std::domain_error("i2 != 2");
            if (d1 != 3.14) throw std::domain_error("d1 != 3.14");
            if (d2 != 6.28) throw ExpectedException("d2 != 6.28");
            return true;
        }
    );
    src1(int_M(1), int_M(2));
    src2(double_M(3.14), double_M(6.28));
    src2(double_M(3.14), double_M(0));
    ASSERT_THROW(src1(int_M(1), int_M(2)), ExpectedException);
    ASSERT_EQ(2u, callback_counts);
}

