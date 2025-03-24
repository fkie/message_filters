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
#include "test.h"

#include <fkie_message_filters/buffer.h>
#include <fkie_message_filters/simple_user_filter.h>
#include <fkie_message_filters/user_source.h>

template<typename int_T>
void buffer_test_code()
{
    using Source = mf::UserSource<int_T>;
    using Buffer = mf::Buffer<int_T>;
    using Sink = mf::SimpleUserFilter<int_T>;

    std::size_t callback_counts = 0;
    Source src;
    Buffer buf(mf::BufferPolicy::Queue, 3);
    Sink snk;
    snk.set_processing_function(
        [&](const int_T& i) -> bool
        {
            ++callback_counts;
            if (i != 0)
                throw std::domain_error("invalid value");
            return true;
        });
    mf::chain(src, buf, snk);
    ASSERT_FALSE(buf.has_some());
    // Purposely overflow the buffer queue to verify that the first item gets
    // discarded
    src(int_T(10000));
    src(int_T(0));
    src(int_T(0));
    src(int_T(0));
    ASSERT_TRUE(buf.has_some());
    ASSERT_EQ(0u, callback_counts);
    // Check manual processing functions
    buf.process_one();
    ASSERT_EQ(1u, callback_counts);
    buf.spin_once();
    ASSERT_FALSE(buf.has_some());
    ASSERT_EQ(3u, callback_counts);
    buf.spin_once();
    ASSERT_EQ(3u, callback_counts);
    src(int_T(0));
    buf.spin_once();
    ASSERT_EQ(4u, callback_counts);
}

TEST(fkie_message_filters, BufferCopyConstructible)
{
    buffer_test_code<int_C>();
}

TEST(fkie_message_filters, BufferMoveConstructible)
{
    buffer_test_code<int_M>();
}
