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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_LATCH_IMPL_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_LATCH_IMPL_HPP_
#pragma once

// IWYU pragma: private; include "latch.hpp"

#include "../helpers/scoped_unlock.hpp"
#include "../helpers/tuple.hpp"
#include "latch.hpp"

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

namespace combiner_policies
{

template<typename... IOs>
Latch<IOs...>::Latch() noexcept : trigger_(0)
{
}

template<typename... IOs>
Latch<IOs...>& Latch<IOs...>::trigger_on_any() noexcept
{
    trigger_.reset();
    return *this;
}

template<typename... IOs>
Latch<IOs...>& Latch<IOs...>::trigger_on(std::size_t source) noexcept
{
    trigger_ = source;
    return *this;
}

template<typename... IOs>
void Latch<IOs...>::reset() noexcept
{
    helpers::for_each_apply<sizeof...(IOs)>([this](auto I) { std::get<I>(this->latched_).reset(); });
}

template<typename... IOs>
bool Latch<IOs...>::has_complete_tuple() noexcept
{
    return helpers::all_true<sizeof...(IOs)>([this](auto I) { return !!std::get<I>(this->latched_); });
}

template<typename... IOs>
template<std::size_t N>
void Latch<IOs...>::add(std::unique_lock<std::mutex>& lock, std::tuple_element_t<N, IncomingTuples>&& item)
{
    std::get<N>(latched_) = std::move(item);
    if (trigger_ && *trigger_ != N)
        return;
    if (!has_complete_tuple())
        return;
    helpers::index_apply<sizeof...(IOs)>(
        [this, &lock](auto... Is)
        {
            OutgoingTuple e{std::tuple_cat(*std::get<Is>(this->latched_)...)};
            auto unlock = helpers::with_scoped_unlock(lock);
            this->emit(e);
        });
}

}  // namespace combiner_policies
FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#endif
