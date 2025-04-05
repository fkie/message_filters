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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_FILTER_IMPL_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_FILTER_IMPL_HPP_
#pragma once

// IWYU pragma: private; include "filter.hpp"

#include "filter.hpp"

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

template<class In, class Out>
void Filter<In, Out>::disconnect() noexcept
{
    this->disconnect_from_all_sources();
    this->disconnect_from_all_sinks();
}

namespace helpers
{

template<typename Filter1, typename Filter2, typename... MoreFilters>
void chain_impl(Filter1& flt1, Filter2& flt2, MoreFilters&... filters) noexcept
{
    if constexpr (sizeof...(MoreFilters))
        chain_impl(flt2, filters...);
    flt1.connect_to_sink(flt2);
}

}  // namespace helpers

template<typename Filter1, typename Filter2, typename... MoreFilters>
void chain(Filter1& flt1, Filter2& flt2, MoreFilters&... filters) noexcept
{
    helpers::chain_impl(flt1, flt2, filters...);
}

FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_FILTER_IMPL_HPP_ */
