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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_SOURCE_IMPL_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_SOURCE_IMPL_HPP_
#pragma once

// IWYU pragma: private; include "source.hpp"

#include "source.hpp"

#include <mutex>

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

template<typename... Outputs>
Connection Source<Outputs...>::connect_to_sink(Sink<Outputs...>& dst) noexcept
{
    std::lock_guard<std::mutex> lock(dst.mutex_);
    Connection c = signal_.connect([&dst](Outputs&&... out) { dst.receive_cb(std::forward<Outputs&&>(out)...); });
    dst.conn_.push_back(c);
    return c;
}

template<typename... Outputs>
void Source<Outputs...>::disconnect_from_all_sinks() noexcept
{
    signal_.disconnect_all_slots();
}

template<typename... Outputs>
void Source<Outputs...>::disconnect() noexcept
{
    disconnect_from_all_sinks();
}

template<typename... Outputs>
void Source<Outputs...>::send(helpers::argument_t<Outputs>... out)
{
    signal_(helpers::maybe_move(out)...);
}

FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_SOURCE_IMPL_HPP_ */
