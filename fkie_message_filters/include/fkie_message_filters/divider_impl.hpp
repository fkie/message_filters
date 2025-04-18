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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_DIVIDER_IMPL_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_DIVIDER_IMPL_HPP_
#pragma once

// IWYU pragma: private; include "divider.hpp"

#include "divider.hpp"
#include "helpers/tuple.hpp"

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

template<class... Inputs>
template<std::size_t N>
typename Divider<Inputs...>::template SourceType<N>& Divider<Inputs...>::source() noexcept
{
    return std::get<N>(sources_);
}

template<class... Inputs>
template<std::size_t N>
const typename Divider<Inputs...>::template SourceType<N>& Divider<Inputs...>::source() const noexcept
{
    return std::get<N>(sources_);
}

template<class... Inputs>
typename Divider<Inputs...>::Connections Divider<Inputs...>::connect_to_sinks(Sink<Inputs>&... sinks) noexcept
{
    Connections conn;
    if constexpr (sizeof...(Inputs))
        connect_to_sinks_impl<0>(conn, sinks...);
    return conn;
}

template<class... Inputs>
void Divider<Inputs...>::disconnect_from_all_sinks() noexcept
{
    helpers::for_each_apply<sizeof...(Inputs)>([this](auto I)
                                               { std::get<I>(this->sources_).disconnect_from_all_sinks(); });
}

template<class... Inputs>
void Divider<Inputs...>::disconnect() noexcept
{
    this->disconnect_from_all_sources();
    disconnect_from_all_sinks();
}

template<class... Inputs>
template<class Input>
template<class ForwardedInput>
void Divider<Inputs...>::DividerSource<Input>::forward(ForwardedInput&& in)
{
    this->send(helpers::maybe_move(in));
}

template<class... Inputs>
void Divider<Inputs...>::receive(helpers::argument_t<Inputs>... ins)
{
    if constexpr (sizeof...(Inputs))
        forward_to_sources<0>(helpers::maybe_move(ins)...);
}

template<class... Inputs>
template<std::size_t N, typename ThisInput, typename... OtherInputs>
void Divider<Inputs...>::forward_to_sources(ThisInput&& in, OtherInputs&&... ins)
{
    std::get<N>(sources_).forward(std::forward<ThisInput&&>(in));
    if constexpr (sizeof...(OtherInputs))
        forward_to_sources<N + 1>(std::forward<OtherInputs&&>(ins)...);
}

template<class... Inputs>
template<std::size_t N, typename ThisSink, typename... OtherSinks>
void Divider<Inputs...>::connect_to_sinks_impl(Connections& conn, ThisSink& sink, OtherSinks&... sinks) noexcept
{
    conn[N] = std::get<N>(sources_).connect_to_sink(sink);
    if constexpr (sizeof...(OtherSinks))
        connect_to_sinks_impl<N + 1>(conn, sinks...);
}

FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_DIVIDER_IMPL_HPP_ */
