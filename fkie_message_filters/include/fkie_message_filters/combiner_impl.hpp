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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_IMPL_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_IMPL_HPP_
#pragma once

// IWYU pragma: private; include "combiner.hpp"

#include "combiner.hpp"

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

template<template<typename...> class PolicyTmpl, class... IOs>
Combiner<PolicyTmpl, IOs...>::Combiner(const Policy& policy) noexcept : policy_(policy)
{
    helpers::for_each_apply<std::tuple_size<IncomingTuples>::value>([this](auto I)
                                                                    { std::get<I>(this->sinks_).set_parent(this); });
    connect_policy();
}

template<template<typename...> class PolicyTmpl, class... IOs>
void Combiner<PolicyTmpl, IOs...>::connect_policy() noexcept
{
    policy_.set_emitter_callback(
        [this](OutgoingTuple& out)
        {
            helpers::index_apply<std::tuple_size<OutgoingTuple>::value>(
                [this, &out](auto... Is) { this->send(helpers::maybe_move(std::get<Is>(out))...); });
        });
    helpers::for_each_apply<std::tuple_size<IncomingTuples>::value>(
        [this](auto I)
        {
            std::get<I>(this->sinks_)
                .set_policy_input(
                    [this](std::unique_lock<std::mutex>& lock, auto&& in)
                    { this->policy_.template add<decltype(I)::value>(lock, std::forward<decltype(in)>(in)); });
        });
}

template<template<typename...> class PolicyTmpl, class... IOs>
void Combiner<PolicyTmpl, IOs...>::set_policy(const Policy& policy) noexcept
{
    std::lock_guard<std::mutex> lock(combiner_mutex_);
    policy_ = policy;
    connect_policy();
}

template<template<typename...> class PolicyTmpl, class... IOs>
void Combiner<PolicyTmpl, IOs...>::reset() noexcept
{
    std::lock_guard<std::mutex> lock(combiner_mutex_);
    policy_.reset();
}

template<template<typename...> class PolicyTmpl, class... IOs>
template<std::size_t N>
typename Combiner<PolicyTmpl, IOs...>::template SinkType<N>& Combiner<PolicyTmpl, IOs...>::sink() noexcept
{
    return std::get<N>(sinks_);
}

template<template<typename...> class PolicyTmpl, class... IOs>
template<std::size_t N>
const typename Combiner<PolicyTmpl, IOs...>::template SinkType<N>& Combiner<PolicyTmpl, IOs...>::sink() const noexcept
{
    return std::get<N>(sinks_);
}

template<template<typename...> class PolicyTmpl, class... IOs>
const typename Combiner<PolicyTmpl, IOs...>::Policy& Combiner<PolicyTmpl, IOs...>::policy() const noexcept
{
    return policy_;
}

template<template<typename...> class PolicyTmpl, class... IOs>
void Combiner<PolicyTmpl, IOs...>::disconnect_from_all_sources() noexcept
{
    helpers::for_each_apply<sizeof...(IOs)>([this](auto I)
                                            { std::get<I>(this->sinks_).disconnect_from_all_sources(); });
}

template<template<typename...> class PolicyTmpl, class... IOs>
void Combiner<PolicyTmpl, IOs...>::disconnect() noexcept
{
    disconnect_from_all_sources();
    this->disconnect_from_all_sinks();
}

template<template<typename...> class PolicyTmpl, class... IOs>
template<std::size_t N, typename ThisSource, typename... OtherSources>
void Combiner<PolicyTmpl, IOs...>::connect_to_sources_impl(Connections& conn, ThisSource& src,
                                                           OtherSources&... sources) noexcept
{
    conn[N] = std::get<N>(sinks_).connect_to_source(src);
    if constexpr (sizeof...(OtherSources))
        connect_to_sources_impl<N + 1>(conn, sources...);
}

template<template<typename...> class PolicyTmpl, class... IOs>
typename Combiner<PolicyTmpl, IOs...>::Connections
Combiner<PolicyTmpl, IOs...>::connect_to_sources(helpers::io_rewrap_t<IOs, Source>&... sources) noexcept
{
    Connections conn;
    if constexpr (sizeof...(IOs))
        connect_to_sources_impl<0>(conn, sources...);
    return conn;
}

template<template<typename...> class PolicyTmpl, class... IOs>
template<class... Inputs>
void Combiner<PolicyTmpl, IOs...>::CombinerSink<Inputs...>::set_parent(Combiner<PolicyTmpl, IOs...>* parent) noexcept
{
    parent_ = parent;
}

template<template<typename...> class PolicyTmpl, class... IOs>
template<class... Inputs>
void Combiner<PolicyTmpl, IOs...>::CombinerSink<Inputs...>::set_policy_input(const PolicyInFunc& f) noexcept
{
    forward_ = f;
}

template<template<typename...> class PolicyTmpl, class... IOs>
template<class... Inputs>
void Combiner<PolicyTmpl, IOs...>::CombinerSink<Inputs...>::receive(helpers::argument_t<Inputs>... in)
{
    assert(parent_);
    std::unique_lock<std::mutex> lock(parent_->combiner_mutex_);
    if (forward_)
    {
        forward_(lock, std::forward_as_tuple(helpers::maybe_move(in)...));
    }
}

FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_IMPL_HPP_ */
