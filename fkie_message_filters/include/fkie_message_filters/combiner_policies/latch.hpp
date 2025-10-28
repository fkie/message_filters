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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_LATCH_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_LATCH_HPP_
#pragma once

#include "../helpers/abi_namespace.hpp"
#include "policy_base.hpp"

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

namespace combiner_policies
{

/** \brief Latch policy.
 *
 * This is a policy for the Combiner class. It works similar to the Fifo policy, but
 * it retains the latest data from each connected source and will reuse data
 * if some inputs arrive at a slower rate than others. The policy can be configured to
 * generate output if a particular source is updated or any source receives new data.
 *
 * As the policy must be able to create copies of incoming data, it is not
 * suitable for move-only input types.
 */
template<typename... IOs>
class Latch : public PolicyBase<IOs...>
{
public:
    template<template<typename...> class, class...>
    friend class fkie_message_filters::Combiner;
    using typename PolicyBase<IOs...>::EmitterCB;
    using typename PolicyBase<IOs...>::IncomingTuples;
    using typename PolicyBase<IOs...>::OutgoingTuple;

    /** \brief Constructor.
     *
     * By default, the policy will generate output whenever the first connected source receives data.
     */
    Latch() noexcept;
    /** \brief Generate output on any update.
     *
     * Instruct the policy to generate output whenever any new data arrives.
     */
    Latch& trigger_on_any() noexcept;
    /** \brief Generate output on any update.
     *
     * Instruct the policy to generate output whenever new data arrives from a particular source.
     */
    Latch& trigger_on(std::size_t source) noexcept;

protected:
    /** \brief Input function.
     *
     * This function will be called by the Combiner class for incoming data.
     */
    template<std::size_t N>
    void add(std::unique_lock<std::mutex>&, std::tuple_element_t<N, IncomingTuples>&&);
    void reset() noexcept override;

private:
    static constexpr std::size_t NUM_SLOTS = sizeof...(IOs);
    using typename PolicyBase<IOs...>::MaybeOutgoingTuples;
    bool has_complete_tuple() noexcept;
    std::optional<std::size_t> trigger_;
    MaybeOutgoingTuples latched_;
};

}  // namespace combiner_policies
FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#include "latch_impl.hpp"  // IWYU pragma: keep

#endif
