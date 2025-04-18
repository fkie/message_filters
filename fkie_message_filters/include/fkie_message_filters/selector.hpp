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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_SELECTOR_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_SELECTOR_HPP_
#pragma once

#include "filter.hpp"
#include "helpers/abi_namespace.hpp"

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

#ifndef DOXYGEN
template<class IO, std::size_t... Is>
class Selector;
#endif

/** \brief Reorder or reduce an N-ary filter.
 *
 * This filter reorders the arguments of an N-ary filter and/or chooses an arbitrary subset of the input arguments. The
 * output arguments are selected from the input by index. The following example reduces a ternary filter to a binary
 * filter of the first two arguments, in swapped order:
 *
 * \code
 * namespace mf = fkie_message_filters;
 *
 * using BufferIn = mf::Buffer<T0, T1, T2>;
 * using Select = mf::Selector<BufferIn::Output, 1, 0>;
 * using BufferOut = mf::Buffer<T1, T0>;
 *
 * BufferIn buf_in;
 * Select select;
 * BufferOut buf_out;
 * mf::chain(buf_in, select, buf_out);
 * \endcode
 * \sa Divider, Combiner
 */
#ifndef DOXYGEN
template<class... Inputs, std::size_t... Is>
class Selector<IO<Inputs...>, Is...> : public Filter<IO<Inputs...>, typename IO<Inputs...>::template Select<Is...>>
#else
template<class... Inputs, std::size_t... Is>
class Selector : public Filter<IO<Inputs...>, typename IO<Inputs...>::template Select<Is...>>
#endif
{
protected:
    void receive(helpers::argument_t<Inputs>... in) override;
};

FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#include "selector_impl.hpp"  // IWYU pragma: keep

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_SELECTOR_HPP_ */
