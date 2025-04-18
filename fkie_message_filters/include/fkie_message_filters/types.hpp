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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_TYPES_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_TYPES_HPP_
#pragma once

#include "helpers/abi_namespace.hpp"
#include "helpers/io.hpp"
#include "helpers/signaling.hpp"

#include <tuple>

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

/** \brief Tracks connections from sources to sinks. */
using Connection = helpers::Connection;

/** \brief Group multiple data types as filter input or output.
 *
 * This is a helper class to disambiguate filter definitions with N-ary inputs and outputs. Consider the following
 * hypothetical example:
 * \code
 * Filter<Type1, Type2, Type3> flt;
 * \endcode
 * It is unclear which types refer to inputs and which refer to outputs. In contrast, consider:
 * \code
 * Filter<IO<Type1>, IO<Type2, Type3>> flt1;
 * Filter<IO<Type1, Type2>, IO<Type3>> flt2;
 * \endcode
 * This declaration makes it clear that the first filter accepts \c Type1 as input and will pass \c Type2 and \c Type3
 * as output, while the second filter will take \c Type1 and \c Type2 as input and pass \c Type3 as output.
 */
template<typename... Types>
class IO
{
public:
    /** \brief Tuple of the grouped data types. */
    using Tuple = std::tuple<helpers::io_unwrap_t<Types>...>;
    /** \brief Nth data type of an IO tuple. */
    template<std::size_t N>
    using Type = typename std::tuple_element<N, std::tuple<helpers::io_unwrap_t<Types>...>>::type;
    /** \brief Subset of the grouped data types. */
    template<std::size_t... Is>
    using Select = IO<typename std::tuple_element<Is, std::tuple<helpers::io_unwrap_t<Types>...>>::type...>;
    /** \brief Rewrap the grouped data types in a different wrapper template type. */
    template<template<typename...> class Outer>
    using Rewrap = Outer<helpers::io_unwrap_t<Types>...>;
};

FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_TYPES_HPP_ */
