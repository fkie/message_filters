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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_HELPERS_IO_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_HELPERS_IO_HPP_
#pragma once

#include "abi_namespace.hpp"

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

template<typename...>
class IO;

namespace helpers
{

template<typename T>
struct io_wrap
{
    using type = IO<T>;
};

template<typename... Ts>
struct io_wrap<IO<Ts...>>
{
    using type = IO<Ts...>;
};

template<typename... Ts>
struct io_wrap<IO<IO<Ts...>>>
{
    using type = typename io_wrap<IO<Ts...>>::type;
};

template<typename T>
using io_wrap_t = typename io_wrap<T>::type;

template<typename T>
struct io_unwrap
{
    using type = T;
};

template<typename T>
struct io_unwrap<IO<T>>
{
    using type = typename io_unwrap<T>::type;
};

template<typename T>
using io_unwrap_t = typename io_unwrap<T>::type;

template<typename IO, template<typename...> class Wrap>
struct io_rewrap
{
    using type = typename io_wrap_t<IO>::template Rewrap<Wrap>;
};

template<typename IO, template<typename...> class Wrap>
using io_rewrap_t = typename io_rewrap<IO, Wrap>::type;

template<typename T, typename U>
struct io_concat_impl;

template<typename... Ts, typename... Us>
struct io_concat_impl<IO<Ts...>, IO<Us...>>
{
    using type = IO<Ts..., Us...>;
};

template<typename... Ts>
struct io_concat;

template<typename T1, typename T2, typename... Ts>
struct io_concat<T1, T2, Ts...>
{
    using type = typename io_concat<typename io_concat_impl<io_wrap_t<T1>, io_wrap_t<T2>>::type, Ts...>::type;
};

template<typename T>
struct io_concat<T>
{
    using type = io_wrap_t<T>;
};

template<typename... Ts>
using io_concat_t = typename io_concat<Ts...>::type;

template<typename IO>
struct io_tuple
{
    using type = typename io_wrap_t<IO>::Tuple;
};

template<typename IO>
using io_tuple_t = typename io_tuple<IO>::type;

}  // namespace helpers
FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_HELPERS_IO_HPP_ */
