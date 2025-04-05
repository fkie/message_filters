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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_HELPERS_ARGUMENT_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_HELPERS_ARGUMENT_HPP_
#pragma once

#include "abi_namespace.hpp"

#include <type_traits>
#include <utility>

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

namespace helpers
{

template<class T, bool, bool>
struct argument;

template<class T>
struct argument<T, true, true>
{
    using type = const T&;
};

template<class T>
struct argument<T, false, true>
{
    using type = T;
};

template<typename T>
using argument_t = typename argument<T, std::is_copy_constructible_v<T>, std::is_move_constructible_v<T>>::type;

template<typename T, std::enable_if_t<std::is_copy_constructible_v<T>, bool> = true>
constexpr T& maybe_move(T& arg) noexcept
{
    return arg;
}

template<typename T, std::enable_if_t<!std::is_copy_constructible_v<T>, bool> = true>
constexpr T&& maybe_move(T& arg) noexcept
{
    return std::move(arg);
}

}  // namespace helpers
FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#endif
