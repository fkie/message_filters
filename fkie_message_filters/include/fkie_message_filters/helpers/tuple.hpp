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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_HELPERS_TUPLE_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_HELPERS_TUPLE_HPP_
#pragma once

#include "abi_namespace.hpp"

#include <initializer_list>
#include <tuple>

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

namespace helpers
{

template<std::size_t N, typename... Ts>
using select_nth = std::tuple_element_t<N, std::tuple<Ts...>>;

template<class Function, std::size_t... Is>
auto index_apply_impl(Function f, std::index_sequence<Is...>)
{
    return f(std::integral_constant<std::size_t, Is>{}...);
}

template<std::size_t N, class Function>
auto index_apply(Function f)
{
    return index_apply_impl(f, std::make_index_sequence<N>{});
}

template<class Function, std::size_t... Is>
void for_each_apply_impl(Function f, std::index_sequence<Is...>)
{
    (void)std::initializer_list<int>{(f(std::integral_constant<std::size_t, Is>{}), void(), 0)...};
}

template<std::size_t N, class Function>
void for_each_apply(Function f)
{
    for_each_apply_impl(f, std::make_index_sequence<N>{});
}

template<std::size_t N, class Function>
void select_apply(std::size_t i, Function f)
{
    for_each_apply<N>(
        [&](auto&& Is)
        {
            if (Is == i)
                f(std::forward<decltype(Is)>(Is));
        });
}

template<class Predicate, std::size_t... Is>
bool all_true_impl(Predicate p, std::index_sequence<Is...>)
{
    return (p(std::integral_constant<std::size_t, Is>{}) && ...);
}

template<std::size_t N, class Predicate>
bool all_true(Predicate p)
{
    return all_true_impl(p, std::make_index_sequence<N>{});
}

}  // namespace helpers
FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_HELPERS_TUPLE_HPP_ */
