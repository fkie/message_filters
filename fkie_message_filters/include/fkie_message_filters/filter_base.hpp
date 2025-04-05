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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_FILTER_BASE_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_FILTER_BASE_HPP_
#pragma once

#include "helpers/abi_namespace.hpp"
#include "helpers/argument.hpp"
#include "types.hpp"

#include <functional>

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

namespace helpers
{
template<class... Types>
struct FilterCB
{
    using Type = std::function<void(helpers::argument_t<Types>...)>;
};

template<class... Types>
struct FilterCB<IO<Types...>>
{
    using Type = std::function<void(helpers::argument_t<Types>...)>;
};

}  // namespace helpers

/** \brief Callback function for customizable filters.
 * \sa UserFilter, SimpleUserFilter
 */
template<class... Types>
using FilterCB = typename helpers::FilterCB<Types...>::Type;

/** \brief Base class for filters.
 *
 * All filters process some input and generate some output, possibly with different data types.
 * This class provides the base class for all filter implementations.
 */
class FilterBase
{
public:
    FilterBase() = default;
    FilterBase(const FilterBase&) = delete;
    FilterBase& operator=(const FilterBase&) = delete;
    virtual ~FilterBase() {}
    /** \brief Disconnect from all connected sources and sinks.
     *
     * \nothrow
     */
    virtual void disconnect() noexcept = 0;
    /** \brief Reset filter state.
     *
     * For stateful filters, this method resets the internal state as if the filter had just been created.
     * Existing connections to sources and sinks are unaffected.
     *
     * The default implementation does nothing.
     *
     * \nothrow
     */
    virtual void reset() noexcept {};
};

FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_FILTER_BASE_HPP_ */
