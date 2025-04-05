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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_FILTER_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_FILTER_HPP_
#pragma once

#include "helpers/abi_namespace.hpp"
#include "sink.hpp"
#include "source.hpp"

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

/** \brief Typed base class for filters.
 *
 * All filters process some input and generate some output, possibly with different data types. This class provides the
 * base class for all filter implementations, templated on the input and output data types.
 */
template<class In, class Out>
class Filter : public Sink<In>, public Source<Out>
{
public:
    /** \brief Disconnect from all connected sources and sinks.
     *
     * Convenience function that calls disconnect_from_all_sources() and disonnect_from_all_sinks().
     *
     * \nothrow
     */
    virtual void disconnect() noexcept override;
};

/** \brief Convenience function to chain multiple filters.
 *
 * Basically calls Source::connect_to_sink() on adjacent filters to form a chain.
 */
template<typename Filter1, typename Filter2, typename... MoreFilters>
void chain(Filter1& flt1, Filter2& flt2, MoreFilters&... filters) noexcept;

FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#include "filter_impl.hpp"  // IWYU pragma: keep

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_FILTER_HPP_ */
