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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_SIMPLE_USER_FILTER_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_SIMPLE_USER_FILTER_HPP_
#pragma once

#include "filter.hpp"
#include "helpers/abi_namespace.hpp"

#include <functional>

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

/** \brief Simplified filter with user-defined callback function.
 *
 * This is a simplified version of the UserFilter where inputs and outputs are identical. A user-defined
 * processing function is called for all incoming data. If the function returns \c true, the data is passed on,
 * otherwise the data is discarded.
 *
 * The filter will throw a \c std::bad_function_call exception if it is invoked without a user-defined processing
 * function.
 */
template<typename... Inputs>
class SimpleUserFilter : public Filter<IO<Inputs...>, IO<Inputs...>>
{
public:
    /** \brief Processing function type.
     *
     * This can be any user-defined function and will be called to process incoming data.
     * \retval true if the data shall be passed to the connected sinks
     * \retval false if the data shall be discarded
     *
     * \nothrow
     */
    using ProcessingFunction = std::function<bool(const Inputs&...)>;
    /** \brief Set the user-defined processing function.
     *
     * You must call this method before the filter gets invoked with incoming data.
     *
     * \nothrow
     */
    void set_processing_function(const ProcessingFunction& f) noexcept;

protected:
    void receive(helpers::argument_t<Inputs>... in) override;

private:
    ProcessingFunction f_;
};

#ifndef DOXYGEN
template<typename... Inputs>
class SimpleUserFilter<IO<Inputs...>> : public SimpleUserFilter<Inputs...>
{
};
#endif

FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#include "simple_user_filter_impl.hpp"  // IWYU pragma: keep

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_SIMPLE_USER_FILTER_HPP_ */
