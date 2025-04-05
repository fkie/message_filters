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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_USER_FILTER_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_USER_FILTER_HPP_
#pragma once

#include "filter.hpp"
#include "helpers/abi_namespace.hpp"

#include <functional>

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

#ifndef DOXYGEN
template<class In, class Out>
class UserFilter;
#endif

/** \brief Generic filter with user-defined callback function.
 *
 * You can implement your own filter logic and integrate it with the filter pipeline. For this, you need to define your
 * own function that takes the Inputs and a CallbackFunction as arguments. The function will be called by the generic
 * filter, and expects you to feed back the processed data using the callback function. Typically, your code will look
 * similar to this:
 *
 * \code
 * namespace mf = fkie_message_filters;
 * using InBuffer = mf::Buffer<T1, T2>;
 * using OutBuffer = mf::Buffer<T3, T4>;
 * using MyFilter = mf::UserFilter<InBuffer::Output, OutBuffer::Input>;
 *
 * void my_processing_func(const T1& t1, const T2& t2, const MyFilter::CallbackFunction& cb)
 * {
 *     // Process the inputs t1 and t2 and prepare the outputs t3 and t4
 *     T3 t3;
 *     T4 t4;
 *     // ...
 *
 *     // Feed the outputs back into the filter pipeline
 *     cb(t3, t4);
 * }
 * \endcode
 *
 * Set up your filter pipeline like this:
 * \code
 * InBuffer in;
 * MyFilter flt;
 * OutBuffer out;
 *
 * flt.set_processing_function(my_processing_func);
 * mf::chain(in, flt, out);
 * \endcode
 *
 * In your processing function, you can call the callback function as often as you want, or even not at all. There is no
 * requirement that each input produces exactly one output. When you have no output at all, or the output is the same as
 * the input, you should consider the SimpleUserFilter instead, which is easier to set up.
 *
 * The filter will throw a \c std::bad_function_call exception if it is invoked without a user-defined processing
 * function.
 *
 */
#ifndef DOXYGEN
template<class... Inputs, class... Outputs>
class UserFilter<IO<Inputs...>, IO<Outputs...>> : public Filter<IO<Inputs...>, IO<Outputs...>>
#else
template<class... Inputs, class... Outputs>
class UserFilter : public Filter<IO<Inputs...>, IO<Outputs...>>
#endif
{
public:
    /** \brief Callback function type.
     *
     * This is the signature of the callback function pointer your processing function will receive.
     */
    using CallbackFunction = FilterCB<Outputs...>;
    /** \brief Processing function type.
     *
     * This can be any user-defined function and will be called to process incoming data.
     * The results are expected be returned via the callback function.
     */
    using ProcessingFunction = std::function<void(helpers::argument_t<Inputs>..., const CallbackFunction&)>;
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

FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#include "user_filter_impl.hpp"  // IWYU pragma: keep

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_USER_FILTER_HPP_ */
