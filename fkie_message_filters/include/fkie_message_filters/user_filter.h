/****************************************************************************
 *
 * fkie_message_filters
 * Copyright © 2018 Fraunhofer FKIE
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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_USER_FILTER_H_
#define INCLUDE_FKIE_MESSAGE_FILTERS_USER_FILTER_H_

#include "filter.h"
#include <functional>

namespace fkie_message_filters
{

#ifndef DOXYGEN
template<class In, class Out>
class UserFilter;
#endif

/** \brief Generic filter with user-defined callback function.
 *
 * You can implement your own filter logic and integrate it with the filter pipeline. For this, you need to define
 * your own function that takes the Inputs and a CallbackFunction as arguments. The function will be called by
 * the generic filter, and expects you to feed back the processed data using the callback function.
 *
 * The filter will throw a std::bad_function_call exception if it is invoked without a user-defined processing function.
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
    using ProcessingFunction = std::function<void(const Inputs&..., const CallbackFunction&)>;
    /** \brief Set the user-defined processing function.
     *
     * You must call this method before the filter gets invoked with incoming data.
     *
     * \nothrow
     */
    void set_processing_function (const ProcessingFunction& f) noexcept;
protected:
    void receive (const Inputs&... in) override;
private:
    ProcessingFunction f_;
};

} // namespace fkie_message_filters

#include "user_filter_impl.h"

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_USER_FILTER_H_ */
