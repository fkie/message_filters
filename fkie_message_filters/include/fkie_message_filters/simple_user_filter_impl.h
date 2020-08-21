/****************************************************************************
 *
 * fkie_message_filters
 * Copyright © 2018-2020 Fraunhofer FKIE
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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_SIMPLE_USER_FILTER_IMPL_H_
#define INCLUDE_FKIE_MESSAGE_FILTERS_SIMPLE_USER_FILTER_IMPL_H_

#include "simple_user_filter.h"

namespace fkie_message_filters
{

template<typename... Inputs>
void SimpleUserFilter<Inputs...>::set_processing_function(const ProcessingFunction& f) noexcept
{
    f_ = f;
}

template<typename... Inputs>
void SimpleUserFilter<Inputs...>::receive(const Inputs&... in)
{
    if (f_(in...)) this->send(in...);
}

} // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_SIMPLE_USER_FILTER_IMPL_H_ */
