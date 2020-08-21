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

#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_USER_SOURCE_H_
#define INCLUDE_FKIE_MESSAGE_FILTERS_USER_SOURCE_H_

#include "source.h"

namespace fkie_message_filters
{

/** \brief Manually operated data source.
 *
 * This data source behaves like a function object and can be called to pass data to its sinks. It is mostly useful
 * to connect a filter pipeline to foreign data sources. Even though its technically possible to use a UserSource
 * object as callback for a ROS subscriber, the specialized Subscriber sources have additional functionality and are
 * easier to set up.
 */
template<class... Outputs>
class UserSource : public Source<Outputs...>
{
public:
    /** \brief Manually inject data and pass it to the connected sinks.
     * \arg \c out data
     *
     * \filterthrow
     */
    void operator()(const Outputs&... out);
};

template<class... Outputs>
class UserSource<IO<Outputs...>> : public UserSource<Outputs...> {};

} // namespace fkie_message_filters

#include "user_source_impl.h"

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_USER_SOURCE_H_ */
