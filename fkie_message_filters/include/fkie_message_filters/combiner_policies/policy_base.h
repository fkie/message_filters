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

#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_POLICY_BASE_H_
#define INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_POLICY_BASE_H_

#include "../types.h"
#include <boost/optional.hpp>

namespace fkie_message_filters
{

template<template<typename...> class, class...> class Combiner;

namespace combiner_policies
{

/** \brief Base class for combiner policies. */
template<class... IOs>
class PolicyBase
{
public:
    template<template<typename...> class, class...> friend class fkie_message_filters::Combiner;
    /** \brief Tuple type of incoming data tuples. */
    using IncomingTuples = std::tuple<helpers::io_tuple_t<IOs>...>;
    /** \brief Combined tuple type for data output. */
    using OutgoingTuple = helpers::io_tuple_t<helpers::io_concat_t<IOs...>>;
    /** \brief Callback for assembled outputs. */
    using EmitterCB = std::function<void(const OutgoingTuple&)>;
    virtual ~PolicyBase() {}
protected:
    /** \brief Set output function.
     *
     * This function is called by the policy whenever it has output ready to be passed on.
     */
    void set_emitter_callback(const EmitterCB&) noexcept;
    /** \brief Reset internal state.
     *
     * This function is called by the Combiner if the filter is reset.
     */
    virtual void reset() noexcept = 0;
    /** \brief Tuple of outgoing tuple candidates.
     *
     * This is basically a tuple of optionals, so elements can remain empty until a suitable
     * data element has been found by the policy.
     */
    using MaybeOutgoingTuples = std::tuple<boost::optional<helpers::io_tuple_t<IOs>>...>;
    /** \brief Emit data.
     *
     * This returns combined data back to the Combiner class.
     */
    void emit (const OutgoingTuple& out);
private:
    EmitterCB emit_;
};

} // namespace combiner_policies
} // namespace fkie_message_filters

#include "policy_base_impl.h"

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_POLICY_BASE_H_ */
