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

#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_FIFO_H_
#define INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_FIFO_H_

#include "policy_base.h"
#include <boost/circular_buffer.hpp>
#include <mutex>

namespace fkie_message_filters
{
namespace combiner_policies
{

/** \brief First-In-First-Out policy.
 *
 * This is a policy for the Combiner class. It will assemble data from the connected sources in a FIFO manner.
 * As soon as at least one element from each source has been received, a combined output tuple is created and passed to
 * the sink. There is no attempt to reorder inputs in any way.
 *
 * The FIFO policy is sufficient if all corresponding inputs arrive always in order and with the same frequency.
 * The policy has no requirements with regard to the data types it processes.
 */
template<typename... IOs>
class Fifo : public PolicyBase<IOs...>
{
public:
    template<template<typename...> class, class...> friend class Combiner;
    using typename PolicyBase<IOs...>::EmitterCB;
    using typename PolicyBase<IOs...>::IncomingTuples;
    using typename PolicyBase<IOs...>::OutgoingTuple;
    /** \brief Constructor.
     *
     * \arg \c max_queue_size maximum queue length for the FIFO queue, per input source
     *
     * \throw std::invalid_argument if \a max_queue_size is zero
     */
    explicit Fifo(std::size_t max_queue_size = 1);
protected:
    /** \brief Input function.
     *
     * This function will be called by the Combiner class for incoming data.
     */
    template<std::size_t N>
    void add(std::unique_lock<std::mutex>&, const std::tuple_element_t<N, IncomingTuples>&);
    void reset() noexcept override;
private:
    using typename PolicyBase<IOs...>::MaybeOutgoingTuples;
    using IncomingQueues = std::tuple<boost::circular_buffer<helpers::io_tuple_t<IOs>>...>;
    bool has_complete_tuple() noexcept;
    MaybeOutgoingTuples assemble_output() noexcept;
    IncomingQueues in_;
};

} // namespace combiner_policies
} // namespace fkie_message_filters

#include "fifo_impl.h"

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_FIFO_H_ */
