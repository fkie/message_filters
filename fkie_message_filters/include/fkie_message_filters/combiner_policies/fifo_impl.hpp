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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_FIFO_IMPL_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_FIFO_IMPL_HPP_
#pragma once

// IWYU pragma: private; include "fifo.hpp"

#include "../helpers/scoped_unlock.hpp"
#include "../helpers/tuple.hpp"
#include "fifo.hpp"

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

namespace combiner_policies
{

template<typename... IOs>
Fifo<IOs...>::Fifo(std::size_t max_queue_size) : max_queue_size_(max_queue_size)
{
}

template<typename... IOs>
Fifo<IOs...>::Fifo(const Fifo& other) : PolicyBase<IOs...>(other), max_queue_size_(other.max_queue_size_)
{
    /* The copy constructor deliberately avoids copying the incoming queue, because
     * a) the connection to any Combiner instance is broken by the copying anyway and
     * b) it avoids issues with move-only types (std::deque<T> is copyable iff T is copyable)
     */
}

template<typename... IOs>
template<std::size_t N>
void Fifo<IOs...>::add(std::unique_lock<std::mutex>& lock, std::tuple_element_t<N, IncomingTuples>&& item)
{
    auto& queue = std::get<N>(queues_);
    queue.push_back(std::move(item));
    if (queue.size() > max_queue_size_)
        queue.pop_front();
    while (has_complete_tuple())
    {
        MaybeOutgoingTuples tmp = assemble_output();
        helpers::index_apply<sizeof...(IOs)>(
            [this, &tmp, &lock](auto... Is)
            {
                auto unlock = helpers::with_scoped_unlock(lock);
                OutgoingTuple e{std::tuple_cat(std::move(*std::get<Is>(tmp))...)};
                this->emit(e);
            });
    }
}

template<typename... IOs>
bool Fifo<IOs...>::has_complete_tuple() noexcept
{
    return helpers::all_true<sizeof...(IOs)>([this](auto I) { return !std::get<I>(this->queues_).empty(); });
}

template<typename... IOs>
typename Fifo<IOs...>::MaybeOutgoingTuples Fifo<IOs...>::assemble_output() noexcept
{
    MaybeOutgoingTuples tmp;
    helpers::for_each_apply<sizeof...(IOs)>(
        [this, &tmp](auto I)
        {
            auto& queue = std::get<I>(this->queues_);
            std::get<I>(tmp) = std::move(queue.front());
            queue.pop_front();
        });
    return tmp;
}

template<typename... IOs>
void Fifo<IOs...>::reset() noexcept
{
    helpers::for_each_apply<sizeof...(IOs)>([this](auto I) { std::get<I>(this->queues_).clear(); });
}

}  // namespace combiner_policies
FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_FIFO_IMPL_HPP_ */
