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

#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_FIFO_IMPL_H_
#define INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_FIFO_IMPL_H_

#include "fifo.h"
#include "../helpers/tuple.h"
#include "../helpers/scoped_unlock.h"

namespace fkie_message_filters
{
namespace combiner_policies
{

template<typename... IOs>
Fifo<IOs...>::Fifo(std::size_t max_queue_size)
{
    if (max_queue_size == 0) throw std::invalid_argument("max_queue_size must not be zero");
    helpers::for_each_apply<sizeof...(IOs)>(
        [this, max_queue_size](auto I)
        {
            std::get<I>(this->in_).set_capacity(max_queue_size);
        }
    );
}

template<typename... IOs>
template<std::size_t N>
void Fifo<IOs...>::add(std::unique_lock<std::mutex>& lock, const std::tuple_element_t<N, IncomingTuples>& item)
{
    std::get<N>(in_).push_back(item);
    while (has_complete_tuple())
    {
        MaybeOutgoingTuples tmp = assemble_output();
        helpers::index_apply<sizeof...(IOs)>(
            [this, &tmp, &lock](auto... Is)
            {
                auto unlock = helpers::with_scoped_unlock(lock);
                this->emit(std::tuple_cat(*std::get<Is>(tmp)...));
            }
        );
    }
}

template<typename... IOs>
bool Fifo<IOs...>::has_complete_tuple() noexcept
{
    bool result = true;
    helpers::for_each_apply<sizeof...(IOs)>(
        [this, &result](auto I)
        {
            if (std::get<I>(this->in_).empty()) result = false;
        }
    );
    return result;
}


template<typename... IOs>
typename Fifo<IOs...>::MaybeOutgoingTuples Fifo<IOs...>::assemble_output() noexcept
{
    MaybeOutgoingTuples tmp;
    helpers::for_each_apply<sizeof...(IOs)>(
        [this, &tmp](auto I)
        {
            std::get<I>(tmp) = std::get<I>(this->in_).front();
            std::get<I>(this->in_).pop_front();
        }
    );
    return tmp;
}

template<typename... IOs>
void Fifo<IOs...>::reset() noexcept
{
    helpers::for_each_apply<sizeof...(IOs)>(
        [this](auto I)
        {
            std::get<I>(this->in_).clear();
        }
    );
}

} // namespace combiner_policies
} // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_FIFO_IMPL_H_ */
