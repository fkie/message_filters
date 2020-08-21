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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_SINK_IMPL_H_
#define INCLUDE_FKIE_MESSAGE_FILTERS_SINK_IMPL_H_

#include "sink.h"
#include "helpers/scoped_unlock.h"

namespace fkie_message_filters
{

template<typename... Inputs>
class Sink<Inputs...>::ReentryProtector
{
public:
    ReentryProtector(Sink<Inputs...>& parent)
    : running_(parent.running_), this_id_(std::this_thread::get_id())
    {
        if (!running_.insert(this_id_).second)
        {
            throw std::logic_error("recursive invocation detected");
        }
    }

    ~ReentryProtector()
    {
        running_.erase(this_id_);
    }

private:
    std::set<std::thread::id>& running_;
    const std::thread::id this_id_;
};


template<typename... Inputs>
Connection Sink<Inputs...>::connect_to_source(Source<Inputs...>& src) noexcept
{
    std::lock_guard<std::mutex> lock(mutex_);
    Connection c = src.signal_.connect_extended([this](const Connection& conn, const Inputs&... in) { this->receive_cb(conn, in...); });
    conn_.push_back(c);
    return c;
}

template<typename... Inputs>
void Sink<Inputs...>::disconnect_from_all_sources() noexcept
{
    std::lock_guard<std::mutex> lock(mutex_);
    conn_.clear();
}

template<typename... Inputs>
void Sink<Inputs...>::disconnect() noexcept
{
    disconnect_from_all_sources();
}

template<typename... Inputs>
void Sink<Inputs...>::receive_cb(const Connection&, const Inputs&... in)
{
    std::unique_lock<std::mutex> lock(mutex_);
    conn_.erase(std::remove_if(conn_.begin(), conn_.end(), [](const Connection& c) -> bool { return !c.connected(); }), conn_.end());
    ReentryProtector p{*this};
    auto unlock = helpers::with_scoped_unlock(lock);
    receive(in...);
}

} // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_SINK_IMPL_H_ */
