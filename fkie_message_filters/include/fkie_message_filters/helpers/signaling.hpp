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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_HELPERS_SIGNALING_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_HELPERS_SIGNALING_HPP_
#pragma once

#include "abi_namespace.hpp"

#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <type_traits>

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

namespace helpers
{

template<typename... Args>
constexpr bool is_copyable_v = (std::is_copy_constructible_v<Args> && ...);

class Connection;

class SignalBase
{
public:
    virtual bool disconnect(const Connection&) noexcept = 0;
};

template<typename... Args>
class Signal;

class Connection
{
public:
    template<typename... Args>
    friend class Signal;

    Connection() = default;

    bool connected() const noexcept
    {
        return priv_ && priv_->owner;
    }
    bool disconnect() noexcept
    {
        if (priv_ && priv_->owner)
            return priv_->owner->disconnect(*this);
        return false;
    }

private:
    explicit Connection(SignalBase* owner) : priv_(std::make_shared<Private>(owner)) {}

    struct Private
    {
        SignalBase* owner = nullptr;
        Private(SignalBase* o = nullptr) : owner(o) {};
    };
    std::shared_ptr<Private> priv_;
};

class ScopedConnection
{
public:
    ScopedConnection() : holder_(std::make_shared<Holder>(Connection())) {}
    ScopedConnection(const Connection& conn) : holder_(std::make_shared<Holder>(conn)) {}
    operator Connection&()
    {
        return holder_->conn_;
    }
    operator const Connection&() const
    {
        return holder_->conn_;
    }

private:
    struct Holder
    {
        Connection conn_;

        Holder(const Connection& c) : conn_(c) {}
        ~Holder()
        {
            conn_.disconnect();
        }
    };
    std::shared_ptr<Holder> holder_;
};

template<typename... Args>
class Signal : public SignalBase
{
public:
    using SlotCB = std::function<void(Args...)>;

    Signal() = default;
    Signal(const Signal&) = delete;
    Signal& operator=(const Signal&) = delete;
    Signal(Signal&& other)
    {
        std::lock_guard<std::mutex> lock{other.slot_mutex_};
        slots_ = std::move(other.slots_);
        for (Slot& slot : slots_)
        {
            slot.priv->owner = this;
        }
    }
    Signal& operator=(Signal&& other)
    {
        if (this != &other)
        {
            std::lock_guard<std::mutex> lock1{slot_mutex_};
            std::lock_guard<std::mutex> lock2{other.slot_mutex_};
            slots_ = std::move(other.slots_);
            for (Slot& slot : slots_)
            {
                slot.priv->owner = this;
            }
        }
        return *this;
    }

    virtual ~Signal()
    {
        disconnect_all_slots();
    }

    Connection connect(const SlotCB& callback)
    {
        std::lock_guard<std::mutex> lock{slot_mutex_};
        if constexpr (!is_copyable_v<Args...>)
        {
            if (!slots_.empty())
                throw std::logic_error("sources with noncopyable types can only connect to a single sink");
        }
        Connection conn{this};
        slots_.push_back({callback, conn.priv_});
        return conn;
    }

    bool disconnect(const Connection& c) noexcept override
    {
        std::lock_guard<std::mutex> lock{slot_mutex_};
        if (!c.connected())
            return false;
        auto it = std::find_if(slots_.begin(), slots_.end(),
                               [&c](const Slot& slot) -> bool { return slot.priv.get() == c.priv_.get(); });
        if (it != slots_.end())
        {
            it->priv->owner = nullptr;
            slots_.erase(it);
            return true;
        }
        return false;
    }

    void disconnect_all_slots() noexcept
    {
        std::lock_guard<std::mutex> lock{slot_mutex_};
        for (Slot& slot : slots_)
        {
            slot.priv->owner = nullptr;
        }
        slots_.clear();
    }

    template<typename... ForwardedArgs>
    void operator()(ForwardedArgs&&... args) const
    {
        std::unique_lock<std::mutex> lock(slot_mutex_);
        std::list<Slot> tmp = slots_;
        lock.unlock();
        for (const Slot& slot : tmp)
        {
            slot.cb(std::forward<ForwardedArgs&&>(args)...);
        }
    }

private:
    struct Slot
    {
        SlotCB cb;
        std::shared_ptr<Connection::Private> priv;
    };
    mutable std::mutex slot_mutex_;
    std::list<Slot> slots_;
};

}  // namespace helpers
FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#endif
