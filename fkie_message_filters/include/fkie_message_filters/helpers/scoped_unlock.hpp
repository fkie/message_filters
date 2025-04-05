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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_HELPERS_SCOPED_UNLOCK_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_HELPERS_SCOPED_UNLOCK_HPP_
#pragma once

#include "abi_namespace.hpp"

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

namespace helpers
{

template<class BasicLockable>
class ScopedUnlock
{
public:
    ScopedUnlock(BasicLockable& lockable) : lockable_(lockable), owns_lock_(lockable_.owns_lock())
    {
        if (owns_lock_)
            lockable_.unlock();
    }

    ScopedUnlock(ScopedUnlock&& other) noexcept : lockable_(other.lockable_), owns_lock_(other.owns_lock_)
    {
        other.owns_lock_ = false;
    }

    ~ScopedUnlock()
    {
        if (owns_lock_)
            lockable_.lock();
    }

    ScopedUnlock(const ScopedUnlock&) = delete;
    ScopedUnlock& operator=(const ScopedUnlock&) = delete;

private:
    BasicLockable& lockable_;
    bool owns_lock_;
};

template<class BasicLockable>
ScopedUnlock<BasicLockable> with_scoped_unlock(BasicLockable& lockable)
{
    return ScopedUnlock<BasicLockable>(lockable);
}

}  // namespace helpers
FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_HELPERS_SCOPED_UNLOCK_HPP_ */
