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

#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_HELPERS_SHARED_PTR_COMPAT_H_
#define INCLUDE_FKIE_MESSAGE_FILTERS_HELPERS_SHARED_PTR_COMPAT_H_

#include <memory>
#include <boost/shared_ptr.hpp>

namespace fkie_message_filters
{
namespace helpers
{

template<class SP>
struct Holder
{
    SP sp;
    explicit Holder (const SP& p) : sp(p) {}
    void operator()(...) { sp.reset(); }
};

template<class T>
std::shared_ptr<T> to_std_shared_ptr (const boost::shared_ptr<T>& p)
{
    using H = Holder<std::shared_ptr<T>>;
    if (H* h = boost::get_deleter<H, T>(p))
    {
        return h->sp;
    }
    else
    {
        return std::shared_ptr<T>(p.get(), Holder<boost::shared_ptr<T>>(p));
    }
}

template<class T>
std::shared_ptr<T> to_std_shared_ptr (const std::shared_ptr<T>& p)
{
    return p;
}

template<class T>
boost::shared_ptr<T> to_boost_shared_ptr (const std::shared_ptr<T>& p)
{
    using H = Holder<boost::shared_ptr<T>>;
    if (H* h = std::get_deleter<H, T>(p))
    {
        return h->sp;
    }
    else
    {
        return boost::shared_ptr<T>(p.get(), Holder<std::shared_ptr<T>>(p));
    }
}

template<class T>
boost::shared_ptr<T> to_boost_shared_ptr (const boost::shared_ptr<T>& p)
{
    return p;
}

template<class T, class U>
void convert_shared_ptr (const boost::shared_ptr<T>& from, boost::shared_ptr<U>& to)
{
    to = from;
}

template<class T, class U>
void convert_shared_ptr (const boost::shared_ptr<T>& from, std::shared_ptr<U>& to)
{
    to = to_std_shared_ptr(from);
}

template<class T, class U>
void convert_shared_ptr (const std::shared_ptr<T>& from, std::shared_ptr<U>& to)
{
    to = from;
}

template<class T, class U>
void convert_shared_ptr (const std::shared_ptr<T>& from, boost::shared_ptr<U>& to)
{
    to = to_boost_shared_ptr(from);
}

template<class SP, class T>
SP convert_shared_ptr (const std::shared_ptr<T>& p)
{
    SP sp;
    convert_shared_ptr<T, typename SP::element_type>(p, sp);
    return sp;
}

template<class SP, class T>
SP convert_shared_ptr (const boost::shared_ptr<T>& p)
{
    SP sp;
    convert_shared_ptr<T, typename SP::element_type>(p, sp);
    return sp;
}

} // namespace helpers
} // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_HELPERS_SHARED_PTR_COMPAT_H_ */
