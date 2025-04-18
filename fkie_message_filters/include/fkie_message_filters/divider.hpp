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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_DIVIDER_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_DIVIDER_HPP_
#pragma once

#include "helpers/abi_namespace.hpp"
#include "helpers/tuple.hpp"
#include "sink.hpp"
#include "source.hpp"

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

/** \brief Split an N-ary source into N unary ones.
 *
 * The divider splits an N-ary source into its constituent elements, so they can be processed independently. It is
 * mostly used as the penultimate pipeline filter to forward message tuples to independent Publisher instances.
 *
 * Technically, the divider acts as one sink and N sources, one for each data type that is passed in. You can connect
 * the sources independently using the source() function.
 *
 * The divider will always completely separate the input arguments. If you want a partial split only, you should use one
 * or more Selector filters instead.
 *
 * \code
 * namespace mf = fkie_message_filters;
 *
 * using BufferIn = mf::Buffer<M1, M2>;
 * using MyDivider = mf::Divider<BufferIn::Output>;
 *
 * BufferIn buf_in;
 * MyDivider div;
 * mf::Publisher<M1, mf::RosMessage> pub1;
 * mf::Publisher<M2, mf::RosMessage> pub2;
 * mf::chain(buf_in, div);
 * div.connect_to_sinks(pub1, pub2);
 * \endcode
 * \sa Combiner
 */
template<class... Inputs>
class Divider : public Sink<Inputs...>
{
public:
    using Sink<Inputs...>::NUM_INPUTS;
    /** \brief Array of connection objects. */
    using Connections = std::array<Connection, NUM_INPUTS>;
    /** \brief Base class of the Nth source. */
    template<std::size_t N>
    using SourceType = helpers::select_nth<N, Source<Inputs>...>;
    /** \brief Access the source for the Nth data element. */
    template<std::size_t N>
    SourceType<N>& source() noexcept;
    /** \brief Access the source for the Nth data element. */
    template<std::size_t N>
    const SourceType<N>& source() const noexcept;
    /** \brief Convenience function to connect all sources at once.
     *
     * \nothrow
     */
    Connections connect_to_sinks(Sink<Inputs>&... sinks) noexcept;
    /** \brief Disconnect all sources from their sinks.
     *
     * \nothrow
     */
    void disconnect_from_all_sinks() noexcept;
    /** \brief Disconnect from all connected sources and sinks.
     *
     * \nothrow
     */
    void disconnect() noexcept override;

protected:
    void receive(helpers::argument_t<Inputs>... in) override;

private:
#ifndef DOXYGEN
    template<class Input>
    class DividerSource : public Source<Input>
    {
    public:
        template<class ForwardedInput>
        void forward(ForwardedInput&& in);
    };
#endif
    std::tuple<DividerSource<Inputs>...> sources_;
    template<std::size_t N, typename ThisInput, typename... OtherInputs>
    void forward_to_sources(ThisInput&& in, OtherInputs&&... ins);
    template<std::size_t N, typename ThisSink, typename... OtherSinks>
    void connect_to_sinks_impl(Connections& conn, ThisSink& sink, OtherSinks&... sinks) noexcept;
};

#ifndef DOXYGEN
template<class... Inputs>
class Divider<IO<Inputs...>> : public Divider<Inputs...>
{
};
#endif

FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#include "divider_impl.hpp"  // IWYU pragma: keep

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_DIVIDER_HPP_ */
