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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_DIVIDER_H_
#define INCLUDE_FKIE_MESSAGE_FILTERS_DIVIDER_H_


#include "source.h"
#include "sink.h"
#include "helpers/tuple.h"

namespace fkie_message_filters
{

/** \brief Split an N-ary source into N unary ones.
 *
 * The divider splits an N-ary source into its constituent elements, so they can be processed independently.
 * It is mostly used as the penultimate pipeline filter to forward message tuples to
 * independent Publisher instances.
 *
 * Technically, the divider acts as one sink and N sources, one for each data type that is passed in.
 * You can connect the sources independently using the source() function.
 *
 * The divider will always completely separate the input arguments. If you want a partial split only,
 * you should use one or more Selector filters instead.
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
    template<std::size_t N> using SourceType = helpers::select_nth<N, Source<Inputs>...>;
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
    void receive (const Inputs&... in) override;
private:
    template<class Input>
    class DividerSource : public Source<Input>
    {
    public:
        void forward(const Input& in);
    };
    std::tuple<DividerSource<Inputs>...> sources_;
    template<std::size_t N, typename ThisInput, typename... OtherInputs>
    void forward_to_sources(const ThisInput& in, const OtherInputs&... ins);
    template<std::size_t N>
    void forward_to_sources();
    template<std::size_t N, typename ThisSink, typename... OtherSinks>
    void connect_to_sinks_impl(Connections& conn, ThisSink& sink, OtherSinks&... sinks) noexcept;
    template<std::size_t N>
    void connect_to_sinks_impl(Connections& conn) noexcept;
};

template<class... Inputs>
class Divider<IO<Inputs...>> : public Divider<Inputs...> {};

} // namespace fkie_message_filters

#include "divider_impl.h"

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_DIVIDER_H_ */
