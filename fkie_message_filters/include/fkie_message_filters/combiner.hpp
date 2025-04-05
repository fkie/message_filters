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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_HPP_
#pragma once

#include "helpers/abi_namespace.hpp"
#include "helpers/tuple.hpp"
#include "sink.hpp"
#include "source.hpp"

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

/** \brief Combiner policies.
 *
 * This namespace contains combiner policy classes.
 */
namespace combiner_policies
{
}

/** \brief Combine multiple sources into a single one.
 *
 * Sometimes, a filter pipeline will receive corresponding inputs from different sources, which must be processed
 * together. The combiner provides a policy-driven way to aggregate data from multiple sources into a single sink.
 * Policies can be anything from a simple FIFO to an elaborate approximate time stamp synchronization.
 *
 * The output arity of the filter is the sum of all input arities. For example, given two binary input filters (M1,M2)
 * and (M3,M4), the combiner will create a quaternary output (M1,M2,M3,M4). Furthermore, policies which examine data
 * generally look at the first argument of each input. In this example, the timing policies would match the inputs based
 * on M1 and M3. You can prepend a Selector filter to swap the argument order if you need a different element examined,
 * or the Divider filter to match all arguments independently.
 *
 * \code
 * namespace mf = fkie_message_filters;
 *
 * using InBuffer1 = mf::Buffer<M1, M2>;
 * using InBuffer2 = mf::Buffer<M3, M4>;
 * using Combiner = mf::Combiner<mf::combiner_policies::Fifo, InBuffer1::Output, InBuffer2::Output>;
 * using OutBuffer = mf::Buffer<M1, M2, M3, M4>;
 *
 * InBuffer1 buf1;
 * InBuffer2 buf2;
 * OutBuffer out;
 * Combiner combiner;
 *
 * combiner.connect_to_sources(buf1, buf2);
 * combiner.connect_to_sink(out);
 * \endcode
 *
 * \sa combiner_policies, Divider, Selector
 */
#ifndef DOXYGEN
template<template<typename...> class PolicyTmpl, class... IOs>
class Combiner : public helpers::io_rewrap_t<helpers::io_concat_t<IOs...>, Source>
#else
template<template<typename...> class PolicyTmpl, class... IOs>
class Combiner : public Source<IOs...>
#endif
{
public:
    /** \brief Number of sinks. */
    static constexpr std::size_t NUM_SINKS = sizeof...(IOs);
    /** \brief Array of connection objects. */
    using Connections = std::array<Connection, NUM_SINKS>;
    /** \brief Base class for the Nth sink. */
    template<std::size_t N>
    using SinkType = helpers::io_rewrap_t<helpers::select_nth<N, IOs...>, Sink>;
    /** \brief Class type of the policy that applies to the combiner. */
    using Policy = PolicyTmpl<IOs...>;
    /** \brief Constructor.
     * \arg \c policy Instance of the policy class that applies to the combiner.
     *
     * \nothrow
     */
    explicit Combiner(const Policy& policy = Policy()) noexcept;
    /** \brief Access the sink for the Nth input. */
    template<std::size_t N>
    SinkType<N>& sink() noexcept;
    /** \brief Access the sink for the Nth input. */
    template<std::size_t N>
    const SinkType<N>& sink() const noexcept;
    /** \brief Access combiner policy. */
    const Policy& policy() const noexcept;
    /** \brief Set combiner policy.
     *
     * Any previous policy is superseded. Implies a call to reset().
     *
     * \nothrow
     */
    void set_policy(const Policy& policy) noexcept;
    /** \brief Convenience function to connect all sinks at once.
     *
     * \nothrow
     */
    Connections connect_to_sources(helpers::io_rewrap_t<IOs, Source>&... sources) noexcept;
    /** \brief Disconnect the sinks from their sources.
     *
     * \nothrow
     */
    void disconnect_from_all_sources() noexcept;
    /** \brief Reset filter.
     *
     * This will reset the internal state by calling the PolicyTmpl::reset() method
     *
     * \nothrow
     */
    void reset() noexcept override;
    /** \brief Disconnect from all connected sources and sinks.
     *
     * \nothrow
     */
    void disconnect() noexcept override;

private:
    using IncomingTuples = std::tuple<typename helpers::io_tuple_t<IOs>...>;
    using OutgoingTuple = helpers::io_tuple_t<helpers::io_concat_t<IOs...>>;
#ifndef DOXYGEN
    template<typename... Inputs>
    class CombinerSink : public Sink<Inputs...>
    {
    public:
        using Tuple = std::tuple<Inputs...>;
        using PolicyInFunc = std::function<void(std::unique_lock<std::mutex>&, Tuple&&)>;
        void set_parent(Combiner* parent) noexcept;
        void set_policy_input(const PolicyInFunc& f) noexcept;

    protected:
        void receive(helpers::argument_t<Inputs>... in) override;

    private:
        PolicyInFunc forward_;
        Combiner* parent_{nullptr};
    };
#endif
    std::mutex combiner_mutex_;
    Policy policy_;
    std::tuple<typename IOs::template Rewrap<CombinerSink>...> sinks_;
    void connect_policy() noexcept;
    template<std::size_t N, typename ThisSource, typename... OtherSources>
    void connect_to_sources_impl(Connections& conn, ThisSource& src, OtherSources&... sources) noexcept;
};

FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#include "combiner_impl.hpp"  // IWYU pragma: keep

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_HPP_ */
