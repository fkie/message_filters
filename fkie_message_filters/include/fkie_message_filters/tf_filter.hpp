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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_TF_FILTER_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_TF_FILTER_HPP_
#pragma once

#include "filter.hpp"
#include "helpers/abi_namespace.hpp"

#include <tf2/buffer_core.hpp>

#include <memory>
#include <vector>

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

/** \brief TF transformation results.
 *
 * These results are used to indiciate specific failure modes in the TfFilter failure callback.
 * \sa TfFilter::set_filter_failure_function()
 */
enum class TfFilterResult
{
    /** \brief Success */
    TransformAvailable,
    /** \brief The message has an empty TF frame ID and cannot be transformed. */
    EmptyFrameID,
    /** \brief The requested transform is no longer available, likely because the message is too old. */
    TransformExpired,
    /** \brief The requested transform is unavailable. */
    TransformFailed,
    /** \brief The message has been dropped because of a queue overflow. */
    QueueOverflow,
    /** \brief Unknown failure reason. */
    UnknownFailure
};

/** \brief Wait for TF transformations for incoming messages.
 *
 * This filter is intended to be used with a Subscriber as source, and will delay incoming messages until they can be
 * transformed to the specified TF target frames. If the filter input is not unary, only the first argument is examined.
 *
 * \code
 * namespace mf = fkie_message_filters;
 *
 * mf::Subscriber<M> sub;
 * mf::TfFilter<mf::Subscriber<M>::Output> flt;
 * ros::NodeHandle nh;
 *
 * sub.subscribe(nh, "topic", 10);
 * flt.init(tf2_buffer, "target_frame", 10, nh);
 * mf::chain(sub, flt);
 * \endcode
 */
template<class... Inputs>
class TfFilter : public Filter<IO<Inputs...>, IO<Inputs...>>
{
public:
    /** \brief Type alias for \c std::vector<std::string> */
    using V_string = std::vector<std::string>;

    /** \brief Callback for failed transform queries. */
    using FilterFailureCB = std::function<void(const Inputs&..., TfFilterResult)>;

    /** \brief Empty constructor.
     *
     * Constructs an uninitialized filter object. You need to call init() before you can use the object.
     */
    TfFilter() noexcept {}
    /** \brief Construct and initialize the filter.
     *
     * The constructor calls init() and set_target_frame() for you.
     *
     * \arg \c bc a tf2 buffer instance
     * \arg \c target_frame the TF target frame for the incoming messages
     * \arg \c queue_size the maximum number of queued messages
     *
     * \nothrow
     */
    TfFilter(tf2::BufferCore& bc, const std::string& target_frame, uint32_t queue_size) noexcept;
    /** \brief Construct and initialize the filter.
     *
     * The constructor calls init() and set_target_frame() for you.
     *
     * \arg \c bc a tf2 buffer instance
     * \arg \c target_frames the TF target frames for the incoming messages
     * \arg \c queue_size the maximum number of queued messages
     *
     * \nothrow
     */
    TfFilter(tf2::BufferCore& bc, const V_string& target_frames, uint32_t queue_size) noexcept;
    /** \brief Initialize the filter.
     *
     * This function allocates internal data structures and makes the filter operational. If the function is called on
     * an already initialized filter, the filter is reinitialized and reset() is called implicitly.
     *
     * \arg \c bc a TF2 buffer instance
     * \arg \c queue_size the maximum number of queued messages
     *
     * \nothrow
     */
    void init(tf2::BufferCore& bc, uint32_t queue_size) noexcept;
    /** \brief Choose the TF target frame.
     *
     * \arg \c target_frame all passed messages will be transformable to this TF frame
     *
     * \throw std::logic_error if the filter is uninitialized
     */
    void set_target_frame(const std::string& target_frame);
    /** \brief Choose the TF target frames.
     *
     * \arg \c target_frames all passed messages will be transformable to all these TF frames
     *
     * \throw std::logic_error if the filter is uninitialized
     */
    void set_target_frames(const V_string& target_frames);
    /** \brief Reset filter state.
     *
     * Discards all queued messages. Existing connections to sources and sinks are unaffected.
     *
     * \nothrow
     */
    void reset() noexcept override;
    /** \brief Register callback for failed transforms.
     *
     * Whenever a message is discarded, this callback is called with the message and the reason why the
     * TF transform failed.
     *
     * \arg \c cb callback function
     *
     * \throw std::logic_error if the filter is uninitialized
     */
    void set_filter_failure_function(FilterFailureCB cb);

protected:
    void receive(helpers::argument_t<Inputs>... in) override;

private:
    using MessageTuple = std::tuple<Inputs...>;
    struct Impl;
    std::shared_ptr<Impl> impl_;
    void transformable(tf2::TransformableRequestHandle request_handle, const std::string& target_frame,
                       const std::string& source_frame, tf2::TimePoint time, tf2::TransformableResult result);
    void report_failure(std::unique_lock<std::mutex>&, MessageTuple&, TfFilterResult);
    void send_message(std::unique_lock<std::mutex>&, MessageTuple& m);
};

#ifndef DOXYGEN
template<class... Inputs>
class TfFilter<IO<Inputs...>> : public TfFilter<Inputs...>
{
public:
    using TfFilter<Inputs...>::TfFilter;
};
#endif

FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#include "tf_filter_impl.hpp"  // IWYU pragma: keep

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_TF_FILTER_HPP_ */
