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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_BUFFER_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_BUFFER_HPP_
#pragma once

#include "filter.hpp"
#include "helpers/abi_namespace.hpp"

#include <memory>
#include <mutex>
#include <tuple>

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

/** \brief %Buffer policy.
 *
 * An enumeration of the possible policies which can be enacted by a Buffer.
 */
enum class BufferPolicy
{
    /** \brief Discard all data.
     *
     * Turns the buffer into a black hole that lets all incoming data vanish.
     */
    Discard,
    /** \brief Queue for later use.
     *
     * Stores incoming data for until consumed by .
     */
    Queue,
    /** \brief Forward data immediately.
     *
     * All data is forwarded to the connected sinks immediately, as if the buffer did not exist.
     */
    Passthru
};

/** \brief Store and forward data.
 *
 * The buffer acts as a decoupler in the filter pipeline. Data can be stored and processed at a later time. The pipeline
 * is effectively split into independent upstream and downstream parts, and it becomes possible to run the downstream
 * data processing asynchronously. For instance, you can run computationally expensive algorithms on ROS messages in a
 * different thread without blocking the ROS subscriber callback queue.
 *
 * The buffer can be used for at least three different use cases:
 *
 * -# <b>Use the buffer as a valve</b><br>
 *    You can toggle between the BufferPolicy::Discard and BufferPolicy::Passthru modes to selectively disable or enable
 *    data processing at specific times. This is the simplest use case without any asynchronous processing.
 * -# <b>Run multiple ROS callback groups</b><br>
 *    You can let the buffer processing happen in a dedicated ROS 2 callback group. This is the simplest way to deal
 *    with computationally expensive message processing:
 *    \code
 *    namespace mf = fkie_message_filters;
 *
 *    rclcpp::Node::SharedPtr node = ...;
 *    mf::Buffer<M> buf1(node, 10);  // will create a callback group from node
 *    \endcode
 * -# <b>Run your own thread(s) for data processing</b><br>
 *    This is the most flexible option for advanced users. You can set up your worker threads as you desire and then
 *    call spin(), spin_once() or process_one() as you see fit.
 *
 *    \code
 *    namespace mf = fkie_message_filters;
 *
 *    mf::Subscriber<M> sub;
 *    mf::Buffer<mf::Subscriber<M>::Output> buffer(mf::BufferPolicy::Queue, 10);
 *    mf::SimpleUserFilter<mf::Subscriber<M>::Output> flt;
 *
 *    std::thread t([&buffer]{ buffer.spin(); });
 *    flt.set_processing_function(...);
 *    mf::chain(sub, buffer, flt);
 *    \endcode
 */
template<class... Inputs>
class Buffer : public Filter<IO<Inputs...>, IO<Inputs...>>
{
public:
    /** \brief Constructor.
     *
     * Constructs a buffer with BufferPolicy::Queue policy and data processing via ROS callbacks.
     *
     * \arg \c node ROS node instance which will host a dedicated callback group for data processing
     * \arg \c max_queue_size the maximum number of queued data items
     *
     * \nothrow
     */
    template<class NodeT>
    Buffer(NodeT&& node, std::size_t max_queue_size) noexcept;
    /** \brief Constructor.
     * \arg \c policy the buffer policy
     * \arg \c max_queue_size for the BufferPolicy::Queue policy, the maximum number of queued data items.
     *
     * \nothrow
     * \sa set_policy(), set_callback_queue()
     */
    Buffer(BufferPolicy policy = BufferPolicy::Discard, std::size_t max_queue_size = 1) noexcept;
    virtual ~Buffer();
    /** \brief Modify the buffer policy.
     *
     * If the new buffer policy is not BufferPolicy::Queue, any pending call to wait(), process_one(), or spin() will
     * return. If the buffer policy is changed to BufferPolicy::Passthru, all pending data is processed immediately
     * before the function returns. If the buffer policy is changed to BufferPolicy::Discard, all pending data is
     * discarded immediately.
     *
     * \arg \c policy the buffer policy \arg \c max_queue_size for the BufferPolicy::Queue policy, the maximum number of
     * queued data items. If zero, the previously set queue size remains unchanged.
     *
     * \warning If you change the policy from BufferPolicy::Queue to BufferPolicy::Passthru and there is still a pending
     * call to process_one(), spin_once(), or spin() in a different thread, some data might be processed in parallel or
     * out of order when the queue is flushed.
     *
     * \filterthrow
     */
    void set_policy(BufferPolicy policy, std::size_t max_queue_size = 0);
    /** \brief Process data in a dedicated ROS callback group.
     *
     * Instead of running your own processing threads, you can use the ROS callback system to schedule data processing
     * whenever new data arrives.
     *
     * \arg \c node the ROS node or \c nullptr to disable ROS callbacks.
     *
     * \rmwthrow
     *
     * \code
     * namespace mf = fkie_message_filters;
     *
     * mf::Buffer<...> buf;
     * rclcpp::Node::SharedPtr node = ...;
     *
     * buf.set_node(node);
     * rclcpp::spin(node);
     * \endcode
     */
    template<class NodeT>
    void set_node(NodeT&& node);
    /** \brief Check if the buffer has pending data.
     *
     * \retval true if the current policy is BufferPolicy::Queue and a subsequent call to process_one() or spin_once()
     * will process data. \retval false otherwise
     *
     * \nothrow
     */
    bool has_some() const noexcept;
    /** \brief Wait for pending data.
     *
     * \retval true there is data available for consumption by spin_once()
     * \retval false either the current buffer policy is not BufferPolicy::Queue, or the policy has been
     * changed to something other than BufferPolicy::Queue while the function was waiting, or the ROS node has been shut
     * down.
     *
     * \nothrow
     */
    bool wait() noexcept;
    /** \brief Wait for pending data until timeout expires.
     *
     * \arg \c timeout maximum duration to wait for new data if none is available
     *
     * \retval true there is data available for consumption by spin_once()
     * \retval false either the current buffer policy is not BufferPolicy::Queue, or the policy has been
     * changed to something other than BufferPolicy::Queue while the function was waiting, or the timeout expired,
     * or the ROS node has been shut down.
     *
     * \nothrow
     */
    template<class Rep, class Period>
    bool wait_for(const std::chrono::duration<Rep, Period>& timeout) noexcept;
    /** \brief Wait for and process one data item.
     *
     * \retval true data has been processed successfully
     * \retval false either the current buffer policy is not BufferPolicy::Queue, or the policy has been
     * changed to something other than BufferPolicy::Queue while the function was waiting, or the ROS node has been shut
     * down.
     *
     * \filterthrow
     */
    bool process_one();
    /** \brief Wait for and process one data item.
     *
     * \arg \c timeout maximum duration to wait for new data if none is available
     *
     * \retval true data has been processed successfully
     * \retval false either the current buffer policy is not BufferPolicy::Queue, or the policy has been
     * changed to something other than BufferPolicy::Queue while the function was waiting, or time timeout expired,
     * or the ROS node has been shut down.
     *
     * \filterthrow
     */
    template<class Rep, class Period>
    bool process_one(const std::chrono::duration<Rep, Period>& timeout);
    /** \brief Process pending data.
     *
     * Does nothing if the buffer policy is not BufferPolicy::Queue.
     * The method is guaranteed to return as it will only process data which is pending at invocation time.
     * This also means that there may be new data pending already when this method returns.
     *
     * \filterthrow
     */
    void spin_once();
    /** \brief Process all data indefinitely.
     *
     * Blocks and processes all incoming data until the buffer policy is changed to something
     * other than BufferPolicy::Queue or the ROS node is shut down.
     *
     * You can call the function from multiple threads at once, and the workload will be shared among all participating
     * threads.
     *
     * \filterthrow
     * \sa set_policy()
     */
    void spin();
    /** \brief Reset filter.
     *
     * If the buffer policy is BufferPolicy::Queue, this will clear the internal queue and discard all pending data.
     * Otherwise, this function has no effect.
     *
     * \nothrow
     */
    void reset() noexcept override;

protected:
    virtual void receive(helpers::argument_t<Inputs>... in) override;

private:
    struct Impl;
    using QueueElement = std::tuple<Inputs...>;
    void process_some(std::unique_lock<std::mutex>&);
    void send_queue_element(QueueElement& e);
    std::shared_ptr<Impl> impl_;
};

#ifndef DOXYGEN
template<class... Inputs>
class Buffer<IO<Inputs...>> : public Buffer<Inputs...>
{
public:
    using Buffer<Inputs...>::Buffer;
};
#endif

FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#include "buffer_impl.hpp"  // IWYU pragma: keep

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_BUFFER_HPP_ */
