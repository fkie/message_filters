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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_BUFFER_H_
#define INCLUDE_FKIE_MESSAGE_FILTERS_BUFFER_H_


#include "filter.h"
#include <boost/circular_buffer.hpp>
#include <tuple>
#include <mutex>
#include <condition_variable>
#include <ros/callback_queue_interface.h>
#include <ros/node_handle.h>


namespace fkie_message_filters
{

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
 * data processing asynchronously. For instance, you can run computationally expensive algorithms on ROS messages
 * in a different thread without blocking the ROS subscriber callback queue.
 * 
 * The buffer can be used for at least three different use cases:
 * -# <b>Use the buffer as a valve</b><br>
 *    You can toggle between the BufferPolicy::Discard and BufferPolicy::Passthru modes to selectively disable or enable
 *    data processing at specific times. This is the simplest use case without any asynchronous processing.
 * -# <b>Run multiple ROS callback queues</b><br>
 *    If your ROS node runs multiple callback queues, you can use the buffer to bind processing to a particular queue:
 *    \code
 *    namespace mf = fkie_message_filters;
 *
 *    ros::NodeHandle nh;
 *    nh.setCallbackQueue(...);
 *
 *    // Version 1
 *    mf::Buffer<M> buf1(nh, 10);  // will use the callback queue of nh
 *
 *    // Version 2
 *    mf::Buffer<M> buf2(mf::BufferPolicy::Queue, 10);
 *    buf2.set_callback_queue(...); // will explicitly set the ROS callback queue
 *    \endcode
 * -# <b>Run your own thread(s) for data processing</b><br>
 *    This is the most versatile option for advanced users. You can set up your
 *    worker threads as you desire and then call spin(), spin_once() or process_one()
 *    as you see fit.
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
 *    ros::spin();
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
     * \arg \c nh ROS node handle whose callback queue is used for data processing
     * \arg \c max_queue_size the maximum number of queued data items
     *
     * \nothrow
     */
    Buffer (const ros::NodeHandle& nh, std::size_t max_queue_size) noexcept;
    /** \brief Constructor.
     * \arg \c policy the buffer policy
     * \arg \c max_queue_size for the BufferPolicy::Queue policy, the maximum number of queued data items.
     * \arg \c cbq ROS callback queue that is used to process queued data
     *
     * \nothrow
     * \sa set_policy(), set_callback_queue()
     */
    Buffer (BufferPolicy policy = BufferPolicy::Discard, std::size_t max_queue_size = 1, ros::CallbackQueueInterface* cbq = nullptr) noexcept;
    virtual ~Buffer();
    /** \brief Modify the buffer policy.
     *
     * If the new buffer policy is not BufferPolicy::Queue, any pending call to wait(), process_one(), or spin()
     * will return. If the buffer policy is changed to BufferPolicy::Passthru, all pending data
     * is processed immediately before the function returns. If the buffer policy is changed to BufferPolicy::Discard,
     * all pending data is discarded immediately.
     *
     * \arg \c policy the buffer policy
     * \arg \c max_queue_size for the BufferPolicy::Queue policy, the maximum number of queued data items. If zero,
     * the previously set queue size remains unchanged.
     *
     * \warning If you change the policy from BufferPolicy::Queue to BufferPolicy::Passthru and there is still a
     * pending call to process_one(), spin_once(), or spin() in a different thread, some data might be processed
     * in parallel or out of order when the queue is flushed.
     *
     * \filterthrow
     */
    void set_policy (BufferPolicy policy, std::size_t max_queue_size = 0);
    /** \brief Process data with a ROS callback queue.
     *
     * Instead of running your own processing threads, you can use the ROS callback system to schedule data processing
     * whenever new data arrives.
     *
     * \arg \c cbq the ROS callback queue or \c nullptr to disable ROS callbacks.
     *
     * \nothrow
     *
     * \code
     * namespace mf = fkie_message_filters;
     *
     * mf::Buffer<...> buf;
     *
     * buf.set_callback_queue(ros::getGlobalCallbackQueue());
     * ros::spin();
     * \endcode
     */
    void set_callback_queue(ros::CallbackQueueInterface* cbq) noexcept;
    /** \brief Check if the buffer has pending data.
     *
     * \retval true if the current policy is BufferPolicy::Queue and a subsequent call to process_one() or spin_once() will process data.
     * \retval false otherwise
     *
     * \nothrow
     */
    bool has_some() const noexcept;
    /** \brief Wait for pending data.
     *
     * \retval true there is data available for consumption by spin_once()
     * \retval false either the current buffer policy is not BufferPolicy::Queue, or the policy has been
     * changed to something other than BufferPolicy::Queue while the function was waiting, or the ROS node has been shut down.
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
     * changed to something other than BufferPolicy::Queue while the function was waiting, or the ROS node has been shut down.
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
    virtual void receive(const Inputs&... in) override;
private:
    struct Impl;
    class RosCB;
    using QueueElement = std::tuple<Inputs...>;
    void process_some(std::unique_lock<std::mutex>&);
    void send_queue_element(const QueueElement& e);
    std::shared_ptr<Impl> impl_;
    ros::CallbackQueueInterface* cbq_;
};

template<class... Inputs>
class Buffer<IO<Inputs...>> : public Buffer<Inputs...> {};

} // namespace fkie_message_filters

#include "buffer_impl.h"

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_BUFFER_H_ */
