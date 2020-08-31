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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_SINK_H_
#define INCLUDE_FKIE_MESSAGE_FILTERS_SINK_H_

#include "types.h"
#include "filter_base.h"
#include <vector>
#include <set>
#include <mutex>
#include <thread>

namespace fkie_message_filters
{

template<typename...> class Source;

/** \brief Base class for data consumers.
 *
 * %In the message filter library, all data flows from sources to sinks. The sinks are data consumers, which process
 * all data they receive from a source.
 *
 * Derived classes must override the receive() method to actually process data. The receive() method takes the same
 * number and types of arguments as specified in the template instantiation.
 *
 * \sa Source
 */
template<typename... Inputs>
class Sink : public virtual FilterBase
{
    template<typename...> friend class Source;
public:
    /** \brief Number of input arguments. */
    static constexpr std::size_t NUM_INPUTS = sizeof...(Inputs);
    /** \brief Grouped input types.
     *
     * This type can be used to define sources with matching types.
     */
    using Input = IO<Inputs...>;

    virtual ~Sink() {}
    /** \brief Connect this sink to a source.
     *
     * Can be called multiple times to connect multiple sources; in that case, the sink receives data from all connected
     * sources. This function does basically the same thing as Source::connect_to_sink(), only from the opposite
     * point of view.
     *
     * \arg \c src the source that is to be connected
     *
     * \return a connection object that can be used to monitor or sever the created connection
     * \nothrow
     */
    Connection connect_to_source (Source<Inputs...>& src) noexcept;
    /** \brief Disconnect from all connected sources.
     *
     * Severs the connection to all sources. The receive() method will not be called any more.
     *
     * \nothrow
     */
    void disconnect_from_all_sources() noexcept;
    /** \brief Disconnect from all connected sources.
     *
     * The sink implementation calls disconnect_from_all_sources().
     *
     * \nothrow
     */
    virtual void disconnect() noexcept override;
protected:
    /** \brief Process incoming data.
     *
     * Derived classes need to override this method to handle all data that is to be consumed by the sink.
     *
     * \abstractthrow
     */
    virtual void receive(const Inputs&... in) = 0;
private:
    class ReentryProtector;
    void receive_cb (const Connection&, const Inputs&... in);
    std::vector<boost::signals2::scoped_connection> conn_;
    std::mutex mutex_;
    std::set<std::thread::id> running_;
};

template<typename... Inputs>
class Sink<IO<Inputs...>> : public Sink<Inputs...>
{
};

} // namespace fkie_message_filters

#include "source.h"
#include "sink_impl.h"

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_SINK_H_ */
