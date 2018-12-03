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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_SOURCE_H_
#define INCLUDE_FKIE_MESSAGE_FILTERS_SOURCE_H_

#include "types.h"
#include "filter_base.h"
#include <boost/signals2.hpp>

namespace fkie_message_filters
{

template<typename...> class Sink;

/** \brief Base class for data providers.
 *
 * %In the message filter library, all data flows from sources to sinks. The sources are data providers, which may either
 * be generated synthetically or gathered from other sources, such as ROS topics.
 *
 * Derived classes need to call the send() method to pass actual data to the connected sinks. This class does
 * nothing but track which sinks have been connected.
 *
 * \sa Sink
 */
template<typename... Outputs>
class Source : public virtual FilterBase
{
    template<typename...> friend class Sink;
public:
    /** \brief Number of outputs. */
    static constexpr std::size_t NUM_OUTPUTS = sizeof...(Outputs);
    /** \brief Grouped output types.
     *
     * This type can be used to define sinks with matching types.
     */
    using Output = IO<Outputs...>;

    virtual ~Source() {}
    /** \brief Connect this source to a sink.
     *
     * Can be called multiple times to connect multiple sinks; in that case, the sinks receive data in the same order as they
     * have been connected. This function does basically the same thing as Sink::connect_to_source(), only from the opposite
     * point of view.
     *
     * \arg \c dst the sink that is to be connected
     *
     * \return a connection object that can be used to monitor or sever the created connection
     *
     * \nothrow
     */
    Connection connect_to_sink (Sink<Outputs...>& dst) noexcept;
    /** \brief Disconnect from all connected sinks.
     *
     * Severs the connection to all sinks, turning the send() method into a no-op.
     *
     * \nothrow
     */
    void disconnect_from_all_sinks() noexcept;
    /** \brief Disconnect from all connected sinks.
     *
     * The source implementation calls disconnect_from_all_sinks().
     *
     * \nothrow
     */
    virtual void disconnect() noexcept override;
protected:
    /** \brief Pass data to all connected sinks.
     *
     * \arg \c out data
     *
     * \filterthrow
     */
    void send (const Outputs&... out);
private:
    boost::signals2::signal<void(const Outputs&...)> signal_;
};

template<typename... Outputs>
class Source<IO<Outputs...>> : public Source<Outputs...>
{
};

} // namespace fkie_message_filters

#include "sink.h"
#include "source_impl.h"

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_SOURCE_H_ */
