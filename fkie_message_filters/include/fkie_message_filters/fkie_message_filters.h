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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_FKIE_MESSAGE_FILTERS_H_
#define INCLUDE_FKIE_MESSAGE_FILTERS_FKIE_MESSAGE_FILTERS_H_

#include "types.h"
#include "buffer.h"
#include "camera_publisher.h"
#include "camera_subscriber.h"
#include "combiner.h"
#include "combiner_policies/fifo.h"
#include "combiner_policies/approximate_time.h"
#include "combiner_policies/exact_time.h"
#include "divider.h"
#include "image_publisher.h"
#include "image_subscriber.h"
#include "publisher.h"
#include "selector.h"
#include "simple_user_filter.h"
#include "subscriber.h"
#include "tf_filter.h"
#include "user_filter.h"
#include "user_source.h"

/**
 * \namespace fkie_message_filters
 * \brief Primary namespace
 *
 * \mainpage
 *
 * The fkie_message_filters library is a replacement for the ROS message_filters
 * package. It is written in modern C++ and more type-safe than the original
 * version.
 *
 * \section overview Overview
 *
 * The data flow is modeled with a pipeline metaphor, where data always flows from
 * a source to a sink. A filter is both source and sink for data, possibly with
 * different data types. For integration with ROS, the library provides a number of
 * subscribers and publishers which act as sources or sinks of the data flow.
 *
 * \section getting-started Getting Started
 *
 * While you are free to derive your own classes from the one of the base classes,
 * most programs will want to register a custom callback function for their
 * application logic.
 *
 * The \link fkie_message_filters::SimpleUserFilter SimpleUserFilter \endlink
 * works almost like a regular ROS callback, but it
 * expects a boolean return value that determines if the data is passed on
 * to subsequent filters in the pipeline (if any), or if processing terminates.
 * You can use this type of filter to consume data at the end of the pipeline,
 * or if you want to remove invalid inputs before further processing occurs.
 *
 * The \link fkie_message_filters::UserFilter UserFilter \endlink is more
 * generic and can be used if your filter outputs differ from its inputs. You
 * can implement pretty much any kind of transforming filter.
 * 
 * A third filter \link fkie_message_filters::UserSource UserSource \endlink is
 * a simple data source which can be used as callback in third-party code.
 *
 * As a simple "Hello World" example, consider:
 * \include hello.cpp
 *
 * The user-defined filter accepts a \c std_msgs::String message with a name as
 * input and composes a new \c std_msgs::String message with a personalized greeting
 * as output. Note that each source can have arbitrarily many sinks connected to it,
 * and vice vera, so the simplicity of the three-link chain in this example is by
 * no means a limitation of the library.
 * 
 * \section types Data Types
 *
 * Sources and sinks are strongly typed, i.e., each source will only pass on data
 * of a particular type, and each sink will only accept data of a particular type.
 * The compiler will error out if you try to connect incompatible filters. As the
 * strong typing relies on the C++ template mechanism, the error messages can be
 * quite verbose and difficult to parse (looking at you, GCC). It is very much
 * recommended to use the \c Input and \c Output typedefs which are provided by every
 * filter.
 * 
 * \section n-ary-filters N-ary filters
 * 
 * All sources and sinks support the grouping of multiple data types, where
 * items of different types are combined and passed on as a unit. This is particularly
 * useful to process messages from distinct topics which belong together conceptually,
 * e.g., the \c sensor_msgs::Image and \c sensor_msgs::CameraInfo messages from a
 * calibrated camera.
 * N-ary filters can be created, rearranged, and broken up using the
 * \link fkie_message_filters::Combiner Combiner \endlink,
 * \link fkie_message_filters::Divider Divider \endlink, and
 * \link fkie_message_filters::Selector Selector \endlink filters.
 *
 * \section implementation Implementation Details
 *
 * Generally, the pipeline processing is executed by nested calls to receive and
 * send functions. The library is thread-safe and guarantees basic exception safety,
 * but you are expected to handle your own exceptions in your callbacks.
 * Exceptions which propagate through library code will abort processing
 * for the offending message immediately, even if not all downstream sinks have
 * received the message yet. If there is no upstream user-defined filter that catches
 * the exception, the uncaught exception will eventually terminate the program.
 * The library will detect cycles in the pipeline and abort with a \c std::logic_error
 * exception.
 */
#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_FKIE_MESSAGE_FILTERS_H_ */
