FKIE Message Filters
====================

Summary
-------

The `fkie_message_filters` library is a replacement for the ROS
`message_filters` package. It is written in modern C++ and more type-safe than
the original version.

The data flow is modeled with a pipeline metaphor, where data always flows from
a source to a sink. A filter is both source and sink for data, possibly with
different data types. For integration with ROS, the library provides a number
of subscribers and publishers which act as sources or sinks of the data flow.

Requirements
------------

The `fkie_message_filters` library requires C++14 or better. Some filters
depend on `image_transport` or `tf2_ros`.

Design
------

The filters are written to be as data agnostic as possible. Therefore, many
filters can process arbitrary data types and are not restricted to ROS
messages. A few filters need access to ROS header information, such as
time stamp or TF frame identifier.

Technically, the pipeline processing is executed by nested calls to receive and
send functions. The library is thread-safe and exception-safe, but you are
expected to handle your own exceptions in your callbacks. Exceptions which
propagate through the library will abort processing for the message that caused
the exception, and if not caught eventually by a preceding filter in the
pipeline, terminate the program.

Certain filters, such as the `Buffer` or the `TfFilter`, can interoperate with
ROS callback queues for convenient workload scheduling.

Available Filters
-----------------

See the [API documentation](https://fkie.github.io/message_filters/) for more
details.

* `Buffer`: Store and forward data
* `CameraPublisher`: Publish consumed data to a ROS camera topic
* `CameraSubscriber`: Subscribe to a ROS camera topic as data provider
* `Combiner`: Combine multiple sinks into a single source with one of the
   following policies:
    - `Fifo`: First-In-First-Out
    - `ExactTime`: Exactly matching time stamp
    - `ApproximateTime`: Approximately matching time stamp
* `Divider`: Split source with multiple inputs into independent output sinks
* `ImagePublisher`: Publish consumed data to a ROS image topic
* `ImageSubscriber`: Subscribe to a ROS image topic as data provider
* `Publisher`: Publish consumed data on a ROS topic
* `Selector`: Choose an arbitrary set of inputs to be forwarded
* `Sequencer`: Enforce correct temporal order
* `SimpleUserFilter`: Simplified filter with user-defined callback function
* `Subscriber`: Subscribe to a ROS topic as data provider
* `TfFilter`: Wait for TF transformations for incoming messages
* `UserFilter`: Generic filter with user-defined callback function
* `UserSource`: Manually operated data source

Customized Filters
------------------

While you are free to derive your own classes from the one of the base classes,
most programs will want to register a custom callback function for their
application logic.

Unlike the original `message_filters`, which mixed custom callbacks
and filter chaining, this library keeps those concepts distinct. Therefore,
there are two dedicated filters, `UserFilter` and `SimpleUserFilter`, for
callback processing.

The `SimpleUserFilter` works almost like a regular ROS callback, but it
expects a boolean return value that determines if the data is passed on
to subsequent filters in the pipeline (if any), or if processing terminates.
You can use this type of filter to consume data at the end of the pipeline,
or if you want to remove invalid inputs before further processing occurs.

The `UserFilter` is more generic and can be used if your filter
outputs differ from its inputs. You can implement pretty much any kind of
transforming filter.

A third filter `UserSource` is a simple data source which can be used as
callback in third-party code.

