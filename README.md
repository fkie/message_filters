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

Sources and sinks are strongly typed, i.e., each source will only pass on data
of a particular type, and each sink will only accept data of a particular type.
The compiler will error out if you try to connect incompatible filters. As the
strong typing relies on the C++ template mechanism, the error messages can be
quite verbose and difficult to parse sometimes (looking at you, GCC).

The library filters support arbitrary arities, i.e., the grouping of multiple
data types, where items of different types are combined and passed on as a
unit. This is particularly useful to process messages from distinct topics
which belong together conceptually, e.g., the `sensor_msgs::Image` and
`sensor_msgs::CameraInfo` messages from a calibrated camera.

Getting Started
---------------

While you are free to derive your own classes from the one of the base classes,
most programs will want to register a custom callback function for their
application logic.

The [SimpleUserFilter](https://fkie.github.io/message_filters/classfkie__message__filters_1_1SimpleUserFilter.html) 
works almost like a regular ROS callback, but it expects a
boolean return value that determines if the data is passed on to subsequent
filters in the pipeline (if any), or if processing terminates. You can use this
type of filter to consume data at the end of the pipeline, or if you want to
remove invalid inputs before further processing occurs.

The [UserFilter](https://fkie.github.io/message_filters/classfkie__message__filters_1_1UserFilter.html)
is more generic and can be used if your filter outputs differ
from its inputs. You can implement pretty much any kind of transforming filter.

The [UserSource](https://fkie.github.io/message_filters/classfkie__message__filters_1_1UserSource.html)
is a simple data source which can be used as callback in third-party code.

As a simple "Hello World" example, consider:

```c++
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <fkie_message_filters/fkie_message_filters.h>

namespace mf = fkie_message_filters;

using StringSubscriber = mf::Subscriber<std_msgs::String, mf::RosMessage>;
using StringPublisher = mf::Publisher<std_msgs::String, mf::RosMessage>;
using GreetingFilter = mf::UserFilter<StringSubscriber::Output, StringPublisher::Input>;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hello");
    ros::NodeHandle nh;
    StringSubscriber sub(nh, "name", 1);
    StringPublisher pub(nh, "greeting", 1);
    GreetingFilter flt;
    flt.set_processing_function(
        [](const std_msgs::String& input, const GreetingFilter::CallbackFunction& output)
        {
            std_msgs::String greeting;
            greeting.data = "Hello, " + input.data + "!";
            output(greeting);
        }
    );
    mf::chain(sub, flt, pub);
    ros::spin();
    return 0;
}
```

The user-defined filter accepts a `std_msgs::String` message with a name as input
and composes a new `std_msgs::String` message with a personalized greeting as
output. Note that each source can have arbitrarily many sinks connected to it,
and vice vera, so the simplicity of the three-link chain in this example is by
no means a limitation of the library.

Available Filters
-----------------

See the [API documentation](https://fkie.github.io/message_filters/) for more
details.

* `Buffer`: Store and forward data
* `CameraPublisher`: Publish consumed data to a ROS camera topics
* `CameraSubscriber`: Subscribe to ROS camera topics as data provider
* `Combiner`: Combine multiple sources into a single one, using one of the
   following policies:
    - `Fifo`: First-In-First-Out
    - `ExactTime`: Exactly matching time stamp
    - `ApproximateTime`: Approximately matching time stamp
* `Divider`: Split N-ary sources into N unary ones
* `ImagePublisher`: Publish consumed data to a ROS image topic
* `ImageSubscriber`: Subscribe to a ROS image topic as data provider
* `Publisher`: Publish consumed data on a ROS topic
* `Selector`: Reorder or reduce an N-ary filter
* `Sequencer`: Enforce correct temporal order
* `SimpleUserFilter`: Simplified filter with user-defined callback function
* `Subscriber`: Subscribe to a ROS topic as data provider
* `TfFilter`: Wait for TF transformations for incoming messages
* `UserFilter`: Generic filter with user-defined callback function
* `UserSource`: Manually operated data source

Implementation Details
----------------------

The pipeline processing is executed by nested calls to receive and
send functions. The library is thread-safe and  guarantees basic exception
safety, but you are expected to handle your own exceptions in your callbacks.
Exceptions which propagate through library code will abort processing for the
offending message immediately, even if not all downstream sinks have received
the message yet. If there is no upstream user-defined filter that catches the
exception, the uncaught exception will eventually terminate the program. The
library will detect cycles in the pipeline and abort with a `std::logic_error`
exception.

Certain filters, such as the `Buffer` or the `TfFilter`, can interoperate with
ROS callback queues for convenient workload scheduling.

