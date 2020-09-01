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
