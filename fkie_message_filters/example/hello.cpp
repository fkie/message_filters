#include <fkie_message_filters/fkie_message_filters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace mf = fkie_message_filters;

using StringSubscriber = mf::Subscriber<std_msgs::msg::String, mf::RosMessage>;
using StringPublisher = mf::Publisher<std_msgs::msg::String, mf::RosMessage>;
using GreetingFilter = mf::UserFilter<StringSubscriber::Output, StringPublisher::Input>;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("hello");
    StringSubscriber sub(node, "name");
    StringPublisher pub(node, "greeting");
    GreetingFilter flt;
    flt.set_processing_function(
        [](const std_msgs::msg::String& input, const GreetingFilter::CallbackFunction& output)
        {
            std_msgs::msg::String greeting;
            greeting.data = "Hello, " + input.data + "!";
            output(greeting);
        });
    mf::chain(sub, flt, pub);
    rclcpp::spin(node);
    return 0;
}
