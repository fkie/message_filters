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

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <std_msgs/Empty.h>
#include <fkie_message_filters/user_source.h>
#include <fkie_message_filters/simple_user_filter.h>
#include <fkie_message_filters/publisher.h>
#include <fkie_message_filters/subscriber.h>

namespace mf = fkie_message_filters;

namespace {
    void process_pending_events()
    {
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(50ms); // Apparently, we need to allow for some time to complete the message round trip
        ros::spinOnce();
    }
}

TEST(fkie_message_filters_ros_integration, PublisherMessage)
{
    std::size_t received_msgs = 0;
    ros::NodeHandle nh("~");
    ros::Subscriber sub = nh.subscribe<std_msgs::Empty>("publisher_test", 1, [&received_msgs](const std_msgs::EmptyConstPtr&) { ++received_msgs; });
    mf::UserSource<std_msgs::Empty> src;
    mf::Publisher<std_msgs::Empty, mf::RosMessage> pub;
    src.connect_to_sink(pub);
    ASSERT_EQ(0, sub.getNumPublishers());
    pub.advertise(nh, "publisher_test", 1);
    process_pending_events();
    ASSERT_EQ(1, sub.getNumPublishers());
    ASSERT_EQ(0, received_msgs);
    src(std_msgs::Empty());
    process_pending_events();
    ASSERT_EQ(1, received_msgs);
}

TEST(fkie_message_filters_ros_integration, PublisherMessageConstPtr)
{
    std::size_t received_msgs = 0;
    ros::NodeHandle nh("~");
    ros::Subscriber sub = nh.subscribe<std_msgs::Empty>("publisher_test", 1, [&received_msgs](const std_msgs::EmptyConstPtr&) { ++received_msgs; });
    mf::UserSource<std_msgs::Empty::ConstPtr> src;
    mf::Publisher<std_msgs::Empty, mf::RosMessageConstPtr> pub;
    src.connect_to_sink(pub);
    ASSERT_EQ(0, sub.getNumPublishers());
    pub.advertise(nh, "publisher_test", 1);
    process_pending_events();
    ASSERT_EQ(1, sub.getNumPublishers());
    ASSERT_EQ(0, received_msgs);
    src(std_msgs::Empty::ConstPtr(new std_msgs::Empty()));
    process_pending_events();
    ASSERT_EQ(1, received_msgs);
}

TEST(fkie_message_filters_ros_integration, PublisherMessageStdSharedPtr)
{
    std::size_t received_msgs = 0;
    ros::NodeHandle nh("~");
    ros::Subscriber sub = nh.subscribe<std_msgs::Empty>("publisher_test", 1, [&received_msgs](const std_msgs::EmptyConstPtr&) { ++received_msgs; });
    mf::UserSource<std::shared_ptr<const std_msgs::Empty>> src;
    mf::Publisher<std_msgs::Empty, mf::RosMessageStdSharedPtr> pub;
    src.connect_to_sink(pub);
    ASSERT_EQ(0, sub.getNumPublishers());
    pub.advertise(nh, "publisher_test", 1);
    process_pending_events();
    ASSERT_EQ(1, sub.getNumPublishers());
    ASSERT_EQ(0, received_msgs);
    src(std::shared_ptr<const std_msgs::Empty>(new std_msgs::Empty()));
    process_pending_events();
    ASSERT_EQ(1, received_msgs);
}

TEST(fkie_message_filters_ros_integration, PublisherMessageEvent)
{
    std::size_t received_msgs = 0;
    ros::NodeHandle nh("~");
    ros::Subscriber sub = nh.subscribe<std_msgs::Empty>("publisher_test", 1, [&received_msgs](const std_msgs::EmptyConstPtr&) { ++received_msgs; });
    mf::UserSource<ros::MessageEvent<const std_msgs::Empty>> src;
    mf::Publisher<std_msgs::Empty, mf::RosMessageEvent> pub;
    src.connect_to_sink(pub);
    ASSERT_EQ(0, sub.getNumPublishers());
    pub.advertise(nh, "publisher_test", 1);
    process_pending_events();
    ASSERT_EQ(1, sub.getNumPublishers());
    ASSERT_EQ(0, received_msgs);
    src(ros::MessageEvent<const std_msgs::Empty>(std_msgs::Empty::ConstPtr(new std_msgs::Empty())));
    process_pending_events();
    ASSERT_EQ(1, received_msgs);
}

TEST(fkie_message_filters_ros_integration, PublisherWithSubscriberCB)
{
    std::size_t received_msgs = 0, connected_cbs = 0, disconnected_cbs = 0;
    ros::NodeHandle nh("~");
    mf::UserSource<std_msgs::Empty> src;
    mf::Publisher<std_msgs::Empty, mf::RosMessage> pub;
    src.connect_to_sink(pub);
    pub.advertise(nh, "publisher_test", 1,
        [&connected_cbs](const ros::SingleSubscriberPublisher&) { ++connected_cbs; },
        [&disconnected_cbs](const ros::SingleSubscriberPublisher&) { ++disconnected_cbs; }
    );
    process_pending_events();
    ASSERT_EQ(false, pub.is_active());
    ASSERT_EQ(0, connected_cbs);
    ASSERT_EQ(0, disconnected_cbs);
    ros::Subscriber sub = nh.subscribe<std_msgs::Empty>("publisher_test", 1, [&received_msgs](const std_msgs::EmptyConstPtr&) { ++received_msgs; });
    process_pending_events();
    ASSERT_EQ(1, sub.getNumPublishers());
    ASSERT_EQ(0, received_msgs);
    ASSERT_EQ(1, connected_cbs);
    ASSERT_EQ(0, disconnected_cbs);
    src(std_msgs::Empty());
    process_pending_events();
    ASSERT_EQ(1, received_msgs);
    sub.shutdown();
    process_pending_events();
    ASSERT_EQ(1, connected_cbs);
    ASSERT_EQ(1, disconnected_cbs);
}

TEST(fkie_message_filters_ros_integration, SubscriberMessageEvent)
{
    std::size_t received_msgs = 0;
    ros::NodeHandle nh("~");
    ros::Publisher pub = nh.advertise<std_msgs::Empty>("subscriber_test", 1);
    mf::Subscriber<std_msgs::Empty, mf::RosMessageEvent> sub;
    mf::SimpleUserFilter<ros::MessageEvent<const std_msgs::Empty>> flt;
    sub.connect_to_sink(flt);
    flt.set_processing_function([&received_msgs](const ros::MessageEvent<const std_msgs::Empty>&) -> bool { ++received_msgs; return true; });
    ASSERT_EQ(0, pub.getNumSubscribers());
    sub.subscribe(nh, "subscriber_test", 1);
    process_pending_events();
    ASSERT_EQ(1, pub.getNumSubscribers());
    ASSERT_EQ(0, received_msgs);
    pub.publish<std_msgs::Empty>(std_msgs::Empty());
    process_pending_events();
    ASSERT_EQ(1, received_msgs);
}

TEST(fkie_message_filters_ros_integration, SubscriberMessageConstPtr)
{
    std::size_t received_msgs = 0;
    ros::NodeHandle nh("~");
    ros::Publisher pub = nh.advertise<std_msgs::Empty>("subscriber_test", 1);
    mf::Subscriber<std_msgs::Empty, mf::RosMessageConstPtr> sub;
    mf::SimpleUserFilter<std_msgs::Empty::ConstPtr> flt;
    sub.connect_to_sink(flt);
    flt.set_processing_function([&received_msgs](const std_msgs::Empty::ConstPtr&) -> bool { ++received_msgs; return true; });
    ASSERT_EQ(0, pub.getNumSubscribers());
    sub.subscribe(nh, "subscriber_test", 1);
    process_pending_events();
    ASSERT_EQ(1, pub.getNumSubscribers());
    ASSERT_EQ(0, received_msgs);
    pub.publish<std_msgs::Empty>(std_msgs::Empty());
    process_pending_events();
    ASSERT_EQ(1, received_msgs);
}

TEST(fkie_message_filters_ros_integration, SubscriberMessageStdSharedPtr)
{
    std::size_t received_msgs = 0;
    ros::NodeHandle nh("~");
    ros::Publisher pub = nh.advertise<std_msgs::Empty>("subscriber_test", 1);
    mf::Subscriber<std_msgs::Empty, mf::RosMessageStdSharedPtr> sub;
    mf::SimpleUserFilter<std::shared_ptr<const std_msgs::Empty>> flt;
    sub.connect_to_sink(flt);
    flt.set_processing_function([&received_msgs](const std::shared_ptr<const std_msgs::Empty>&) -> bool { ++received_msgs; return true; });
    ASSERT_EQ(0, pub.getNumSubscribers());
    sub.subscribe(nh, "subscriber_test", 1);
    process_pending_events();
    ASSERT_EQ(1, pub.getNumSubscribers());
    ASSERT_EQ(0, received_msgs);
    pub.publish<std_msgs::Empty>(std_msgs::Empty());
    process_pending_events();
    ASSERT_EQ(1, received_msgs);
}

TEST(fkie_message_filters_ros_integration, SubscriberMessage)
{
    std::size_t received_msgs = 0;
    ros::NodeHandle nh("~");
    ros::Publisher pub = nh.advertise<std_msgs::Empty>("subscriber_test", 1);
    mf::Subscriber<std_msgs::Empty, mf::RosMessage> sub;
    mf::SimpleUserFilter<std_msgs::Empty> flt;
    sub.connect_to_sink(flt);
    flt.set_processing_function([&received_msgs](const std_msgs::Empty&) -> bool { ++received_msgs; return true; });
    ASSERT_EQ(0, pub.getNumSubscribers());
    sub.subscribe(nh, "subscriber_test", 1);
    process_pending_events();
    ASSERT_EQ(1, pub.getNumSubscribers());
    ASSERT_EQ(0, received_msgs);
    pub.publish<std_msgs::Empty>(std_msgs::Empty());
    process_pending_events();
    ASSERT_EQ(1, received_msgs);
}

int main (int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init (argc, argv, "test_publish_subscribe");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
