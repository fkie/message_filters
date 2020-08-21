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

TEST(fkie_message_filters_ros_integration, Publisher)
{
    std::size_t received_msgs = 0;
    ros::NodeHandle nh("~");
    ros::Subscriber sub = nh.subscribe<std_msgs::Empty>("publisher_test", 1, [&received_msgs](const std_msgs::EmptyConstPtr&) { ++received_msgs; });
    mf::UserSource<std_msgs::Empty> src;
    mf::Publisher<std_msgs::Empty, mf::RosMessage> pub;
    src.connect_to_sink(pub);
    ASSERT_EQ(0, sub.getNumPublishers());
    pub.advertise(nh, "publisher_test", 1);
    ros::spinOnce();
    ASSERT_EQ(1, sub.getNumPublishers());
    ASSERT_EQ(0, received_msgs);
    src(std_msgs::Empty());
    ros::spinOnce();
    usleep(100000);
    ros::spinOnce();
    ASSERT_EQ(1, received_msgs);
}

TEST(fkie_message_filters_ros_integration, Subscriber)
{
    std::size_t received_msgs = 0;
    ros::NodeHandle nh("~");
    ros::Publisher pub = nh.advertise<std_msgs::Empty>("subscriber_test", 1);
    mf::Subscriber<std_msgs::Empty> sub;
    mf::SimpleUserFilter<ros::MessageEvent<const std_msgs::Empty>> flt;
    sub.connect_to_sink(flt);
    flt.set_processing_function([&received_msgs](const ros::MessageEvent<const std_msgs::Empty>&) -> bool { ++received_msgs; return true; });
    ASSERT_EQ(0, pub.getNumSubscribers());
    sub.subscribe(nh, "subscriber_test", 1);
    ros::spinOnce();
    ASSERT_EQ(1, pub.getNumSubscribers());
    ASSERT_EQ(0, received_msgs);
    pub.publish<std_msgs::Empty>(std_msgs::Empty());
    ros::spinOnce();
    usleep(100000);
    ros::spinOnce();
    ASSERT_EQ(1, received_msgs);
}

int main (int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init (argc, argv, "test_publish_subscribe");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
