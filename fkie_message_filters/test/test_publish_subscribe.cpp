/****************************************************************************
 *
 * fkie_message_filters
 * Copyright © 2018-2025 Fraunhofer FKIE
 * Author: Timo Röhling
 * SPDX-License-Identifier: Apache-2.0
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

#include "test.hpp"

#include <fkie_message_filters/camera_publisher.hpp>
#include <fkie_message_filters/camera_subscriber.hpp>
#include <fkie_message_filters/image_publisher.hpp>
#include <fkie_message_filters/image_subscriber.hpp>
#include <fkie_message_filters/publisher.hpp>
#include <fkie_message_filters/simple_user_filter.hpp>
#include <fkie_message_filters/subscriber.hpp>
#include <fkie_message_filters/user_source.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_interfaces/get_node_base_interface.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/empty.hpp>

namespace mf = fkie_message_filters;

namespace
{

template<class NodeT>
void process_pending_events(NodeT&& node)
{
    rclcpp::spin_some(rclcpp::node_interfaces::get_node_base_interface(std::forward<NodeT&&>(node)));
}
}  // namespace

template<class Node, class Source, class Publisher, class Subscriber, class MessageCreator>
void common_publisher_test_code(Node& node, Source& src, Publisher& pub, Subscriber& sub, std::size_t& received_msgs,
                                MessageCreator create)
{
    src.connect_to_sink(pub);
    ASSERT_EQ(0, get_publisher_count(sub));
    ASSERT_FALSE(pub.is_active());
    pub.advertise(node, "publisher_test", 1);
    ASSERT_EQ("/publisher_test", pub.topic());
    process_pending_events(node);
    ASSERT_EQ(1, get_publisher_count(sub));
    ASSERT_TRUE(pub.is_active());
    ASSERT_EQ(0, received_msgs);
    auto msg = create();
    src(mf::helpers::maybe_move(msg));
    process_pending_events(node);
    ASSERT_EQ(1, received_msgs);
}

TEST(fkie_message_filters, PublisherMessage)
{
    std::size_t received_msgs = 0;
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("publisher_message");
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub = node->create_subscription<std_msgs::msg::Empty>(
        "publisher_test", 1, [&received_msgs](const std_msgs::msg::Empty::ConstSharedPtr&) { ++received_msgs; });
    mf::UserSource<std_msgs::msg::Empty> src;
    mf::Publisher<std_msgs::msg::Empty, mf::RosMessage> pub;
    common_publisher_test_code(node, src, pub, sub, received_msgs,
                               []() -> std_msgs::msg::Empty { return std_msgs::msg::Empty(); });
}

TEST(fkie_message_filters, LifecyclePublisherMessage)
{
    std::size_t received_msgs = 0;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node =
        std::make_shared<rclcpp_lifecycle::LifecycleNode>("lifecycle_publisher_message");
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub = node->create_subscription<std_msgs::msg::Empty>(
        "publisher_test", 1, [&received_msgs](const std_msgs::msg::Empty::ConstSharedPtr&) { ++received_msgs; });
    mf::UserSource<std_msgs::msg::Empty> src;
    mf::Publisher<std_msgs::msg::Empty, mf::RosMessage> pub;
    node->configure();
    node->activate();
    common_publisher_test_code(node, src, pub, sub, received_msgs,
                               []() -> std_msgs::msg::Empty { return std_msgs::msg::Empty(); });
    node->deactivate();
    node->shutdown();
}

TEST(fkie_message_filters, PublisherMessageUniquePtr)
{
    std::size_t received_msgs = 0;
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("publisher_message_unique_ptr");
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub = node->create_subscription<std_msgs::msg::Empty>(
        "publisher_test", 1, [&received_msgs](const std_msgs::msg::Empty::ConstSharedPtr&) { ++received_msgs; });
    mf::UserSource<std_msgs::msg::Empty::UniquePtr> src;
    mf::Publisher<std_msgs::msg::Empty, mf::RosMessageUniquePtr> pub;
    common_publisher_test_code(node, src, pub, sub, received_msgs, []() -> std_msgs::msg::Empty::UniquePtr
                               { return std::make_unique<std_msgs::msg::Empty>(); });
}

TEST(fkie_message_filters, LifecyclePublisherMessageUniquePtr)
{
    std::size_t received_msgs = 0;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node =
        std::make_shared<rclcpp_lifecycle::LifecycleNode>("lifecycle_publisher_message_unique_ptr");
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub = node->create_subscription<std_msgs::msg::Empty>(
        "publisher_test", 1, [&received_msgs](const std_msgs::msg::Empty::ConstSharedPtr&) { ++received_msgs; });
    mf::UserSource<std_msgs::msg::Empty::UniquePtr> src;
    mf::Publisher<std_msgs::msg::Empty, mf::RosMessageUniquePtr> pub;
    node->configure();
    node->activate();
    common_publisher_test_code(node, src, pub, sub, received_msgs, []() -> std_msgs::msg::Empty::UniquePtr
                               { return std::make_unique<std_msgs::msg::Empty>(); });
    node->deactivate();
    node->shutdown();
}

TEST(fkie_message_filters, PublisherMessageSharedPtr)
{
    std::size_t received_msgs = 0;
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("publisher_message_shared_ptr");
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub = node->create_subscription<std_msgs::msg::Empty>(
        "publisher_test", 1, [&received_msgs](const std_msgs::msg::Empty::ConstSharedPtr&) { ++received_msgs; });
    mf::UserSource<std_msgs::msg::Empty::ConstSharedPtr> src;
    mf::Publisher<std_msgs::msg::Empty, mf::RosMessageSharedPtr> pub;
    common_publisher_test_code(node, src, pub, sub, received_msgs, []() -> std_msgs::msg::Empty::ConstSharedPtr
                               { return std::make_shared<std_msgs::msg::Empty>(); });
}

TEST(fkie_message_filters, LifecyclePublisherMessageSharedPtr)
{
    std::size_t received_msgs = 0;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node =
        std::make_shared<rclcpp_lifecycle::LifecycleNode>("lifecycle_publisher_message_shared_ptr");
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub = node->create_subscription<std_msgs::msg::Empty>(
        "publisher_test", 1, [&received_msgs](const std_msgs::msg::Empty::ConstSharedPtr&) { ++received_msgs; });
    mf::UserSource<std_msgs::msg::Empty::ConstSharedPtr> src;
    mf::Publisher<std_msgs::msg::Empty, mf::RosMessageSharedPtr> pub;
    node->configure();
    node->activate();
    common_publisher_test_code(node, src, pub, sub, received_msgs, []() -> std_msgs::msg::Empty::ConstSharedPtr
                               { return std::make_shared<std_msgs::msg::Empty>(); });
    node->deactivate();
    node->shutdown();
}

TEST(fkie_message_filters, ImagePublisherMessage)
{
    std::size_t received_msgs = 0;
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("image_publisher_message");
    image_transport::Subscriber sub = image_transport::create_subscription(
        node.get(), "publisher_test",
        [&received_msgs](const sensor_msgs::msg::Image::ConstSharedPtr&) { ++received_msgs; }, "raw");
    mf::UserSource<sensor_msgs::msg::Image> src;
    mf::ImagePublisher<mf::RosMessage> pub;
    common_publisher_test_code(node, src, pub, sub, received_msgs,
                               []() -> sensor_msgs::msg::Image { return sensor_msgs::msg::Image(); });
}

TEST(fkie_message_filters, ImagePublisherMessageUniquePtr)
{
    std::size_t received_msgs = 0;
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("image_publisher_message_unique_ptr");
    image_transport::Subscriber sub = image_transport::create_subscription(
        node.get(), "publisher_test",
        [&received_msgs](const sensor_msgs::msg::Image::ConstSharedPtr&) { ++received_msgs; }, "raw");
    mf::UserSource<sensor_msgs::msg::Image::UniquePtr> src;
    mf::ImagePublisher<mf::RosMessageUniquePtr> pub;
    common_publisher_test_code(node, src, pub, sub, received_msgs, []() -> sensor_msgs::msg::Image::UniquePtr
                               { return std::make_unique<sensor_msgs::msg::Image>(); });
}

TEST(fkie_message_filters, ImagePublisherMessageSharedPtr)
{
    std::size_t received_msgs = 0;
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("image_publisher_message_shared_ptr");
    image_transport::Subscriber sub = image_transport::create_subscription(
        node.get(), "publisher_test",
        [&received_msgs](const sensor_msgs::msg::Image::ConstSharedPtr&) { ++received_msgs; }, "raw");
    mf::UserSource<sensor_msgs::msg::Image::ConstSharedPtr> src;
    mf::ImagePublisher<mf::RosMessageSharedPtr> pub;
    common_publisher_test_code(node, src, pub, sub, received_msgs, []() -> sensor_msgs::msg::Image::ConstSharedPtr
                               { return std::make_shared<sensor_msgs::msg::Image>(); });
}

template<template<class> class MessageCreator, class Source, class Publisher, class Subscriber>
void camera_publisher_test_code(rclcpp::Node::SharedPtr& node, Source& src, Publisher& pub, Subscriber& sub,
                                std::size_t& received_msgs)
{
    src.connect_to_sink(pub);
    ASSERT_EQ(0, get_publisher_count(sub));
    ASSERT_FALSE(pub.is_active());
    pub.advertise(node, "publisher_test", 1);
    ASSERT_EQ("/publisher_test", pub.topic());
    process_pending_events(node);
    ASSERT_TRUE(pub.is_active());
    ASSERT_EQ(1, get_publisher_count(sub));
    ASSERT_EQ(0, received_msgs);
    auto img = MessageCreator<sensor_msgs::msg::Image>::create();
    auto info = MessageCreator<sensor_msgs::msg::CameraInfo>::create();
    src(mf::helpers::maybe_move(img), mf::helpers::maybe_move(info));
    process_pending_events(node);
    ASSERT_EQ(1, received_msgs);
}

TEST(fkie_message_filters, CameraPublisherMessage)
{
    std::size_t received_msgs = 0;
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("camera_publisher_message");
    image_transport::CameraSubscriber sub = image_transport::create_camera_subscription(
        node.get(), "publisher_test",
        [&received_msgs](const sensor_msgs::msg::Image::ConstSharedPtr&,
                         const sensor_msgs::msg::CameraInfo::ConstSharedPtr&) { ++received_msgs; },
        "raw");
    mf::UserSource<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> src;
    mf::CameraPublisher<mf::RosMessage> pub;
    camera_publisher_test_code<mf::RosMessage>(node, src, pub, sub, received_msgs);
}

TEST(fkie_message_filters, CameraPublisherMessageUniquePtr)
{
    std::size_t received_msgs = 0;
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("camera_publisher_message_unique_ptr");
    image_transport::CameraSubscriber sub = image_transport::create_camera_subscription(
        node.get(), "publisher_test",
        [&received_msgs](const sensor_msgs::msg::Image::ConstSharedPtr&,
                         const sensor_msgs::msg::CameraInfo::ConstSharedPtr&) { ++received_msgs; },
        "raw");
    mf::UserSource<sensor_msgs::msg::Image::UniquePtr, sensor_msgs::msg::CameraInfo::UniquePtr> src;
    mf::CameraPublisher<mf::RosMessageUniquePtr> pub;
    camera_publisher_test_code<mf::RosMessageUniquePtr>(node, src, pub, sub, received_msgs);
}

TEST(fkie_message_filters, CameraPublisherMessageSharedPtr)
{
    std::size_t received_msgs = 0;
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("camera_publisher_message_shared_ptr");
    image_transport::CameraSubscriber sub = image_transport::create_camera_subscription(
        node.get(), "publisher_test",
        [&received_msgs](const sensor_msgs::msg::Image::ConstSharedPtr&,
                         const sensor_msgs::msg::CameraInfo::ConstSharedPtr&) { ++received_msgs; },
        "raw");
    mf::UserSource<sensor_msgs::msg::Image::ConstSharedPtr, sensor_msgs::msg::CameraInfo::ConstSharedPtr> src;
    mf::CameraPublisher<mf::RosMessageSharedPtr> pub;
    camera_publisher_test_code<mf::RosMessageSharedPtr>(node, src, pub, sub, received_msgs);
}

template<class MessageT, class Node, class Publisher, class Subscriber, class Filter>
void common_subscriber_test_code(Node& node, Publisher& pub, Subscriber& sub, Filter& flt, std::size_t& received_msgs)
{
    sub.connect_to_sink(flt);
    flt.set_processing_function(
        [&received_msgs](const auto&) -> bool
        {
            ++received_msgs;
            return true;
        });
    ASSERT_EQ(0, get_subscription_count(pub));
    sub.subscribe(node, "subscriber_test", 1);
    ASSERT_EQ("/subscriber_test", sub.topic());
    process_pending_events(node);
    ASSERT_EQ(1, get_subscription_count(pub));
    ASSERT_EQ(0, received_msgs);
    publish(pub, MessageT());
    process_pending_events(node);
    ASSERT_EQ(1, received_msgs);
}

TEST(fkie_message_filters, SubscriberMessage)
{
    std::size_t received_msgs = 0;
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("subscriber_message");
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub =
        node->create_publisher<std_msgs::msg::Empty>("subscriber_test", 1);
    mf::Subscriber<std_msgs::msg::Empty, mf::RosMessage> sub;
    mf::SimpleUserFilter<std_msgs::msg::Empty> flt;
    common_subscriber_test_code<std_msgs::msg::Empty>(node, pub, sub, flt, received_msgs);
}

TEST(fkie_message_filters, LifecycleSubscriberMessage)
{
    std::size_t received_msgs = 0;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node =
        std::make_shared<rclcpp_lifecycle::LifecycleNode>("lifecycle_subscriber_message");
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Empty>::SharedPtr pub =
        node->create_publisher<std_msgs::msg::Empty>("subscriber_test", 1);
    mf::Subscriber<std_msgs::msg::Empty, mf::RosMessage> sub;
    mf::SimpleUserFilter<std_msgs::msg::Empty> flt;
    node->configure();
    node->activate();
    common_subscriber_test_code<std_msgs::msg::Empty>(node, pub, sub, flt, received_msgs);
    node->deactivate();
    node->shutdown();
}

TEST(fkie_message_filters, SubscriberMessageUniquePtr)
{
    std::size_t received_msgs = 0;
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("subscriber_message_unique_ptr");
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub =
        node->create_publisher<std_msgs::msg::Empty>("subscriber_test", 1);
    mf::Subscriber<std_msgs::msg::Empty, mf::RosMessageUniquePtr> sub;
    mf::SimpleUserFilter<std_msgs::msg::Empty::UniquePtr> flt;
    common_subscriber_test_code<std_msgs::msg::Empty>(node, pub, sub, flt, received_msgs);
}

TEST(fkie_message_filters, LifecycleSubscriberMessageUniquePtr)
{
    std::size_t received_msgs = 0;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node =
        std::make_shared<rclcpp_lifecycle::LifecycleNode>("lifecycle_subscriber_message_unique_ptr");
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Empty>::SharedPtr pub =
        node->create_publisher<std_msgs::msg::Empty>("subscriber_test", 1);
    mf::Subscriber<std_msgs::msg::Empty, mf::RosMessageUniquePtr> sub;
    mf::SimpleUserFilter<std_msgs::msg::Empty::UniquePtr> flt;
    node->configure();
    node->activate();
    common_subscriber_test_code<std_msgs::msg::Empty>(node, pub, sub, flt, received_msgs);
    node->deactivate();
    node->shutdown();
}

TEST(fkie_message_filters, SubscriberMessageSharedPtr)
{
    std::size_t received_msgs = 0;
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("subscriber_message_shared_ptr");
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub =
        node->create_publisher<std_msgs::msg::Empty>("subscriber_test", 1);
    mf::Subscriber<std_msgs::msg::Empty, mf::RosMessageSharedPtr> sub;
    mf::SimpleUserFilter<std_msgs::msg::Empty::ConstSharedPtr> flt;
    common_subscriber_test_code<std_msgs::msg::Empty>(node, pub, sub, flt, received_msgs);
}

TEST(fkie_message_filters, LifecycleSubscriberMessageSharedPtr)
{
    std::size_t received_msgs = 0;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node =
        std::make_shared<rclcpp_lifecycle::LifecycleNode>("lifecycle_subscriber_message_shared_ptr");
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Empty>::SharedPtr pub =
        node->create_publisher<std_msgs::msg::Empty>("subscriber_test", 1);
    mf::Subscriber<std_msgs::msg::Empty, mf::RosMessageSharedPtr> sub;
    mf::SimpleUserFilter<std_msgs::msg::Empty::ConstSharedPtr> flt;
    node->configure();
    node->activate();
    common_subscriber_test_code<std_msgs::msg::Empty>(node, pub, sub, flt, received_msgs);
    node->deactivate();
    node->shutdown();
}

TEST(fkie_message_filters, ImageSubscriberMessage)
{
    std::size_t received_msgs = 0;
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("image_subscriber_message");
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub =
        node->create_publisher<sensor_msgs::msg::Image>("subscriber_test", 1);
    mf::ImageSubscriber<mf::RosMessage> sub;
    mf::SimpleUserFilter<sensor_msgs::msg::Image> flt;
    common_subscriber_test_code<sensor_msgs::msg::Image>(node, pub, sub, flt, received_msgs);
}

TEST(fkie_message_filters, ImageSubscriberMessageSharedPtr)
{
    std::size_t received_msgs = 0;
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("image_subscriber_message_shared_ptr");
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub =
        node->create_publisher<sensor_msgs::msg::Image>("subscriber_test", 1);
    mf::ImageSubscriber<mf::RosMessageSharedPtr> sub;
    mf::SimpleUserFilter<sensor_msgs::msg::Image::ConstSharedPtr> flt;
    common_subscriber_test_code<sensor_msgs::msg::Image>(node, pub, sub, flt, received_msgs);
}

template<class Publisher, class Subscriber, class Filter>
void camera_subscriber_test_code(rclcpp::Node::SharedPtr& node, Publisher& pub, Subscriber& sub, Filter& flt,
                                 std::size_t& received_msgs)
{
    sub.connect_to_sink(flt);
    flt.set_processing_function(
        [&received_msgs](const auto&, const auto&) -> bool
        {
            ++received_msgs;
            return true;
        });
    ASSERT_EQ(0, get_subscription_count(pub));
    sub.subscribe(node, "subscriber_test", 1);
    ASSERT_EQ("/subscriber_test", sub.topic());
    process_pending_events(node);
    ASSERT_EQ(1, get_subscription_count(pub));
    ASSERT_EQ(0, received_msgs);
    publish(pub, sensor_msgs::msg::Image(), sensor_msgs::msg::CameraInfo());
    process_pending_events(node);
    ASSERT_EQ(1, received_msgs);
}

TEST(fkie_message_filters, CameraSubscriberMessage)
{
    std::size_t received_msgs = 0;
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("camera_subscriber_message");
    image_transport::CameraPublisher pub = image_transport::create_camera_publisher(node.get(), "subscriber_test");
    mf::CameraSubscriber<mf::RosMessage> sub;
    mf::SimpleUserFilter<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> flt;
    camera_subscriber_test_code(node, pub, sub, flt, received_msgs);
}

TEST(fkie_message_filters, CameraSubscriberMessageSharedPtr)
{
    std::size_t received_msgs = 0;
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("camera_subscriber_message_shared_ptr");
    image_transport::CameraPublisher pub = image_transport::create_camera_publisher(node.get(), "subscriber_test");
    mf::CameraSubscriber<mf::RosMessageSharedPtr> sub;
    mf::SimpleUserFilter<sensor_msgs::msg::Image::ConstSharedPtr, sensor_msgs::msg::CameraInfo::ConstSharedPtr> flt;
    camera_subscriber_test_code(node, pub, sub, flt, received_msgs);
}
