/****************************************************************************
 *
 * fkie_message_filters
 * Copyright © 2018-2025 Fraunhofer FKIE
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

#include <fkie_message_filters/publisher_base.hpp>
#include <fkie_message_filters/subscriber_base.hpp>
#include <rclcpp/event.hpp>
#include <rclcpp/node.hpp>

#include <set>

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

#ifndef DOXYGEN
class PublisherBase::Monitor
{
public:
    ~Monitor() {}

    void detach(PublisherBase* publisher)
    {
        std::lock_guard<std::mutex> lock{mutex_};
        publishers_.erase(publisher);
        if (publishers_.empty())
        {
            shutdown_flag_.store(true);
            node_->get_node_graph_interface()->notify_graph_change();
        }
    }

    void attach(PublisherBase* publisher)
    {
        std::lock_guard<std::mutex> lock{mutex_};
        publishers_.insert(publisher);
    }

    static std::shared_ptr<Monitor> instance(const rclcpp::Node::SharedPtr& node)
    {
        std::lock_guard<std::mutex> lock{singleton_mutex_};
        if (!singleton_)
            singleton_ = std::shared_ptr<Monitor>(new Monitor(node->create_sub_node("_graph_monitor")));
        return singleton_;
    }

private:
    explicit Monitor(const rclcpp::Node::SharedPtr& node)
        : node_(node), graph_event_(node_->get_graph_event()), shutdown_flag_(false),
          thread_(std::bind(&Monitor::run, this))
    {
        thread_.detach();
    }

    void run()
    {
        while (!shutdown_flag_.load())
        {
            using namespace std::chrono_literals;
            node_->wait_for_graph_change(graph_event_, 1s);
            if (!shutdown_flag_.load() && graph_event_->check_and_clear())
            {
                std::lock_guard<std::mutex> lock{mutex_};
                for (PublisherBase* publisher : publishers_)
                {
                    publisher->update_subscriber_state();
                }
            }
        }
        singleton_.reset();
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Event::SharedPtr graph_event_;
    std::mutex mutex_;
    std::set<PublisherBase*> publishers_;
    std::atomic_bool shutdown_flag_;
    std::thread thread_;

    static std::mutex singleton_mutex_;
    static std::shared_ptr<Monitor> singleton_;
};
#endif

std::mutex PublisherBase::Monitor::singleton_mutex_;
std::shared_ptr<PublisherBase::Monitor> PublisherBase::Monitor::singleton_;

PublisherBase::~PublisherBase()
{
    shutdown_monitor();
}

std::tuple<Connection, Connection> PublisherBase::link_with_subscriber(SubscriberBase& sub)
{
    Connection c1 = enable_signal_.connect([&sub]() { sub.subscribe_impl(); });
    Connection c2 = disable_signal_.connect([&sub]() { sub.unsubscribe_impl(); });
    if (is_active())
        sub.subscribe_impl();
    else
        sub.unsubscribe_impl();
    return std::tie(c1, c2);
}

void PublisherBase::update_subscriber_state()
{
    if (is_active())
        enable_signal_();
    else
        disable_signal_();
}

void PublisherBase::start_monitor(const rclcpp::Node::SharedPtr& node) noexcept
{
    monitor_ = Monitor::instance(node);
    monitor_->attach(this);
}

void PublisherBase::shutdown_monitor() noexcept
{
    if (monitor_)
    {
        monitor_->detach(this);
        monitor_.reset();
    }
}

SubscriberBase::~SubscriberBase()
{
    unlink_from_publisher();
}

void SubscriberBase::subscribe()
{
    unlink_from_publisher();
    if (is_configured())
        subscribe_impl();
}

void SubscriberBase::unsubscribe()
{
    unlink_from_publisher();
    if (is_configured())
        unsubscribe_impl();
}

void SubscriberBase::subscribe_on_demand(PublisherBase& pub)
{
    unlink_from_publisher();
    if (is_configured())
        link_with_publisher(pub);
}

void SubscriberBase::unlink_from_publisher()
{
    conn1_.disconnect();
    conn2_.disconnect();
}

void SubscriberBase::link_with_publisher(PublisherBase& pub)
{
    unlink_from_publisher();
    std::tie(conn1_, conn2_) = pub.link_with_subscriber(*this);
}

FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters
