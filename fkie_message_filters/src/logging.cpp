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

#include <fkie_message_filters/logging.hpp>

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

#ifndef DOXYGEN
rclcpp::Logger logger_ = rclcpp::get_logger("fkie_message_filters");
#endif

void set_logger(const std::string& name)
{
    logger_ = rclcpp::get_logger(name);
}

void set_logger(const rclcpp::Node::SharedPtr& node, const std::string& name)
{
    if (name.empty())
        logger_ = node->get_logger();
    else
        logger_ = node->get_logger().get_child(name);
}

const rclcpp::Logger& get_logger() noexcept
{
    return logger_;
}

FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters
