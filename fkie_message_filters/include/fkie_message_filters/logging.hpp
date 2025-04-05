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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_IMAGE_LOGGING_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_IMAGE_LOGGING_HPP_
#pragma once

#include "helpers/abi_namespace.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#define FKIE_MF_INFO(...)  RCLCPP_INFO_STREAM(fkie_message_filters::get_logger(), __VA_ARGS__)
#define FKIE_MF_WARN(...)  RCLCPP_WARN_STREAM(fkie_message_filters::get_logger(), __VA_ARGS__)
#define FKIE_MF_ERROR(...) RCLCPP_ERROR_STREAM(fkie_message_filters::get_logger(), __VA_ARGS__)

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

/** \brief Configure ROS logging.
 *
 * \arg \c name the name that is to be used for logging
 */
void set_logger(const std::string& name);
/** \brief Configure ROS logging.
 *
 * \arg \c node the ROS node instance that serves as parent for the logger
 * \arg \c name if not empty, create a named child logger from \c node instead of using its logger directly.
 */
void set_logger(const rclcpp::Node::SharedPtr& node, const std::string& name = std::string());
/** \brief Return the currently configured ROS logger. */
const rclcpp::Logger& get_logger() noexcept;

FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#endif
