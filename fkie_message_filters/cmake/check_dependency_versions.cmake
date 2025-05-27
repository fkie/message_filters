# fkie_message_filters
# Copyright © 2018-2025 Fraunhofer FKIE
# Author: Timo Röhling
# SPDX-License-Identifier: Apache-2.0
math(
    EXPR
    computed_image_transport_version
    "65536 * ${image_transport_VERSION_MAJOR} + 256 * ${image_transport_VERSION_MINOR} + ${image_transport_VERSION_PATCH}"
)
math(
    EXPR
    computed_rclcpp_version
    "65536 * ${rclcpp_VERSION_MAJOR} + 256 * ${rclcpp_VERSION_MINOR} + ${rclcpp_VERSION_PATCH}"
)
set_property(
    TARGET fkie_message_filters::fkie_message_filters
    APPEND
    PROPERTY
        INTERFACE_COMPILE_DEFINITIONS
        "FKIE_MF_IMAGE_TRANSPORT_VERSION=${computed_image_transport_version}"
        "FKIE_MF_RCLCPP_VERSION=${computed_rclcpp_version}"
)
