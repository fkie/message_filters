############################################################################
#
# fkie_message_filters
# Copyright © 2018-2025 Fraunhofer FKIE
# Author: Timo Röhling
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
############################################################################

cmake_minimum_required(VERSION 3.13...3.28)
project(fkie_message_filters VERSION 3.0.0)
set(FKIE_MF_ABI_VERSION 2)

# Options
option(BUILD_SHARED_LIBS "Build shared library" ON)
option(FKIE_MF_TEST_SANITIZE "Enable sanitizer plugins for tests" OFF)
option(FKIE_MF_TEST_COVERAGE "Enable test coverage" ON)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost QUIET)
find_package(image_transport 3.0 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_program(DOXYGEN doxygen)

# Targets
add_library(${PROJECT_NAME} src/logging.cpp src/publisher_subscriber_base.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(
    ${PROJECT_NAME}
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
           $<INSTALL_INTERFACE:include>
)
target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC sensor_msgs::sensor_msgs_library image_transport::image_transport
           tf2_ros::tf2_ros rclcpp::rclcpp
)
set_target_properties(
    ${PROJECT_NAME} PROPERTIES SOVERSION ${FKIE_MF_ABI_VERSION}
)

add_executable(hello_example example/hello.cpp)
target_link_libraries(hello_example PRIVATE ${PROJECT_NAME})

math(
    EXPR
    computed_fkie_mf_version
    "65536 * ${PROJECT_VERSION_MAJOR} + 256 * ${PROJECT_VERSION_MINOR} + ${PROJECT_VERSION_PATCH}"
)
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

configure_file(
    src/version.hpp.in ${CMAKE_CURRENT_BINARY_DIR}/include/version.hpp @ONLY
)

# Testing
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_cmake_ros REQUIRED)
    find_package(std_msgs REQUIRED)
    ament_add_ros_isolated_gtest(
        filtertests
        test/test_buffer.cpp
        test/test_combiner_approximate_time.cpp
        test/test_combiner_exact_time.cpp
        test/test_combiner_fifo.cpp
        test/test_divider.cpp
        test/test_filter_loop.cpp
        test/test_helpers.cpp
        test/test_main.cpp
        test/test_publish_subscribe.cpp
        test/test_selector.cpp
        test/test_sequencer.cpp
        test/test_simple_user_filter.cpp
        test/test_tf_filter.cpp
        test/test_user_filter.cpp
    )
    target_include_directories(filtertests PRIVATE ${std_msgs_INCLUDE_DIRS})
    target_compile_definitions(
        filtertests PRIVATE FKIE_MF_RCLCPP_VERSION=${computed_rclcpp_version}
    )
    if(FKIE_MF_TEST_SANITIZE)
        target_compile_options(
            filtertests PRIVATE -Og -g -fsanitize=address,undefined
        )
        target_link_options(
            filtertests PRIVATE -Og -g -fsanitize=address,undefined
        )
    endif()
    if(FKIE_MF_TEST_COVERAGE)
        find_program(GCOV gcov)
        if(GCOV)
            set(CTEST_COVERAGE_COMMAND ${GCOV})
            target_compile_options(filtertests PRIVATE --coverage)
            target_link_options(filtertests PRIVATE --coverage)
        else()
            message(FATAL_ERROR "Cannot enable test coverage: gcov not found")
        endif()
    endif()
    target_link_libraries(filtertests ${PROJECT_NAME})
    if(TARGET Boost::boost)
        target_link_libraries(filtertests Boost::boost)
    endif()
endif()

# Documentation
if(DOXYGEN)
    add_custom_target(
        doc ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/doc/html/index.html
    )
    add_custom_command(
        OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/doc/html/index.html
        COMMAND ${DOXYGEN}
        COMMENT Generating Doxygen documentation
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/doc
    )
endif()

# Installation
install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/include/version.hpp
        DESTINATION include/${PROJECT_NAME}
)

# Ament package handling
ament_export_dependencies(image_transport rclcpp sensor_msgs tf2_ros)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package(CONFIG_EXTRAS_POST cmake/check_dependency_versions.cmake)
