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
#
cmake_minimum_required(VERSION 3.8...3.28)
project(fkie_message_filters VERSION 2.0.0)

find_package(ament_cmake REQUIRED)
find_package(Boost QUIET)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_program(DOXYGEN doxygen)

###########
# Targets #
###########

add_library(${PROJECT_NAME}
    src/publisher_subscriber_base.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${sensor_msgs_INCLUDE_DIR}
    ${std_msgs_INCLUDE_DIR}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
    image_transport::image_transport
    tf2_ros::tf2_ros
    rclcpp::rclcpp
)
add_executable(tf2_bug test/tf2_bug.cpp)
target_link_libraries(tf2_bug PUBLIC tf2_ros::tf2_ros)

add_executable(hello_example example/hello.cpp)
target_link_libraries(hello_example PRIVATE ${PROJECT_NAME})

configure_file(
    src/version.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/include/version.h
    @ONLY
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_cmake_ros REQUIRED)
    find_package(std_msgs REQUIRED)
    ament_add_ros_isolated_gtest(filtertests
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
    target_link_libraries(filtertests ${PROJECT_NAME})
    if(TARGET Boost::boost)
        target_link_libraries(filtertests Boost::boost)
    endif()
endif()

if(DOXYGEN)
    add_custom_target(doc ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/doc/html/index.html)
    add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/doc/html/index.html
        COMMAND ${DOXYGEN}
        COMMENT Generating Doxygen documentation
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/doc
    )
endif()

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/include/version.h
    DESTINATION include/${PROJECT_NAME})

##########################
# Ament package handling #
##########################
ament_export_targets(${PROJECT_NAME})
ament_package()
