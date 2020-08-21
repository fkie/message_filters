############################################################################
#
# fkie_message_filters
# Copyright © 2018-2020 Fraunhofer FKIE
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
cmake_minimum_required(VERSION 3.1)
project(fkie_message_filters)

set(CMAKE_CXX_STANDARD 14)

###############################################
# Find catkin macros, packages, and libraries #
###############################################

find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  sensor_msgs
  tf2_ros
)

#######################
# System dependencies #
#######################

find_program(DOXYGEN doxygen)

############################
# Configure catkin package #
############################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    image_transport
    roscpp
    sensor_msgs
    tf2_ros
)

#########
# Build #
#########

include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
)
include_directories(include)

add_library(${PROJECT_NAME}
  src/camera_publisher.cpp
  src/camera_subscriber.cpp
  src/image_publisher.cpp
  src/image_subscriber.cpp
  src/publisher_subscriber_base.cpp
)
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(filtertests
    test/test_buffer.cpp
    test/test_combiner_approximate_time.cpp
    test/test_combiner_exact_time.cpp
    test/test_combiner_fifo.cpp
    test/test_divider.cpp
    test/test_filter_loop.cpp
    test/test_helpers.cpp
    test/test_main.cpp
    test/test_selector.cpp
    test/test_sequencer.cpp
    test/test_simple_user_filter.cpp
    test/test_tf_filter.cpp
    test/test_user_filter.cpp
  )
  target_link_libraries(filtertests ${PROJECT_NAME})
  add_rostest_gtest(publish_subscribe_tests rostest/publish_subscribe.launch rostest/test_publish_subscribe.cpp)
  target_link_libraries(publish_subscribe_tests ${PROJECT_NAME})
endif()

if(DOXYGEN)
    add_custom_target(doc ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/doc/html/index.html)
    add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/doc/html/index.html
        COMMAND ${DOXYGEN}
        COMMENT Generating Doxygen documentation
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/doc
    )
endif()

###########
# Install #
###########

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
