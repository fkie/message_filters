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
#include <rclcpp/time.hpp>
#include <tf2/buffer_core.hpp>

#include <iostream>

void callback(tf2::TransformableRequestHandle request_handle, const std::string& target_frame,
              const std::string& source_frame, const tf2::TimePoint& time, tf2::TransformableResult result)
{
    std::cout << "callback(request_handle=" << request_handle << ", target_frame=\"" << target_frame
              << "\", source_frame=\"" << source_frame << "\", time=" << tf2::displayTimePoint(time)
              << ", result=" << result << ")\n";
}

template<typename Rep, typename Period>
tf2::TransformableRequestHandle add_transformable_request(tf2::BufferCore& bc, const std::string& target_frame,
                                                          const std::string& source_frame,
                                                          const std::chrono::duration<Rep, Period>& time)
{
    tf2::TimePoint tp{time};
    tf2::TransformableRequestHandle h = bc.addTransformableRequest(callback, target_frame, source_frame, tp);
    std::cout << "add_transformable_request(target_frame=\"" << target_frame << "\", source_frame=\"" << source_frame
              << "\", time=" << tf2::displayTimePoint(tp) << ") -> " << h << "\n";
    return h;
}

template<typename Rep, typename Period>
void set_transform(tf2::BufferCore& bc, const std::string& target_frame, const std::string& source_frame,
                   const std::chrono::duration<Rep, Period>& time)
{
    std::chrono::nanoseconds ns = std::chrono::duration_cast<std::chrono::nanoseconds>(time);
    geometry_msgs::msg::TransformStamped tr;
    tr.header.frame_id = target_frame;
    tr.header.stamp = rclcpp::Time(ns.count());
    tr.child_frame_id = source_frame;
    tr.transform.rotation.w = 1;  // make transform valid
    std::cout << "set_transform(target_farme=\"" << target_frame << "\", source_frame=\"" << source_frame
              << "\", time=" << tf2::displayTimePoint(tf2::TimePoint(time)) << ")\n";
    bc.setTransform(tr, "test_injector", false);
}

void cancel_transformable_request(tf2::BufferCore& bc, tf2::TransformableRequestHandle request_handle)
{
    std::cout << "cancel_transformable_request(request_handle=" << request_handle << ")\n";
    bc.cancelTransformableRequest(request_handle);
}

int main()
{
    using namespace std::chrono_literals;
    tf2::BufferCore bc{10s};
    tf2::TransformableRequestHandle handle = add_transformable_request(bc, "target", "alpha", 100s);
    add_transformable_request(bc, "target", "beta", 100s);
    add_transformable_request(bc, "target", "gamma", 100s);
    cancel_transformable_request(bc, handle);
    set_transform(bc, "target", "alpha", 100s);
    set_transform(bc, "target", "beta", 100s);
    set_transform(bc, "target", "gamma", 100s);
    return 0;
}
