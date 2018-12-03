/****************************************************************************
 *
 * fkie_message_filters
 * Copyright © 2018 Fraunhofer FKIE
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

#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_TF_FILTER_IMPL_H_
#define INCLUDE_FKIE_MESSAGE_FILTERS_TF_FILTER_IMPL_H_

#include "tf_filter.h"
#include "helpers/access_ros_header.h"
#include "helpers/tuple.h"
#include <mutex>

namespace fkie_message_filters
{

template<class... Inputs>
struct TfFilter<Inputs...>::Impl
{
    static constexpr tf2::TransformableRequestHandle NeverTransformable = 0xffffffffffffffffull;
    static constexpr tf2::TransformableRequestHandle TransformAvailable = 0;

    struct MessageInfo
    {
        MessageInfo(const MessageTuple& m) : message(m) {}

        MessageTuple message;
        std::set<tf2::TransformableRequestHandle> handles;
    };

    Impl(TfFilter<Inputs...>* parent, tf2::BufferCore& bc) noexcept
    : bc_(bc), parent_(parent), cur_queue_size_(0), max_queue_size_(0), tf_handle_(0), cbq_(nullptr) {}

    ~Impl()
    {
        std::lock_guard<std::mutex> lock{mutex_};
        if (tf_handle_ != 0)
            bc_.removeTransformableCallback(tf_handle_);
    }

    tf2::TransformableRequestHandle make_transformable_request(std::shared_ptr<MessageInfo>& info, const std::string& target_frame, const std::string& source_frame, const ros::Time& time) noexcept
    {
        tf2::TransformableRequestHandle h = bc_.addTransformableRequest(tf_handle_, target_frame, source_frame, time);
        if (h != NeverTransformable && h != TransformAvailable)
        {
            info->handles.insert(h);
            requests_.emplace(h, info);
        }
        return h;
    }

    void cancel_all_transformable_requests (std::shared_ptr<MessageInfo>& info) noexcept
    {
        for (const tf2::TransformableRequestHandle& h : info->handles)
        {
            bc_.cancelTransformableRequest(h);
            requests_.erase(h);
        }
        info->handles.clear();
    }

    tf2::BufferCore& bc_;
    TfFilter<Inputs...>* parent_;
    std::mutex mutex_;
    FilterFailureCB failure_cb_;
    ros::V_string target_frames_;
    uint32_t cur_queue_size_, max_queue_size_;
    tf2::TransformableCallbackHandle tf_handle_;
    ros::CallbackQueueInterface* cbq_;
    std::map<tf2::TransformableRequestHandle, std::shared_ptr<MessageInfo>> requests_;
    std::list<std::shared_ptr<MessageInfo>> messages_;
};

template<class... Inputs>
class TfFilter<Inputs...>::RosCB : public ros::CallbackInterface
{
public:
    RosCB(const std::weak_ptr<Impl>& parent, const MessageTuple& msg, TfFilterResult result) noexcept
    : parent_(parent), msg_(msg), result_(result) {}

    CallResult call() noexcept override
    {
        std::shared_ptr<Impl> impl = parent_.lock();
        if (impl)
        {
            if (result_ == TfFilterResult::TransformAvailable)
                helpers::index_apply<sizeof...(Inputs)>
                (
                    [this, &impl](auto... Is)
                    {
                        impl->parent_->send(std::get<Is>(this->msg_)...);
                    }
                );
            else
            {
                std::unique_lock<std::mutex> lock{impl->mutex_};
                FilterFailureCB cb = impl->failure_cb_;
                if (cb)
                {
                    lock.unlock();
                    helpers::index_apply<sizeof...(Inputs)>
                    (
                        [this, &impl, &cb](auto... Is)
                        {
                            cb(std::get<Is>(this->msg_)..., this->result_);
                        }
                    );
                }
            }
        }
        return Success;
    }
private:
    std::weak_ptr<Impl> parent_;
    MessageTuple msg_;
    TfFilterResult result_;
};

namespace helpers
{

std::string strip_slash(const std::string& s) noexcept
{
    std::string::size_type n = s.find_first_not_of('/');
    if (n == std::string::npos) return std::string();
    return s.substr(n);
}

}

template<class... Inputs>
TfFilter<Inputs...>::TfFilter(tf2::BufferCore& bc, const std::string& target_frame, uint32_t queue_size, const ros::NodeHandle& nh) noexcept
{
    init(bc, queue_size, nh);
    set_target_frame(target_frame);
}

template<class... Inputs>
TfFilter<Inputs...>::TfFilter(tf2::BufferCore& bc, const std::string& target_frame, uint32_t queue_size, ros::CallbackQueueInterface* cbq) noexcept
{
    init(bc, queue_size, cbq);
    set_target_frame(target_frame);
}

template<class... Inputs>
TfFilter<Inputs...>::TfFilter(tf2::BufferCore& bc, const ros::V_string& target_frames, uint32_t queue_size, const ros::NodeHandle& nh) noexcept
{
    init(bc, queue_size, nh);
    set_target_frames(target_frames);
}

template<class... Inputs>
TfFilter<Inputs...>::TfFilter(tf2::BufferCore& bc, const ros::V_string& target_frames, uint32_t queue_size, ros::CallbackQueueInterface* cbq) noexcept
{
    init(bc, queue_size, cbq);
    set_target_frames(target_frames);
}

template<class... Inputs>
void TfFilter<Inputs...>::init(tf2::BufferCore& bc, uint32_t queue_size, ros::CallbackQueueInterface* cbq) noexcept
{
    impl_ = std::make_shared<Impl>(this, bc);
    impl_->max_queue_size_ = queue_size;
    impl_->cbq_ = cbq;
    impl_->tf_handle_ = impl_->bc_.addTransformableCallback(
        [this](tf2::TransformableRequestHandle request_handle, const std::string& target_frame, const std::string& source_frame, const ros::Time& time, tf2::TransformableResult result)
        {
            this->transformable(request_handle, target_frame, source_frame, time, result);
        }
    );
}

template<class... Inputs>
void TfFilter<Inputs...>::init(tf2::BufferCore& bc, uint32_t queue_size, const ros::NodeHandle& nh) noexcept
{
    init(bc, queue_size, nh.getCallbackQueue());
}

template<class... Inputs>
void TfFilter<Inputs...>::set_target_frames(const ros::V_string& target_frames)
{
    if (!impl_) throw std::logic_error("TfFilter object is not initialized");
    std::unique_lock<std::mutex> lock{impl_->mutex_};
    impl_->target_frames_.clear();
    for (const std::string& f : target_frames)
    {
        std::string f2 = helpers::strip_slash(f);
        if (!f2.empty()) impl_->target_frames_.push_back(f2);
    }
}

template<class... Inputs>
void TfFilter<Inputs...>::set_target_frame(const std::string& target_frame)
{
    set_target_frames({target_frame});
}

template<class... Inputs>
void TfFilter<Inputs...>::reset() noexcept
{
    if (!impl_) return; // Nothing to do if object is not initialized
    std::lock_guard<std::mutex> lock{impl_->mutex_};
    for (auto& info : impl_->messages_)
    {
        impl_->cancel_all_transformable_requests(info);
    }
    impl_->messages_.clear();
    impl_->cur_queue_size_ = 0;
    assert(impl_->requests_.empty());
}

template<class... Inputs>
void TfFilter<Inputs...>::set_filter_failure_function(FilterFailureCB f)
{
    if (!impl_) throw std::logic_error("TfFilter object is not initialized");
    std::lock_guard<std::mutex> lock{impl_->mutex_};
    impl_->failure_cb_ = f;
}

template<class... Inputs>
void TfFilter<Inputs...>::receive (const Inputs&... in)
{
    using MessageInfo = typename Impl::MessageInfo;
    if (!impl_) return;
    std::unique_lock<std::mutex> lock{impl_->mutex_};
    if (impl_->target_frames_.empty()) return;
    std::shared_ptr<MessageInfo> info = std::make_shared<MessageInfo>(std::forward_as_tuple(in...));
    std::string source_frame = helpers::strip_slash(helpers::access_ros_header_frame_id(std::get<0>(info->message)));
    if (source_frame.empty())
    {
        report_failure(lock, info->message, TfFilterResult::EmptyFrameID);
        return;
    }
    ros::Time stamp = helpers::access_ros_header_stamp(std::get<0>(info->message));
    ros::V_string target_frames = impl_->target_frames_;
    for (const std::string& target_frame : target_frames)
    {
        tf2::TransformableRequestHandle h = impl_->make_transformable_request(info, target_frame, source_frame, stamp);
        if (h == Impl::NeverTransformable)
        {
            impl_->cancel_all_transformable_requests(info);
            report_failure(lock, info->message, TfFilterResult::TransformExpired);
            return;
        }
    }
    if (!info->handles.empty())
    {
        impl_->messages_.push_back(info);
        ++impl_->cur_queue_size_;
        while (impl_->cur_queue_size_ > impl_->max_queue_size_)
        {
            std::shared_ptr<MessageInfo> old = impl_->messages_.front();
            impl_->messages_.pop_front();
            --impl_->cur_queue_size_;
            impl_->cancel_all_transformable_requests(old);
            report_failure(lock, old->message, TfFilterResult::QueueOverflow);
        }
    }
    else
    {
        send_message(lock, info->message);
    }
}

template<class... Inputs>
void TfFilter<Inputs...>::transformable(tf2::TransformableRequestHandle request_handle, const std::string& target_frame, const std::string& source_frame, ros::Time time, tf2::TransformableResult result)
{
    using MessageInfo = typename Impl::MessageInfo;
    std::unique_lock<std::mutex> lock{impl_->mutex_};
    if (!impl_) return;
    auto req = impl_->requests_.find(request_handle);
    if (req == impl_->requests_.end()) return;
    std::shared_ptr<MessageInfo> info = req->second;
    impl_->requests_.erase(req);
    if (!info) return; /* just in case */
    info->handles.erase(request_handle);
    if (info->handles.empty() || result != tf2::TransformAvailable)
    {
        impl_->cancel_all_transformable_requests(info);
        auto msg = std::find(impl_->messages_.begin(), impl_->messages_.end(), info);
        if (msg == impl_->messages_.end()) return;
        impl_->messages_.erase(msg);
        --impl_->cur_queue_size_;
        if (result == tf2::TransformAvailable) /* Everything succeeded */
        {
            std::string source_frame = helpers::strip_slash(helpers::access_ros_header_frame_id(std::get<0>(info->message)));
            ros::Time stamp = helpers::access_ros_header_stamp(std::get<0>(info->message));
            for (const std::string& frame : impl_->target_frames_)
            {
                if (!impl_->bc_.canTransform(frame, source_frame, stamp))
                {
                    report_failure(lock, info->message, TfFilterResult::UnknownFailure);
                    return;
                }
            }
            send_message(lock, info->message);
        }
        else
        {
            report_failure(lock, info->message, TfFilterResult::TransformFailed);
        }
    }
}

template<class... Inputs>
void TfFilter<Inputs...>::report_failure(std::unique_lock<std::mutex>& lock, const MessageTuple& msg, TfFilterResult reason)
{
    if (!impl_) return;
    if (impl_->cbq_)
    {
        impl_->cbq_->addCallback(boost::make_shared<RosCB>(impl_, msg, reason));
    }
    else
    {
        FilterFailureCB cb = impl_->failure_cb_;
        if (cb)
        {
            auto unlock = helpers::with_scoped_unlock(lock);
            helpers::index_apply<sizeof...(Inputs)>
            (
                [&cb, &msg, &reason](auto... Is)
                {
                    cb(std::get<Is>(msg)..., reason);
                }
            );
        }
    }
}

template<class... Inputs>
void TfFilter<Inputs...>::send_message(std::unique_lock<std::mutex>& lock, const MessageTuple& msg)
{
    if (!impl_) return;
    if (impl_->cbq_)
    {
        impl_->cbq_->addCallback(boost::make_shared<RosCB>(impl_, msg, TfFilterResult::TransformAvailable));
    }
    else
    {
        auto unlock = helpers::with_scoped_unlock(lock);
        helpers::index_apply<sizeof...(Inputs)>
        (
            [this, &msg](auto... Is)
            {
                this->send(std::get<Is>(msg)...);
            }
        );
    }
}

} // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_TF_FILTER_IMPL_H_ */
