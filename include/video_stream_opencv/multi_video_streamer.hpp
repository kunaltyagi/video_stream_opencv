#ifndef _VIDEO_STREAM_OPENCV_MULTI_VIDEO_STREAMER_HPP_
#define _VIDEO_STREAM_OPENCV_MULTI_VIDEO_STREAMER_HPP_

#include <algorithm>
#include <array>
#include <cstddef>
#include <memory>
#include <type_traits>

#include <boost/smart_ptr.hpp>

#include <ros/ros.h>

#include <video_stream_opencv/video_streamer.hpp>

namespace video_stream_opencv {
template <class T, typename = std::enable_if_t<std::is_arithmetic<T>::value, std::true_type>>
std::string to_string(T val) {
    return std::to_string(val);
}

namespace detail {
template <class T, T... Ns>
auto make_array(const T& begin, std::integer_sequence<T, Ns...>) noexcept
    -> std::array<T, sizeof...(Ns)> {
    return {{(Ns + begin) ...}};
}

template <class T, T N>
std::array<T, N> make_array(const T& begin) noexcept {
    return make_array<T>(begin, std::make_integer_sequence<T, N>{});
}
/*
typename <class T>
auto make_cameras(const T& nh_names, const T& stream_paths) {
    ;
}
*/
}  // namespace detail

template <std::size_t N, class T, class U = unsigned int>
std::array<T, N> make_numerical_suffix_array(const T& prefix, U begin = U(0)) {
    std::array<U, N> suffix_arr = detail::make_array<U, N>(begin);
    std::array<T, N> suffixed_array;
    std::transform(suffix_arr.begin(), suffix_arr.end(), suffixed_array.begin(),
            [&prefix](auto &x) { return prefix + to_string(x); });
    return suffixed_array;
}

template <std::size_t N = 2>
struct MultiVideoStreamer
{
    using VideoStreamerPtr = VideoStreamer*;
    MultiVideoStreamer():
        MultiVideoStreamer(boost::make_shared<ros::NodeHandle>("~"))
    {}
    MultiVideoStreamer(ros::NodeHandlePtr nh_ptr,
            std::string video_stream_provider = ""):
        MultiVideoStreamer(nh_ptr, make_numerical_suffix_array<N>(video_stream_provider))
    {}
    MultiVideoStreamer(ros::NodeHandlePtr nh_ptr,
                       std::array<std::string, N> video_stream_provider):
        nh_ptr_(nh_ptr) {
        auto nh_suffix = make_numerical_suffix_array<N>("~/");

        cameras_ = AllocTraits::allocate(alloc_, N);
        for (std::size_t i = 0; i < N; ++i) {
            AllocTraits::construct(alloc_, cameras_ + i, nh_suffix[i], video_stream_provider[i], true);
        }

        update_srv_ = nh_ptr_->advertiseService("update_param",
                                                &MultiVideoStreamer::update_cb_,
                                                this);
        update_parameters_(video_stream_provider);
    }

    ~MultiVideoStreamer() {
        for (std::size_t i = 0; i < N; ++i) {
            AllocTraits::destroy(alloc_, cameras_ + i);
        }
        AllocTraits::deallocate(alloc_, cameras_, N);
    }

    int run() {
        for (std::size_t i = 0; i < N; ++i) {
            if(!cameras_[i]->isOpened()) {
                ROS_ERROR_STREAM("Could not open the stream " << i);
                return -1;
            }
        }
        /*
        ROS_INFO_STREAM("Opened the cameras, starting to publish.");
        std::thread cap_thread{[&]() { return capture_frames(img_cap_,
                static_cast<unsigned int>(max_error_)); }};

        consume_frames(*nh_ptr_, img_cap_, fps_, camera_name_, frame_id_, camera_info_url_,
                       flip_image_, flip_value_);
        cap_thread.join();
        return 0;
        */
    }

    ;
  private:
    bool update_cb_(std_srvs::Empty::Request&,
                    std_srvs::Empty::Response&) {
        return update_parameters_();
    }

    bool update_parameters_(std::string video_stream_provider="") {
    }

    using Alloc = std::allocator<VideoStreamer>;
    using AllocTraits = std::allocator_traits<Alloc>;

    ros::NodeHandlePtr nh_ptr_;
    ros::ServiceServer update_srv_;

    VideoStreamerPtr cameras_;
    Alloc alloc_;
};  // struct MultiVideoStreamer

}  // namespace video_stream_opencv

#endif  // ifndef _VIDEO_STREAM_OPENCV_MULTI_VIDEO_STREAMER_HPP_
