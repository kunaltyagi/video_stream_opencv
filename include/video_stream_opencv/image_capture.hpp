#ifndef _VIDEO_STREAM_CAPTURE_IMAGE_CAPTURE_HPP_
#define _VIDEO_STREAM_CAPTURE_IMAGE_CAPTURE_HPP_

#include <algorithm>
#include <fstream>
#include <string>

#include <boost/thread/sync_queue.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/videoio/videoio.hpp>

namespace video_stream_opencv {
namespace {
const double _CAPTURE_FPS = 30;
const unsigned int _MAX_QUEUE_SIZE = 100;
const bool _DEFAULT_LOOP = false;
}

struct ImageCapture
{
    const double camera_fps = _CAPTURE_FPS;

    ImageCapture(const std::string &stream_path, double fps,
                 unsigned int queue_size, bool loop_file = _DEFAULT_LOOP):
        camera_fps(fps),
        max_queue_size_(queue_size),
        loop_(loop_file),
        stream_path_(stream_path) {
        video_stream_provider_ = StreamType::UNKNOWN;
        // same size limit used internally in cv::VideoCapture
        if (stream_path.size() < 4) {
            video_stream_provider_ = StreamType::DEV_VIDEO;
        }
        else if (stream_path.rfind("http://", 0) == 0 ||
                 stream_path.rfind("https://", 0) == 0) {
            video_stream_provider_ = StreamType::HTTP;
        }
        else if (stream_path.rfind("rtsp://", 0) == 0) {
            video_stream_provider_ = StreamType::RTSP;
        }
        // one single % is used to identify img_sequence patterns by cv
        else if (std::count(stream_path.begin(), stream_path.end(), '%') == 1) {
            video_stream_provider_ = StreamType::IMG_SEQ;
        }
        else {
            std::ifstream ifs(stream_path);
            // Check if file exists to know if it's a valid file
            if (ifs.good()) {
                // can be a video or image file, both are valid
                video_stream_provider_ = StreamType::MEDIA_FILE;
            }
        }
        open_();
    }

    ImageCapture(const std::string &stream_path, double fps = _CAPTURE_FPS):
        ImageCapture(stream_path, fps, _MAX_QUEUE_SIZE, _DEFAULT_LOOP)
    {}

    ImageCapture(const std::string &stream_path, unsigned int queue_size):
        ImageCapture(stream_path, _CAPTURE_FPS, queue_size, _DEFAULT_LOOP)
    {}

    ImageCapture(const std::string &stream_path, bool loop):
        ImageCapture(stream_path, _CAPTURE_FPS, _MAX_QUEUE_SIZE, loop)
    {}

    ImageCapture(const std::string &stream_path, double fps, bool loop):
        ImageCapture(stream_path, fps, _MAX_QUEUE_SIZE, loop)
    {}

    bool isFile() const {
        return (video_stream_provider_ == StreamType::MEDIA_FILE ||
                video_stream_provider_ == StreamType::IMG_SEQ);
    }

    // wrap get, set, isOpened, grab, retrieve, read and >> for VideoCapture
    double get(int propId) const {
        return cap_.get(propId);
    }

    bool set(int propId, double value) {
        return cap_.set(propId, value);
    }

    bool isOpened() const {
        return cap_.isOpened();
    }

    bool grab() {
        bool ret = cap_.grab();
        if (ret == false && loop_ == true)
        {
            // nothing has changed wrt StreamType
            open_();
            return grab();
        }
        return ret;
    }

    bool retrieve(cv::OutputArray &arr) {
        return cap_.retrieve(arr);
    }

    bool read(cv::OutputArray &arr) {
        if (grab()) {
            return retrieve(arr);
        }
        return false;
    }

    ImageCapture& operator>> (cv::UMat &img) {
        read(img);
        return *this;
    }

    ImageCapture& operator>> (cv::Mat &img) {
        read(img);
        return *this;
    }

    // wrap pull, push for framesQueue_
    bool push(const cv::Mat &img) {
        bool full = false;
        if (framesQueue_.size() >= max_queue_size_) {
            full = true;
            cv::Mat temp;
            framesQueue_.pull(temp);
        }
        framesQueue_.push(img);
        return full;
    }

    void pull(cv::Mat &img) {
        framesQueue_.pull(img);
    }

    bool empty() {
        return framesQueue_.empty();
    }

    private:
        mutable boost::sync_queue<cv::Mat> framesQueue_;
        mutable cv::VideoCapture cap_;

        const unsigned int max_queue_size_ = _MAX_QUEUE_SIZE;
        const bool loop_ = _DEFAULT_LOOP;
        const std::string stream_path_;

        enum class StreamType { UNKNOWN, DEV_VIDEO, HTTP, RTSP, MEDIA_FILE, IMG_SEQ };
        StreamType video_stream_provider_;

        bool open_() {
            if (video_stream_provider_ == StreamType::DEV_VIDEO) {
                return cap_.open(atoi(stream_path_.c_str()));
            }
            return cap_.open(stream_path_);
        }
};
}  // namespace video_stream_opencv

#endif  // ifndef _VIDEO_STREAM_CAPTURE_IMAGE_CAPTURE_HPP_
