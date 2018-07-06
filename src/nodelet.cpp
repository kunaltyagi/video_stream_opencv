#include <functional>
#include <thread>
#include <tuple>

#include <ros/ros.h>

#include <opencv2/videoio.hpp>

#include <video_stream_opencv/video_stream_opencv.hpp>

using namespace video_stream_opencv;

/**
  * Compulsory parameter:
  *     * video_stream_provider: string, same as input to cv::VideoCapture
  *
  * Optional parameters:
  *     * camera_name: string, defaults to "camera"
  *     * frame_id: string, defaults to camera_name
  *     * camera_info_url: string, used by camera_info_manager to retrieve
  *                        camera settings
  *     * camera_fps: double, defaults to 30
  *     * flip_horizontal: bool, defaults to false, image is flipped if true
  *     * flip_vertical: bool, defaults to false, image is flipped if true
  *     * loop: bool, defaults to true, reopens video_stream_provider if true
  *     * max_error: unsigned int, defaults to 0, max number of continuous
  *                  errors tolerated before quitting. 0 is 
  *     * buffer_queue_size: unsigned int, defaults to 100, used to maintain
  *                          a buffer for streaming data
  *     * fps: double, defaults to 240, throttles the output
  *     * width: int, defaults to 0
  *     * height: int, defaults to 0
  *     If both width and height are more than 0, VideoCapture is used to
  *     modify the image at input itself
  *
  * Set the parameters from command line by
  * $ rosrun video_stream_opencv video_stream _parameter_name:=value
  *
  * For values of type string, ensure that the values aren't numerical.
  * Eg: for video_stream_provider, instead of 0, use /dev/video0
  */

class VideoStreamerNodelet: public VideoStreamer, public ros::nodelet::Nodelet
{
};  // class VideoStreamerNodelet

int main(int argc, char** argv)
{
    ros::init(argc, argv, "video_stream");
    VideoStreamer node;
    return node.run();
}
