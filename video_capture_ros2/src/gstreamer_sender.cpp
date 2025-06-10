#include <memory>
#include <cstdlib>
#include <signal.h>
#include <sys/wait.h>
#include "rclcpp/rclcpp.hpp"

class VideoSender : public rclcpp::Node {
private:
    std::string pipeline;
    std::thread video_thread;

    void streaming()
    {
        execl("/bin/bash", "bash", "-c", pipeline.c_str(), nullptr);
    }

public:
    VideoSender(const std::string& receiver_ip) : Node("video_sender")
    {
        pipeline = "gst-launch-1.0 v4l2src device=/dev/video0 ! "
                   "videoconvert ! x264enc tune=zerolatency key-int-max=15 ! "
                   "video/x-h264,profile=main ! rtph264pay pt=96 config-interval=-1 ! "
                   "udpsink host=" + receiver_ip + " port=5000";
    
        video_thread = std::thread([this]() { this->streaming(); });
    }
   
    ~VideoSender() 
    {
        if (video_thread.joinable()) {
            video_thread.join();
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoSender>("127.0.0.1"));
    rclcpp::shutdown();
    return 0;
}
