#include <memory>
#include <cstdlib>
#include <signal.h>
#include <sys/wait.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include "rclcpp/rclcpp.hpp"


class VideoReceiver : public rclcpp::Node {
public:
    VideoReceiver() : Node("video_receiver")
    {
        pipeline = "udpsrc port=5000 buffer-size=65536 ! " 
                   "application/x-rtp,clock-rate=90000,payload=96 ! " 
                   "rtph264depay ! " 
                   "h264parse ! " 
                   "decodebin ! " 
                   "videoconvert ! " 
                   "video/x-raw,format=BGR ! " 
                   "appsink drop=true emit-signals=true";
    
        pid_t pid = fork();
        
        if (pid == 0) {
            auto cap = cv::VideoCapture(pipeline, cv::CAP_GSTREAMER);

            while (true) {
                cv::Mat frame;
                cap >> frame;
                    
                cv::imshow("Stream", frame);
                if (cv::waitKey(1) == 27) {
                    break;
                }
            }

            cap.release();
            cv::destroyAllWindows();
            exit(1);
        } else if (pid > 0) {
            child_pid = pid;
        } else {
            std::cout << "Fork failed" << std::endl;
        }
    }

    ~VideoReceiver() 
    {
        if (child_pid > 0) {
            kill(child_pid, SIGTERM);
            waitpid(child_pid, nullptr, 0);
        }
    }

private:
    std::string pipeline;
    pid_t child_pid;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoReceiver>());
    rclcpp::shutdown();
    return 0;
}