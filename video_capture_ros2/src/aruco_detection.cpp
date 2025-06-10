#include <memory>
#include <cstdlib>
#include <signal.h>
#include <sys/wait.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>

#include "rclcpp/rclcpp.hpp"


class ArUcoDetector : public rclcpp::Node {
private:
    std::string pipeline;
    std::thread video_thread;

    void detecting()
    {
        auto cap = cv::VideoCapture(pipeline, cv::CAP_GSTREAMER);

        cv::Ptr<cv::aruco::Dictionary> aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();


        while (true) {
            cv::Mat frame;
            
            cap >> frame;

            std::vector<int> marker_ids;
            std::vector<std::vector<cv::Point2f>> marker_corners;
            
            cv::aruco::detectMarkers(frame, aruco_dict, marker_corners, marker_ids, parameters);
            cv::aruco::drawDetectedMarkers(frame, marker_corners, marker_ids);
                
            cv::imshow("Stream", frame);
            if (cv::waitKey(1) == 27) {
                break;
            }

            cap.release();
        }
    }

public:
    ArUcoDetector() : Node("aruco_detector")
    {
        pipeline = "udpsrc port=5000 buffer-size=65536 ! " 
                   "application/x-rtp,clock-rate=90000,payload=96 ! " 
                   "rtph264depay ! " 
                   "h264parse ! " 
                   "decodebin ! " 
                   "videoconvert ! " 
                   "video/x-raw,format=BGR ! " 
                   "appsink drop=true emit-signals=true";
        
        video_thread = std::thread(&ArUcoDetector::detecting, this);
    }

    ~ArUcoDetector() 
    {
        if (video_thread.joinable()) {
            video_thread.join();
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArUcoDetector>());
    rclcpp::shutdown();
    return 0;
}