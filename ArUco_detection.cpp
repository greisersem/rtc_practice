#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

int main()
{
    std::string pipeline =  "udpsrc port=5000 buffer-size=65536 ! " 
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
        
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        cv::aruco::DetectorParameters detector_params = cv::aruco::DetectorParameters();
        cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        cv::aruco::ArucoDetector detector(dictionary, detectorParams);

        while (true) {
            cv::Mat frame;
            cap >> frame;
            
            detector.detectMarkers(frame, marker_corners, marker_ids);
            cv::aruco::drawDetectedMarkers(frame, marker_corners, marker_ids);

            cv::imshow("Stream", frame);
            if (cv::waitKey(1) == 27) {
                break;
            }
        }

        cap.release();
        cv::destroyAllWindows();
    }
}