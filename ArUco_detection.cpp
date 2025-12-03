#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

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
 
    auto cap = cv::VideoCapture(pipeline, cv::CAP_GSTREAMER);
    
    if (!cap.isOpened()) {
        std::cerr << "Failed to open video capture!" << std::endl;
        return -1;
    }
    
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

    while (true) {
        cv::Mat frame;
        cap >> frame;
        
        if (frame.empty()) {
            std::cerr << "Empty frame!" << std::endl;
            continue;
        }
        
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        std::vector<std::vector<cv::Point2f>> rejected_candidates;
        
        cv::aruco::detectMarkers(frame, dictionary, marker_corners, marker_ids, parameters, rejected_candidates);
        
        // Отрисовка обнаруженных маркеров
        if (!marker_ids.empty()) {
            cv::aruco::drawDetectedMarkers(frame, marker_corners, marker_ids);
        }
        
        cv::imshow("Stream", frame);
        if (cv::waitKey(1) == 27) { // ESC key
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();
    
    return 0;
}
