#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgproc.hpp>

#include "rclcpp/rclcpp.hpp"


class QRDetector : public rclcpp::Node {
private:
    std::string pipeline;
    std::thread video_thread;

    void detecting()
    {
        auto cap = cv::VideoCapture(pipeline, cv::CAP_GSTREAMER);

        cv::QRCodeDetector qr_det;

        while (true) {
            cv::Mat frame;
            std::vector<std::string> decoded_info;
            cv::Mat points;
            std::vector<cv::Mat> straight_qrcode;

            cap >> frame;

            qr_det.detectAndDecodeMulti(frame, decoded_info, points, straight_qrcode);

            if (points.rows > 0) {
                for (int i = 0; i < points.rows; i++) {
                    cv::Point2f pt0 = points.at<cv::Point2f>(i, 0);
                    cv::Point2f pt1 = points.at<cv::Point2f>(i, 1);
                    cv::Point2f pt2 = points.at<cv::Point2f>(i, 2);
                    cv::Point2f pt3 = points.at<cv::Point2f>(i, 3);
                    
                    cv::line(frame, pt0, pt1, cv::Scalar(0,255,0), 2);
                    cv::line(frame, pt1, pt2, cv::Scalar(0,255,0), 2);
                    cv::line(frame, pt2, pt3, cv::Scalar(0,255,0), 2);
                    cv::line(frame, pt3, pt0, cv::Scalar(0,255,0), 2);
                    
                    if (i < decoded_info.size() && !decoded_info[i].empty()) {
                        cv::putText(frame, decoded_info[i], pt0, 
                                   cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                                   cv::Scalar(0,0,255), 1);
                    }
                }
            }
                
            cv::imshow("Stream", frame);
            if (cv::waitKey(1) == 27) {
                break;
            }
        }
        cap.release();
        cv::destroyAllWindows();
    }


public:
    QRDetector() : Node("qr_detector")
    {
        pipeline = "udpsrc port=5000 buffer-size=65536 ! " 
                   "application/x-rtp,clock-rate=90000,payload=96 ! " 
                   "rtph264depay ! " 
                   "h264parse ! " 
                   "decodebin ! " 
                   "videoconvert ! " 
                   "video/x-raw,format=BGR ! " 
                   "appsink drop=true emit-signals=true";
        
        video_thread = std::thread(&QRDetector::detecting, this);
    }

    ~QRDetector() 
    {
        if (video_thread.joinable()) {
            video_thread.join();
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QRDetector>());
    rclcpp::shutdown();
    return 0;
}