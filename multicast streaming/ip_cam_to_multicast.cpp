#include <opencv2/opencv.hpp>
#include <iostream>

int main()
{
    std::string pipeline =
        "rtspsrc location=rtsp://admin:kluz5056@192.168.0.121:554/Streaming/Channels/101 "
        "latency=0 buffer-mode=0 protocols=udp drop-on-latency=true ! "
        "rtph264depay ! "
        "h264parse ! "
        "v4l2h264dec capture-io-mode=dmabuf ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false";

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

    if (!cap.isOpened()) {
        std::cerr << "Failed to open camera stream\n";
        return -1;
    }

    cv::Mat frame;

    while (true) {
        if (!cap.read(frame)) {
            std::cerr << "Failed to grab frame\n";
            break;
        }

        cv::imshow("Camera", frame);

        if (cv::waitKey(1) == 27) // ESC
            break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
