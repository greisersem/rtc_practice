#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Cannot open camera\n";
        return -1;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 30);

    std::string pipeline =
        "appsrc ! "
        "videoconvert ! "
        "x264enc tune=zerolatency bitrate=800 speed-preset=ultrafast ! "
        "rtph264pay ! "
        "udpsink host=239.255.0.1 port=5000 auto-multicast=true";

    cv::VideoWriter writer(
        pipeline,
        cv::CAP_GSTREAMER,
        0,
        30,
        cv::Size(640, 480),
        true
    );

    if (!writer.isOpened()) {
        std::cerr << "Failed to open GStreamer pipeline\n";
        return -1;
    }

    cv::Mat frame;

    while (true) {
        cap >> frame;
        if (frame.empty())
            break;

        writer.write(frame);

        if (cv::waitKey(1) == 27)
            break;
    }

    cap.release();
    writer.release();
    return 0;
}
