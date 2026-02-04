#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

class Camera {
public:
    Camera(const std::string& rtsp_url, int width=1920, int height=1080, int fps=25)
        : width(width), height(height), fps(fps) 
    {
        std::string pipeline = rtsp_url + " latency=0 drop-on-latency=true ! "
                               "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink";
        cap.open(pipeline, cv::CAP_GSTREAMER);
        if (!cap.isOpened()) {
            throw std::runtime_error("Failed to open RTSP stream");
        }
    }

    bool readFrame(cv::Mat& frame) {
        cap >> frame;
        return !frame.empty();
    }

    int getWidth() const { return width; }
    int getHeight() const { return height; }
    int getFPS() const { return fps; }

private:
    cv::VideoCapture cap;
    int width, height, fps;
};

class MulticastStreamer {
public:
    MulticastStreamer(const std::string& multicast_ip, int port, int width, int height, int fps)
    {
        std::string pipeline = "appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=2000 speed-preset=ultrafast ! "
                               "rtph264pay config-interval=1 pt=96 ! "
                               "udpsink host=" + multicast_ip + " port=" + std::to_string(port) + " auto-multicast=true";
        
        writer.open(pipeline, cv::CAP_GSTREAMER, 0, fps, cv::Size(width, height));
        if (!writer.isOpened()) {
            throw std::runtime_error("Failed to open multicast pipeline");
        }
    }

    void sendFrame(const cv::Mat& frame) {
        writer.write(frame);
    }

private:
    cv::VideoWriter writer;
};

int main() {
    try {
        std::string rtsp_url = "rtspsrc location=rtsp://admin:kluz5056@192.168.0.121:554/Streaming/Channels/101";
        Camera cam(rtsp_url);

        
        MulticastStreamer streamer("239.255.0.1", 5000, cam.getWidth(), cam.getHeight(), cam.getFPS());

        cv::Mat frame;
        while (true) {
            if (!cam.readFrame(frame)) break;

            streamer.sendFrame(frame);

            // Локальный просмотр для отладки
            cv::imshow("Camera Preview", frame);
            if (cv::waitKey(1) == 27) break; // Esc для выхода
        }
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        return -1;
    }

    return 0;
}
