#include <memory>
#include <cstdlib>
#include <csignal>
#include <sys/wait.h>
#include "rclcpp/rclcpp.hpp"

class VideoSender : public rclcpp::Node
{
public:
  VideoSender(const std::string& receiver_ip) : Node("video_sender")
  {
    // Формируем команду
    pipeline_ = "gst-launch-1.0 v4l2src device=/dev/video0 ! "
                "videoconvert ! x264enc tune=zerolatency key-int-max=15 ! "
                "video/x-h264,profile=main ! rtph264pay pt=96 config-interval=-1 ! "
                "udpsink host=" + receiver_ip + " port=5000";
    
    // Запускаем GStreamer в отдельном процессе
    child_pid_ = fork();
    if (child_pid_ == 0) {
      // Дочерний процесс
      execl("/bin/sh", "sh", "-c", pipeline_.c_str(), nullptr);
      exit(EXIT_FAILURE); // Сюда попадем только при ошибке
    }
    else if (child_pid_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Fork failed!");
    }
    else {
      RCLCPP_INFO(this->get_logger(), "GStreamer started with PID: %d", child_pid_);
    }
  }

  ~VideoSender() {
    if (child_pid_ > 0) {
      // Корректно завершаем GStreamer
      kill(child_pid_, SIGTERM);
      waitpid(child_pid_, nullptr, 0);
      RCLCPP_INFO(this->get_logger(), "GStreamer terminated");
    }
  }

private:
  std::string pipeline_;
  pid_t child_pid_ = -1;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VideoSender>("127.0.0.1"); // Используйте реальный IP получателя
  rclcpp::spin(node); // Исправлено: spin вместо spin_node_once
  rclcpp::shutdown();
  return 0;
}