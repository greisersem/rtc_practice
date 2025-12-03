#include <iostream>
#include <unistd.h>

std::string receiver_ip = "127.0.0.1";

std::string pipeline = "gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! x264enc tune=zerolatency key-int-max=15 ! video/x-h264,profile=main ! rtph264pay pt=96 config-interval=-1 ! udpsink host=" + receiver_ip + " port=5000";

int main(int argc, char * argv[])
{
  execl("/bin/bash", "bash", "-c", pipeline.c_str(), nullptr);
}