import os

os.system(
    "gst-launch-1.0 udpsrc port=5000 !" \
    "application/x-rtp,clock-rate=90000,payload=96 !" \
    "rtpjitterbuffer !" \
    "rtph264depay !" \
    "h264parse !" \
    "avdec_h264 !" \
    "videoconvert !" \
    "xvimagesink")
