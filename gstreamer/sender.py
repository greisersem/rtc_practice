import os

os.system(
    "gst-launch-1.0 v4l2src ! " \
    "queue ! " \
    "videoconvert ! " \
    "x264enc tune=zerolatency key-int-max=15 !" \
    "video/x-h264,profile=main !" \
    "rtph264pay pt=96 config-interval=-1 !" \
    "udpsink host=192.168.1.158 port=5000")