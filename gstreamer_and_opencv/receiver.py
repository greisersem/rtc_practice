import cv2 as cv

pipeline = (
    "udpsrc port=5000 buffer-size=65536 ! " \
    "application/x-rtp,clock-rate=90000,payload=96 ! " \
    "rtph264depay ! " \
    "h264parse ! " \
    "decodebin ! " \
    "videoconvert ! " \
    "video/x-raw,format=BGR ! " \
    "appsink drop=true emit-signals=true" \
)
    
cap = cv.VideoCapture(pipeline, cv.CAP_GSTREAMER)

while True:
    ret, frame = cap.read()
        
    cv.imshow('Video Stream', frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break
