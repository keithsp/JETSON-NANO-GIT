gst-launch-1.0 -e -v ^
v4l2src device=/dev/video0 io-mode=2 do-timestamp=true ! ^
video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! ^
queue max-size-buffers=1 leaky=downstream ! ^
videoconvert ! nvvidconv ! ^
"video/x-raw(memory:NVMM),format=NV12,width=640,height=480,framerate=30/1" ! ^
nvv4l2h264enc bitrate=2000000 insert-sps-pps=true iframeinterval=30 idrinterval=30 preset-level=1 ! ^
h264parse config-interval=-1 ! ^
udpsink host=192.168.137.1 port=5000 sync=false async=false
