#!/usr/bin/env bash
set -euo pipefail
# D435 remains owned by robot_bridge. MJPEG is only consumed on localhost.
# queue downstream leaking: discard old video, never build an unbounded backlog.
exec gst-launch-1.0 -e \
  souphttpsrc location=http://127.0.0.1:8080/stream.mjpg is-live=true do-timestamp=true \
  ! multipartdemux ! jpegdec ! queue max-size-buffers=2 max-size-bytes=0 max-size-time=0 leaky=downstream \
  ! videoconvert ! videorate ! video/x-raw,format=I420,framerate=25/1 \
  ! x264enc tune=zerolatency speed-preset=ultrafast bitrate=2000 pass=cbr \
    key-int-max=25 bframes=0 rc-lookahead=0 vbv-buf-capacity=100 \
    option-string=nal-hrd=cbr:force-cfr=1 \
  ! video/x-h264,profile=constrained-baseline ! h264parse config-interval=-1 \
  ! rtspclientsink location=rtsp://127.0.0.1:8554/robot protocols=tcp
