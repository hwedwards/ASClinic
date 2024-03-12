DEVICE=/dev/video0
echo "Now adjusting the camera settings for $DEVICE"
v4l2-ctl --device=$DEVICE --set-ctrl=focus_auto=0
v4l2-ctl --device=$DEVICE --set-ctrl=focus_absolute=0
v4l2-ctl --device=$DEVICE --set-fmt-video=width=1920,height=1080
echo " "
echo "Camera settings were adjusted to:"
v4l2-ctl --device=$DEVICE --get-ctrl=focus_auto,focus_absolute
v4l2-ctl --device=$DEVICE --get-fmt-video
