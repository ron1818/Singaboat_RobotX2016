COMMANDS
--------

### launch gazebo camera and camshift ###
```bash
# launch gazebo with test 1
roslaunch robotx_gazebo robotx_test.launch
roslaunch robotx_vision camshift.launch namespace:=/bow_stereo/left input_rgb_image:=image_raw
# track left camera on red
roslaunch robotx_vision camshift.launch namespace:=/bow_stereo/right input_rgb_image:=image_raw
# track right camera on green
```
