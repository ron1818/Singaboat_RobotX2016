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
### launch gazebo camera and camshift with auto color detection ###
```bash
# launch gazebo with test 1
roslaunch robotx_gazebo robotx_test.launch
roslaunch robotx_vision camshift_color.launch namespace:=/bow_stereo/left input_rgb_image:=image_raw color_under_detect:=red
# track left camera on red
roslaunch robotx_vision camshift_color.launch namespace:=/bow_stereo/right input_rgb_image:=image_raw color_under_detect:=green
# track right camera on green
```

### feature matching ###
must first install opencv 2.4.13.1 from source (github),
follow opencv website for more info


