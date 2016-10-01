INSTALL OPENCV
--------------
```bash
cd
git clone https://github.com/opencv/opencv
cd opencv
git checkout 2.4
mkdir release
cd release
sudo cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_GTK=ON -D WITH_OPENGL=ON ..
make
sudo make install
```

COMMANDS
--------

### Camshift ###
#### launch gazebo camera and camshift ####
```bash
# launch gazebo with test 1
roslaunch robotx_gazebo robotx_test.launch
roslaunch robotx_vision camshift.launch namespace:=/bow_stereo/left input_rgb_image:=image_raw
# track left camera on red
roslaunch robotx_vision camshift.launch namespace:=/bow_stereo/right input_rgb_image:=image_raw
# track right camera on green
```
#### launch gazebo camera and camshift with auto color detection ####
```bash
# launch gazebo with test 1
roslaunch robotx_gazebo robotx_test.launch
roslaunch robotx_vision camshift_color.launch namespace:=/bow_stereo/left input_rgb_image:=image_raw color_under_detect:=red
# track left camera on red
roslaunch robotx_vision camshift_color.launch namespace:=/bow_stereo/right input_rgb_image:=image_raw color_under_detect:=green
# track right camera on green
```

#### logitech camera and camshift ####
```bash
# launch webcam
roslaunch robotx_vision usb_cam.launch
# do rectification
ROS_NAMESPACE=/camera/rgb rosrun image_proc image_proc
# launch camshift
roslaunch robotx_vision camshift.launch namespace:=/camera/rgb input_rgb_image:=image_rect_color
# or 
roslaunch robotx_vision camshift_color.launch namespace:=/camera/rgb input_rgb_image:=image_rect_color color_under_detect:=red
```

### feature matching ###
