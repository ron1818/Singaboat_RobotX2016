### replay from bag file ###
```bash
# start roscore
roscore
# play back data, ignore tf and tfstatic
rosbag play xxx.bag tf:=fakeft tfstatic:=faketfstatic
# call robot localization
roslaunch robotx_control bag_r_l_control.launch
# view in rviz, specially configured
rosrun rviz rviz -d `rospack find robotx_rviz`/rviz/bag.rviz
# optionally can view gps tracks on a OSM map
roslaunch robotx_sensor gps_osm_viewer.launch
# open a browser that target to: file:///home/xxx/catkin_ws/src/ROS-OSM-map-integration/index.html
```

### bringup the real wamv boat ###
```bash
# check gps and imu
roslaunch robotx_sensor gps_serial.launch
rostopic echo /navsat/fix

roslaunch robotx_sensor imu_razor_pub.launch
roslaunch razor-imu-9dof razor-display.launch
```

```bash
# on the boat computer
roslaunch robotx_bringup wamv_minimal.launch
roslaunch robotx_bringup wamv_move_base.launch mapname:=blank_map

# on the base computer
roslaunch robotx_bringup wamv_teleop.launch
rosrun rviz rviz -d `rospack find robotx_rviz`/rviz/move_base.rviz
# optionally record bag
rosbag record -a
```
