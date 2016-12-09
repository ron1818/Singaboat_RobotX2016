```bash
roscore
rosbag play xxx.bag tf:=fakeft tfstatic:=faketfstatic
roslaunch robotx_control bag_r_l_control.launch
rosrun rviz rviz -d `rospack find robotx_rviz`/rviz/bag.rviz
roslaunch robotx_sensor gps_osm_viewer.launch
# open a browser that target to: file:///home/xxx/catkin_ws/src/ROS-OSM-map-integration/index.html
```
