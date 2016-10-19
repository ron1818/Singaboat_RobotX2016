COMMANDS
--------

### launch fake wamv and movebase ###
```bash
# lauch robot state description
roslaunch robotx_bringup fake_wamv.launch
# launch movebase
roslaunch robotx_nav move_base_map.launch isreal:=false mapname:=pandan
# launch rviz
rosrun rviz rviz -d `rospack find robotx_rviz`/rviz/fake_move_base.rviz
# run your behavior file, makesure *.py is executable< chmod +x *.py
rosrun robotx_nav move_base_loitering.py _loitering_radius:=5 _loitering_polygon:=6 _loitering_ccw:=1
rosrun robotx_nav move_base_forward.py _waypoint_distance:=5
# OR
roslaunch robotx_nav loitering_behavior isreal:=false
roslaunch robotx_nav constant_heading_behavior isreal:=false
```

### launch gazebo and movebase ###
```bash
# lauch robot state description
roslaunch robotx_gazebo robotx_test.launch
# launch movebase
roslaunch robotx_nav move_base_map.launch isreal:=true mapname:=pandan
# launch rviz
rosrun rviz rviz -d `rospack find robotx_rviz`/rviz/move_base.rviz
# launch behavior
roslaunch robotx_nav loitering_behavior isreal:=true
roslaunch robotx_nav constant_heading_behavior isreal:=true
```
