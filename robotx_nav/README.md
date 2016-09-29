COMMANDS
--------

### launch fake wamv and movebase ###
```bash
# lauch robot state description
roslaunch robotx_description fake_wamv_description.launch
# launch movebase
roslaunch robotx_nav move_base_map.lauch isreal:=false mapname:=pandan
# launch rviz
rosrun rviz rviz -d `rospack find robotx_rviz`/rviz/fake_move_base.rviz
```

### launch gazebo and movebase ###
```bash
# lauch robot state description
roslaunch robotx_gazebo robotx_test.launch
# launch movebase
roslaunch robotx_nav move_base_map.lauch isreal:=true mapname:=pandan
# launch rviz
rosrun rviz rviz -d `rospack find robotx_rviz`/rviz/move_base.rviz
```
