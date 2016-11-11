COMMANDS
--------
### changelog ###
####@2016-10-31####
+ change from `isreal` to `isgazebo` to differentiate gazebo simulated boat from real boat

### launch fake wamv and movebase ###
```bash
# lauch robot state description
roslaunch robotx_bringup fake_wamv.launch
# launch movebase
roslaunch robotx_nav move_base_map.launch isgazebo:=false mapname:=pandan
# launch rviz
rosrun rviz rviz -d `rospack find robotx_rviz`/rviz/fake_move_base.rviz
# run your behavior file, makesure *.py is executable< chmod +x *.py
rosrun robotx_nav move_base_loiter.py _radius:=5 _polygon:=6 _ccw:=true
rosrun robotx_nav move_base_forward.py 
rosrun robotx_nav move_base_scout.py
rosrun robotx_nav move_base_zigzag.py
# OR
roslaunch robotx_nav loiter_behavior.launch isgazebo:=false
roslaunch robotx_nav constant_heading_behavior.launch isgazebo:=false
roslaunch robotx_nav scout_behavior.launch isgazebo:=false
roslaunch robotx_nav zigzag_behavior.launch isgazebo:=false
roslaunch robotx_nav reverse_behavior.launch isgazebo:=false
roslaunch robotx_nav rotation_behavior.launch isgazebo:=false
roslaunch robotx_nav station_keep_behavior.launch isgazebo:=false
```

### launch gazebo and movebase ###
the gps coordinates is coded in: `robotx_description\urdf\sensor.gazebo`,
please find the line 106-107 to change the reference latitude/longitude.

for toplevel behavior, you must first have `testx_toplevel.py` in `robotx_nav/nodes` folder,
and `testx_behavior.launch` in `robotx_nav/launch` folder.
notice that the launch file calls the python file.

at anytime, you can use `rosrun rqt_graph rqt_graph` to check the message flow
and `rosrun rqt_tf_tree rqt_tf_tree` to check the tf publication.

for this release, the *tf* is taken care by `robot_state_publisher` and `ekf` from `robot_localization` (`odom->base_link`) package (gps and imu fusion only). the `map->odom` is a static transformation.

```bash
# lauch gazebo in empty environment:
roslaunch robotx_gazebo robotx_test.launch
# if you want to lauch gazebo with pre-built test environment:
roslaunch robotx_gazebo robotx_test.launch test1:=true
# launch movebase, you can change the mapname by: hawaii or blank_map
roslaunch robotx_nav move_base_map.launch isgazebo:=true mapname:=pandan
# launch rviz
rosrun rviz rviz -d `rospack find robotx_rviz`/rviz/move_base.rviz
# launch individual behavior, you can check one by one
roslaunch robotx_nav loiter_behavior.launch isgazebo:=true
roslaunch robotx_nav constant_heading_behavior.launch isgazebo:=true
roslaunch robotx_nav scout_behavior.launch isgazebo:=true
roslaunch robotx_nav zigzag_behavior.launch isgazebo:=true
roslaunch robotx_nav reverse_behavior.launch isgazebo:=true
roslaunch robotx_nav rotation_behavior.launch isgazebo:=true
roslaunch robotx_nav station_keep_behavior.launch isgazebo:=true
# launch toplevel behavior
roslaunch robotx_nav test1_behavior.launch isgazebo:=true
```

### about mapserver ###
in future, the map server will be replaced by another node from the vision group.
please think about how to use that to publish a `/map` topic.

### about control ###
here the `\cmd_vel` is published by `/move_base` only. and in the `robotx_control/launch` folder,
there is a `yocs` related mux-controller so that we can use joystick and keyboard to interrupt the move_base command. However, it will not allowed in the actual competition (human interruption).


### nanyang lake test @ 2016-11-12 ###
+ make sure gps and imu are working:
 - gps should output `navsat/fix` (`sensor_msgs/NavSatFix`) and `navsat/vel`
 - imu should output `middle_middle_imu/imu/data_raw` (`sensor_msgs/Imu`) and `middle_middle_imu/imu/mag` (`sensor_msgs/MagneticField`)
+ make sure `navsat_transform` can convert `navsat/fix` to `odometry/gps`
+ make sure `imu_filter` can output `middle_middle_imu/imu/data`

for the test, we mainly want to check `constant_heading_hebavior` and `gps_waypoint` can work or not.

you can use gazebo to test the code as mentioned above.
notice that way can set mapname to nanyang lake now.
```bash
roslaunch robotx_nav move_base_map.launch isgazebo:=true mapname:=pandan
```

#### migrate from gazebo to real boat ####
TODO

#### detailed tasks ####
1. move boat by RC controller
2. move boat by keyboard
3. move boat to a constant heading, say (10, 1.57, 0) 
4. move boat around a marker, say (5, 5, 0)
5. move boat to a specific gps point, say (1.345122, 103.684729)
6. check the three color led
7. check hydrophone topic
8. check station keeping

