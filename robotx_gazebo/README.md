COMMANDS
--------

### Spawn test scenario ###
```bash
roslaunch gazebo_ros empty_world.launch
rosrun gazebo_ros spawn_model -file `rospack find robotx_description`/urdf/dock.urdf \
-urdf -x 0 -y 0 -z 1 -model dock
```

### Launch with boat ###
```bash
roslaunch robotx_gazebo robotx_test test1:=true
# in total eight tests, to enable, set them to true, e.g. to use test5
roslaunch robotx_gazebo robotx_test test5:=true
# to have a blank map:
roslaunch robotx_gazebo robotx_test
```
