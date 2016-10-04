COMMANDS
--------

### Create a gazebo urdf model ###
1. install google sketchup in Windows.
2. download some skp files from [warehouse](https://3dwarehouse.sketchup.com/?hl=en), in this example, search: boat dock and download [this](https://3dwarehouse.sketchup.com/model.html?id=8c16322267597eaef6f336537d2ffac).
3. open it in sketchup, make necessary modifications, in this example, delete boats, put a wall behind, draw cross, circle and triangle on the wall.
4. move the dock to the origin of the map (Important).
5. scale the dimensions to the actual situation, note that our boat is 5 meter long and 2 meter wide.
6. save it as *dae* file if this *skp* file also has a textile folder, DO NOT delete it.
7. put the *dae* and the folder (same name) into robotx_description/meshes.
8. create urdf file in robotx_description/urdf


### Spawn test scenario ###
```bash
roslaunch gazebo_ros empty_world.launch
rosrun gazebo_ros spawn_model -file `rospack find robotx_description`/urdf/dock.urdf \
-urdf -x 0 -y 0 -z 1 -model dock
```

### Launch with boat ###
```bash
roslaunch robotx_gazebo robotx_test.launch test1:=true
# in total eight tests, to enable, set them to true, e.g. to use test5
roslaunch robotx_gazebo robotx_test.launch test5:=true
# to have a blank map:
roslaunch robotx_gazebo robotx_test.launch
```
