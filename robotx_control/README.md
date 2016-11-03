##ChangeLog##
####@2016-10-31####
+ use `robot-localization` to publish direct `map` -> `base_link` tf
+ configure `~smooth_lagged_data` and `history_length`
####@2016-10-30####
+ use `robot-pose-ekf` to fuse GPS and IMU data, but map origin is offset very far
+ not so many drift though
####@2016-10-29####
+ use `imu-filter-magdwick` to filter IMU data
+  

##COMMAND##
####launch gazebo test scenario####
```bash
# launch gazebo sim with r_l and imu_filter
roslaunch robotx_gazebo robotx_sim.launch
# launch rviz
rosrun rviz rviz -d `rospack find robotx_rviz`/rviz/move_base.rviz
```

