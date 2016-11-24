## use osm to view gps ##
```bash
cd ~/catkin_ws/src
git clone https://github.com/sylvainar/ROS-OSM-map-integration
sudo apt-get install ros-indigo-rosbridge-suite
cd ..
catkin_make
```

The configuration of the module can be modified just by changing the value at the top of the script.js file.

`CONFIG_default_gps_topic_name` : Set the name of the GPS topic the app is going to listen to, e.g. `/navsat/fix`
`CONFIG_cycles_number` : If the GPS is publishing really really fast, the app doesn't have the time to update the marker at each cycle, causing some delay. This parameters sets the number of cycles between each actualisation.
`CONFIG_tile_source` : Set the source of the tiles for the map. If you downloaded the maps of the area you want to move in, then you can set it to local. Else, set it to server.
`CONFIG_tile_local_path` : Path to the downloaded tiles
`CONFIG_ROS_server_URI` : Route to ROS server. It could be localhost or an IP.

`roslaunch rosbridge_server rosbridge_websocket.launch` or `roslaunch robotx_sensor gps_osm_viewer.launch` followed by 
use browser to launch `index.html` in that folder
