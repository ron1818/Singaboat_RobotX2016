## equipments ##
+ linksys E1200 router
+ tplink wa7210n outdoor router

## setup the linksys router ##
+ AP mode with SSID: *RobotxNTU*
+ WPA/WPA2 mixed with password: *robotx2016*
+ router ip address: 192.168.1.1, subnet mask: *255.255.255.0*
+ DHCP enabled, range: *100-149*
+ connect the boat computers and switches with this router
+ assign DHCP reservation to the computers and cameras

## setup the tplink router ##
+ wired connection with static ip address *192.168.0.254*
+ quick setup to bridge mode
+ use *RobotxNTU* as the WAN
+ set an SSID toL *RobotxNTUBase*, same password
+ reassign the tplink router to *192.168.1.254*
+ do NOT use DHCP
+ reboot and check

## test ##
+ laptop 1 connect to *RobotxNTU*
+ laptop 2 connect to *RobotxNTUBase*
+ `ROS_MASTER_URI=http://laptop1.local:11311`
+ run `roscore` on laptop 1
+ check topics on laptop 2
+ reverse the behavior
