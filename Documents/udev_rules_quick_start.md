## final output ##
IN YELLOW COMPUTER
        imu: /dev/USBimu
        arduino: /dev/USBard
        gps: /dev/ttyS5 (port COM6)

# UDEV RULES CHANGE (example for arduino) #
## Check the necesary information ##
1. Find information of arduino: plug in arduino, in terminal type the following
```bash
udevadm info -a -n /dev/ttyUSB0 | grep '{idVendor}' | head -n1
# ATTRS{idVendor}=="0403"
udevadm info -a -n /dev/ttyUSB0 | grep '{idProduct}' | head -n1
# ATTRS{idProduct}=="6001"
udevadm info -a -n /dev/ttyUSB0 | grep '{serial}' | head -n1
# ATTRS{serial}=="A6008isP"
```
 
2. Change the udev rule:
``` bash
cd /ect/udev/rules.d
sudo gedit 99-usb-serial.rules
# using the information above, write:
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="A6008isP", SYMLINK+="USBard"
# change the arduino usb port name to /dev/USBard
```

3. Test: unplug arduino and plug it again, type in terminal
``` bash
ls -l /dev/USBard
# lrwxrwxrwx 1 root root 7 Nov 25 22:12 /dev/USBard -> ttyUSB0
ls -l /dev/ttyUSB0
# crw-rw---- 1 root uucp 188, 0 Nov 25 22:12 /dev/ttyUSB0
sudo chmod a+rw /dev/ttyUSB0
# /if the results resemble the above, all is good, now the arduino usb port name is /dev/USBard
```
