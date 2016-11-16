## make the intel nuc wake on lan ##
### on the nuc ###
Install ethtool

`sudo apt-get install ethtool`

Edit `/etc/rc.local` to add this line before the `exit 0` line:

`ethtool -s eth0 wol g`

Find out the MAC address of your network card (replace eth if with your interface name, eth0, eth1, ..):

`ifconfig eth | grep "HWaddr" | awk '{print $5}'`

### on the base station ###

`sudo apt-get install wakeonlan`

Shutdown the machine. You should be able to wake it up using:

`wakeonlan your_mac`

for intel nuc: f4:4d:30:63:08:aa 
