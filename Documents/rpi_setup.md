Setup raspberry Pi or equivalent
================================
####applied to raspbian or ubuntu 14.04lts####

RTC
----
require *i2c-tools* to be installed

1. load RTC module: `sudo modprobe rtc-ds1307`. 
2. run in root: `sudo bash`. 
3. run: `echo ds1307 0x68 > /sys/class/i2c-adapter/i2c-1/new_device`. 
4. exit by `exit`. 
5. verify with `sudo hwclock -w` and `sudo hwclock -r`. 
6. edit */etc/modules* and add `rtc-ds1307` at the end. 
7. edit */etc/rc.local* and add:
    ```
    echo ds1307 0x68 > /sys/class/i2c-adapter/i2c-1/new_device \n\r
    sudo hwclock -s
    ```.
8. change executable to */etc/rc.local* by `sudo chmod +x /etc/rc.local`.

arduino permissions
-------------------
`sudo adduser <usrname> dialout`

### arduino desktop ###
create `arduino.desktop` on the Desktop

```
[Desktop Entry]
Name=Arduino
Comment=Launcher for Arduino
Exec=/home/erian-hpzbook/arduino-1.7.10-linux64/arduino
Icon=/home/erian-hpzbook/arduino-1.7.10-linux64/reference/arduino.cc/img/logo_46.png
Categories=Application;Development;IDE;
Version=1.7.10
Type=Application
Terminal=true
```

make it executable by `chmod +x`

ssh server and client
---------------------
`sudo apt-get install openssh-server openssh-client`

I2C add user
-----------
`sudo adduser <username> i2c`

Wpa for NTUSECURE
-----------------
for raspian only, ubuntu for arm can be set at desktop

edit: */etc/wpa_supplicant/wpa_supplicant.conf*

```
network={
    ssid="NTUSECURE"
    scan_ssid=1
    key_mgmt=WPA-EAP IEEE8021X
    group=CCMP TKIP
    eap=PEAP TLS
    identity="<name>"
    password="<password>"
    phase2="auth=MSCHAPV2"
}
```

SSH login without password
----

###Your aim###

You want to use Linux and OpenSSH to automate your tasks. Therefore you need an automatic login from host A / user a to Host B / user b. You don't want to enter any passwords, because you want to call ssh from a within a shell script.

###How to do it###

First log in on A as user a and generate a pair of authentication keys. Do not enter a passphrase:

```bash
a@A:~> ssh-keygen -t rsa
Generating public/private rsa key pair.
Enter file in which to save the key (/home/a/.ssh/id_rsa): 
Created directory '/home/a/.ssh'.
Enter passphrase (empty for no passphrase): 
Enter same passphrase again: 
Your identification has been saved in /home/a/.ssh/id_rsa.
Your public key has been saved in /home/a/.ssh/id_rsa.pub.
The key fingerprint is:
3e:4f:05:79:3a:9f:96:7c:3b:ad:e9:58:37:bc:37:e4 a@A
Now use ssh to create a directory ~/.ssh as user b on B. (The directory may already exist, which is fine):
```
```bash
a@A:~> ssh b@B mkdir -p .ssh
b@B's password: 
```
Finally append a's new public key to b@B:.ssh/authorized_keys and enter b's password one last time:

```bash
a@A:~> cat .ssh/id_rsa.pub | ssh b@B 'cat >> .ssh/authorized_keys'
b@B's password:
```
From now on you can log into B as b from A as a without password:

```bash
a@A:~> ssh b@B
```

A note from one of our readers: Depending on your version of SSH you might also have to do the following changes:

Put the public key in `.ssh/authorized_keys2`
Change the permissions of `.ssh to 700`
Change the permissions of `.ssh/authorized_keys2 to 640`

libegl1-mesa-dev problem
------------------------
I solved this by doing the following:

I used : `sudo apt-get download <package>`
to download the packages (the two listed after you ran `apt-get install -f`).

Then I used: `dpkg -i --force-overwrite <package_name.deb>`
to install the packages with forcing overwrites of files from other packages.

For *libegl1-mesa-dev*, it installed, but it failed to configure because it was missing other dependencies. Don't worry, we fix this in a second...

I then ran dpkg with force overwrite for the second package (*libgles2...*) the same way. It installed, but did not configure because libegl1 did not finish configuring...

now run: `sudo apt-get install -f`
this will download all dependencies, and configure everything.

ubuntu sudo without password
----------------------------

###Add the user to sudo'ers###
- this enables sudo to be used without entering a password (may be necessary for some scripts for Raspbian to run):
`sudo visudo`

Place this line at the END of the file - replace 'user' with username:
`user ALL=(ALL) NOPASSWD: ALL`

Save and exit (CTRL+O then CTRL+X).

RPI with ubuntu14.04LTS
=======================
Usage
-----

###ROOT RESIZE###

There are no Raspbian-specific utilities included, specifically no automatic root resizer. However, it's not hard to do manually. Once booted:

`sudo fdisk /dev/mmcblk0`

Delete the second partition (d, 2), then re-create it using the defaults (n, p, 2, enter, enter), then write and exit (w). Reboot the system, then:

`sudo resize2fs /dev/mmcblk0p2`

###SWAP###

There is no swap partition/file included. If you want swap, it's recommended you do:

`sudo apt-get install dphys-swapfile`

You should have a (resized) SD card at least 4GB, because by default it will want to create a ~2GB swapfile.

###WIFI FIRMWARE###

If you are using a wifi dongle, you will likely need to get the linux-firmware package:

`sudo apt-get install linux-firmware`

###SSH SERVER###


If you would like to install an SSH server for remote access:

`sudo apt-get install openssh-server`

###WPA SUPPLICANT###

`sudo apt-get install wpasupplicant`

Using either DHCP or a static config (doesn't matter which)--AND assuming your wifi worked during install--make your /etc/network/interfaces look something like below (for wlan0 should match the name of your wifi card listed under ifconfig -a e.g. your detected wifi card could be nicknamed eth1 by the OS for all I know.):

``` bash
 auto lo iface lo inet loopback     
 auto wlan0 iface wlan0 inet dhcp    
 wpa-conf /etc/wpa_supplicant/wpa_supplicant.conf
 ```
To configure wpa_supplicant use the command (Referenced in the config above)

`wpa_passphrase "YOUR_SSID" SSID_PASSWORD | sudo tee /etc/wpa_supplicant/wpa_supplicant.conf`

Next, create a new executable script named iwconfig (you can name this script anything really, "iwconfig-default-ssid", perhaps?--I just made it short for the example):

```
sudo touch /etc/network/if-up.d/iwconfig && sudo chmod 700
/etc/network/if-up.d/iwconfig && sudo ln -s
/etc/network/if-up.d/iwconfig /etc/network/if-pre-up.d/iwconfig
```
Now edit /etc/network/if-up.d/iwconfig and add the SSID you want Ubuntu Server to connect to on startup:

```
#!/bin/sh
iwconfig wlan0 essid "YOUR_DEFAULT_SSID" mode managed
```

Now bring ifdown (if you haven't already), then ifup, and you should be golden now and when you reboot (as long as you're near your SSID.)

###SERIAL CONSOLE###

To enable the serial console, change the */boot/cmdline.txt* as follows:

`dwc_otg.lpm_enable=0 console=ttyAMA0,115200 kgdboc=ttyAMA0,115200 console=tty1 root=/dev/mmcblk0p2 rootwait`

and add a new file */etc/init/ttyAMA0.conf*:
```
start on stopped rc or RUNLEVEL=[12345]
stop on runlevel [!12345]

respawn
exec /sbin/getty -L 115200 ttyAMA0 vt102
```

Disable login keyring password
-------------------------------
applied for ubuntu-14.04 for arm

`python -c "import gnomekeyring;gnomekeyring.change_password_sync('login', 'MYPASSWORD', '');"`

Automatic login in commandline
------------------------------
To enable automatic login in Ubuntu server we need to edit the tty1 configuration file. To open this file up in nano use the following command:

`sudo nano /etc/init/tty1.conf`
The very last line in this file should start with an exec command. Delete this line and replace it with the following:

`exec /bin/login -f USERNAME < /dev/tty1 > /dev/tty1 2>&1`
Note that you need to change USERNAME to the username of the user that you want to be automatically logged in. Once you reboot your system you should be automatically logged in.

References
----------
http://askubuntu.com/questions/867/how-can-i-stop-being-prompted-to-unlock-the-default-keyring-on-boot

http://www.linuxproblem.org/art_9.html

https://www.raspberrypi.org/forums/viewtopic.php?f=56&t=100553&start=200

https://www.raspberrypi.org/forums/viewtopic.php?f=56&t=112253

http://askubuntu.com/questions/464507/ubuntu-14-04-server-wifi-wpa2-personal
