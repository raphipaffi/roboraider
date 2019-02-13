# RoboRaider
This is a ROS package for my home-built robot. The package requires
* [Latest LTS release of Xubuntu](https://xubuntu.org/download)
* [Latest LTS release of ROS](http://wiki.ros.org/ROS/Installation)
* [Arduino 1.6.8](https://www.arduino.cc/en/main/software)
* for a working tele-op panel in rviz: modified [ROS Visualization Tutorials](http://wiki.ros.org/visualization_tutorials)

Arduino libraries needed to compile the Arduino code for the low level controller:
* BMSerial [github.com/KurtE/Orion/tree/master/Libraries/BMSerial](https://github.com/KurtE/Orion/tree/master/Libraries/BMSerial)
* LiquidCrystal_I2C [www.archiduino.com/liquidcrystal_i2c-h-library-arduino-ide-1-6-x/](http://www.archiduino.com/liquidcrystal_i2c-h-library-arduino-ide-1-6-x/)
* RoboClaw 2x15A Motor Controller [www.ionmc.com/downloads](http://www.ionmc.com/downloads)
* My own version of UM7 [github.com/raphipaffi/UM7](https://github.com/raphipaffi/UM7)

# Installation of Xubuntu
(Most of these steps were copied from [OculusPrime Setup](http://www.xaxxon.com/documentation/view/oculus-prime-xubuntu-setup))
* Download and install [Xubuntu](https://xubuntu.org/download).
* When asked, set to <b>Login Automatically</b>.
* Connect to Wifi network and set to <b>Automatically connect when available</b>.
* Go to <b>Settings > Software and Updates</b>, change package repository reference to <b>Main Server</b>.
* Update currently installed packages: <br />
```
$ sudo apt get update
```
* Install sshd for remote console connection: <br />
```
$ sudo apt-get install openssh-server
```
* Install a VNC server for remote desktop: <br />
```
$ sudo apt-get install x11vnc
```
* Open <b>Setting > Session and Startup</b>, select tab <b>Application Autostart</b>
* Add a new entry with comand line <i>x11vnc -display :0 -loop -forever -shared</i>
* Make sure that the system has a VIRTUAL display (check using <i>xrandr</i> in console)
* If not, create a .conf file in /usr/share/X11/xorg.conf.d/ with the following content: <br />
```
Section "Device"
    Identifier     "Device0"
    Driver         "intel"
    Option         "VirtualHeads" "1"
EndSection
```
* Add a .sh file to the desktop with following content and make it executable: <br />
```
#! /bin/bash
xrandr --newmode "1920x1080"  173.00  1920 2048 2248 2576  1080 1083 1088 1120 -hsync +vsync
xrandr --addmode VIRTUAL1 1920x1080
xrandr --output VIRTUAL1 --mode 1920x1080
xrandr --output DP2 --off
```
* Go to <b>Settings > Light Locker Settings</b>, turn everything to <b>Off</b>, set sliders to <b>0 minutes/never</b>.
* Go to <b>Settings > Power Manager</b>, uncheck <b>Monitor Power Management Control</b> and go through the various tabs and set all the sliders to <b>0 minutes/never</b>.
* If the system doesn’t shut down cleanly, sometimes the Grub boot manager will wait for a menu selection before booting. To avoid this, open the file <b>/etc/default/grub</b> and add the line: <br />
```
GRUB_RECORDFAIL_TIMEOUT=0
```
* Run: <br />
```
$ sudo update-grub
```
* In case of disk checking on startup, you’ll want the system to repair any problems automatically. Edit the file <b>/etc/default/rcS</b> and change the line containing: `FSCKFIX=no`, to `FSCKFIX=yes`.
* When applications crash, they ask if you want to send a report. Disable this feature by editing the file <b>/etc/default/apport</b> and change the line: `enabled=1` to `enabled=0`.
* Go to <b>Settings > Software and Udates</b> and click on the <b>Updates</b> tab. Switch <b>Automatically Check for Updates</b> to <b>Never</b>
* It should be OK to work on the system remotely from here onward.
* Go to [www.google.com/chrome/browser](http://www.google.com/chrome/browser), download the 64-bit '.deb' package and install it using Software Center.
* Run Google Chrome from the menu once and choose <b>set as default browser</b>.
* Go to [https://code.visualstudio.com/](https://code.visualstudio.com/), download the '.deb' package and install it using Software Center.
* USB ports are only available to root by default in Ubuntu. To give yourself access:
```
$ sudo adduser user_name dialout
```
(Substitute “user_name” with your username. NOTE: reboot is required for this to take effect.)


# ROS Setup
(Most of these steps were copied from [OculusPrime Setup](http://www.xaxxon.com/documentation/view/oculus-prime-ros-installation))
* Install ROS  following the instructions at [http://wiki.ros.org/ROS/Installation](http://wiki.ros.org/ROS/Installation). Choose 'Desktop-Full Install.'
* Install Git: <br />
```
$ sudo apt-get install git
```
* Create a ROS Workspace in your home folder by entering: <br />
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ cd ~/catkin_ws/
$ catkin_make
$ cd ~/catkin_ws/src
```
* Clone the modified depthimage_to_laserscan package: <br />
```
$ git clone https://github.com/xaxxontech/depthimage_to_laserscan.git
```
* Clone the default ROS openni2_camera package: <br />
```
$ git clone https://github.com/ros-drivers/openni2_camera.git
```
* For the Orbbec Astra camera, drivers have to be installed manually: <br />
```
$ mkdir ~/temp/
$ cd ~/temp/
$ wget http://www.xaxxon.com/downloads/orbbec_openni2_files.zip
$ unzip orbbec_openni2_files.zip
$ cd orbbec_openni2_files
$ sudo mv 558-orbbec-usb.rules /etc/udev/rules.d/
$ sudo adduser oculus video
$ sudo mv libopenni2.pc /usr/lib/pkgconfig/
$ cd /usr/lib/
$ sudo mv ~/temp/orbbec_openni2_files/libOpenNI2.so .
$ cd OpenNI2/
$ sudo mv ~/temp/orbbec_openni2_files/* ./Drivers/
```
* Add the new workspace to the default ROS environment by doing: <br />
```
$ echo "source $HOME/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
* Reboot.



# Useful commands
sudo apt-get install ros-jade-<package-name>
git clone https://github.com/<package-name>.git 

rosrun rqt_reconfigure rqt_reconfigure
rqt_console
rqt_graph
rqt_plot

rostopic echo sonar
rostopic hz sonar

roscore
roslaunch roboraider make_map.launch 
roslaunch roboraider navigation.launch
roslaunch roboraider test_odom.launch












